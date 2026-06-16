#!/usr/bin/env python3
"""
Ping1D -> Delphis bridge (adaptive, low-lag, CSV debug logging)

This v3 keeps the same modem packet and GPIO/serial communication path as v2:
- Reads distance/confidence from Blue Robotics Ping1D (brping).
- Sends "$U{addr}{len:02d}{distance_mm},{confidence}" to Delphis modem.
- Pulses the LED for each sent sample.

Changes are limited to measurement handling and background diagnostics:
- The old rolling median is replaced by a close-range physical gate. Normal
  samples pass through immediately; only implausible single-frame jumps are
  held, including high-confidence close-range ghost echoes around 0.6 m.
- Every sample is written to a physical CSV debug log by a background writer
  thread, so disk I/O should not slow the measurement loop.
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import queue
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Deque, Dict, Optional, Tuple

import serial
import RPi.GPIO as GPIO
from brping import Ping1D

try:
    from AltimeterSpeed import AltimeterSpeed as ExternalAltimeterSpeed
except Exception:  # pragma: no cover - used only when the helper is missing
    ExternalAltimeterSpeed = None  # type: ignore[assignment]

# ---------------------------- Constants ---------------------------- #

SOFTWARE_VERSION = "echo_altimeter_v3"

LED_PIN = 12                       # BCM numbering
MODEM_PORT = "/dev/ttyUSB0"
MODEM_BAUD = 9600
TARGET_ADDRESS = 2                 # A002 -> "002" in packet

# Low-lag close-range gate. Normal values pass through immediately.
LOW_CONF_THRESHOLD = 60            # %; used by spike gate
CLOSE_ZONE_MM = 1200               # extra protection below 1.2 m
CLOSE_JUMP_REJECT_MM = 250         # reject single-frame jumps near target
MAX_PHYSICAL_SPEED_MPS = 3.0       # permissive bound for real motion
MIN_JUMP_REJECT_MM = 250           # minimum rate gate, regardless of dt
RECOVERY_COUNT = 4                 # accept a new level if it persists
SPIKE_REJECT_MM: Optional[int] = 1000  # kept for log compatibility with v2

# Adaptive mode thresholds (with hysteresis)
NEAR_UP_M = 4.0
NEAR_DN_M = 3.0
MID_UP_M = 25.0
MID_DN_M = 20.0
MODE_SWITCH_COUNT = 5

# Mode configurations (min_mm, max_mm, gain, ping_interval_ms)
# Kept the same as v2 so the external behavior and mode strategy remain stable.
NEAR_CFG = (300, 5000, 1, 40)      # 0.3-5 m, low gain, about 25 Hz
MID_CFG = (300, 25000, 3, 80)      # 0.3-25 m, mid gain, about 12.5 Hz
FAR_CFG = (300, 60000, 5, 150)     # 0.3-60 m, high gain, about 6-7 Hz

# LED pulse (same pulse length as v2)
LED_PULSE_SEC = 0.01

# Logging
DEFAULT_LOG_DIR = "altimeter_logs"
LOG_QUEUE_MAX_ROWS = 10000
NO_DATA_SLEEP_SEC = 0.02

# ---------------------------- Utilities ---------------------------- #


def setup_logging() -> None:
    """Configure concise, time-stamped console logging (line-buffered)."""
    try:
        sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
    except Exception:
        pass
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
        datefmt="%H:%M:%S",
        handlers=[logging.StreamHandler(sys.stdout)],
    )


def build_packet(address: int, distance_mm: int, confidence: int) -> str:
    """Build "$U{addr}{len:02d}{distance},{confidence}"."""
    payload = f"{distance_mm},{confidence}"
    length = len(payload)
    return f"$U{address:03d}{length:02d}{payload}"


def utc_iso(epoch_s: float) -> str:
    """Return an ISO-8601 UTC timestamp for a Unix epoch value."""
    return datetime.fromtimestamp(epoch_s, tz=timezone.utc).isoformat()


def brief_led_pulse(pin: int, seconds: float) -> None:
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(seconds)
    GPIO.output(pin, GPIO.LOW)


@dataclass
class FilterResult:
    filtered_mm: int
    action: str
    accepted: bool
    delta_mm: int
    jump_limit_mm: int
    reject_count: int


class RealtimeFilter:
    """
    Low-lag physical gate for close-range measurements.

    v2 used a rolling median. That removes isolated spikes, but it also delays
    real changes. Here, the default path is immediate pass-through. A sample is
    held only when it is physically implausible relative to the previous good
    sample, especially in the close zone where ghost echoes are common.
    """

    def __init__(
        self,
        low_conf: int,
        close_zone_mm: int,
        close_jump_mm: int,
        max_speed_mps: float,
        min_jump_mm: int,
        recovery_count: int,
    ):
        self._low_conf = low_conf
        self._close_zone_mm = close_zone_mm
        self._close_jump_mm = close_jump_mm
        self._max_speed_mps = max_speed_mps
        self._min_jump_mm = min_jump_mm
        self._recovery_count = max(1, recovery_count)
        self._last_good_mm: Optional[int] = None
        self._last_good_t: Optional[float] = None
        self._last_rejected_mm: Optional[int] = None
        self._reject_count = 0

    def update(
        self,
        distance_mm: int,
        confidence: int,
        monotonic_s: float,
    ) -> FilterResult:
        if self._last_good_mm is None or self._last_good_t is None:
            self._accept(distance_mm, monotonic_s)
            return FilterResult(
                filtered_mm=distance_mm,
                action="accept_initial",
                accepted=True,
                delta_mm=0,
                jump_limit_mm=self._min_jump_mm,
                reject_count=0,
            )

        dt = max(monotonic_s - self._last_good_t, 0.001)
        jump_limit_mm = max(
            self._min_jump_mm,
            int(self._max_speed_mps * 1000.0 * dt),
        )
        delta_mm = distance_mm - self._last_good_mm
        abs_delta_mm = abs(delta_mm)
        close_zone = (
            self._last_good_mm <= self._close_zone_mm
            or distance_mm <= self._close_zone_mm
        )

        reject_reason: Optional[str] = None
        if close_zone and abs_delta_mm > self._close_jump_mm:
            reject_reason = "hold_close_jump"
        elif confidence < self._low_conf and abs_delta_mm > jump_limit_mm:
            reject_reason = "hold_low_conf_jump"
        elif (
            SPIKE_REJECT_MM is not None
            and abs_delta_mm > SPIKE_REJECT_MM
            and confidence < 85
        ):
            reject_reason = "hold_large_jump"

        if reject_reason is None:
            self._accept(distance_mm, monotonic_s)
            return FilterResult(
                filtered_mm=distance_mm,
                action="accept",
                accepted=True,
                delta_mm=delta_mm,
                jump_limit_mm=jump_limit_mm,
                reject_count=0,
            )

        # Do not stay locked forever: if a new level repeats consistently,
        # accept it as a real movement or a real new bottom lock.
        if (
            self._last_rejected_mm is not None
            and abs(distance_mm - self._last_rejected_mm) <= self._close_jump_mm
        ):
            self._reject_count += 1
        else:
            self._reject_count = 1
            self._last_rejected_mm = distance_mm

        if self._reject_count >= self._recovery_count and confidence >= self._low_conf:
            self._accept(distance_mm, monotonic_s)
            return FilterResult(
                filtered_mm=distance_mm,
                action="accept_after_repeated_jump",
                accepted=True,
                delta_mm=delta_mm,
                jump_limit_mm=jump_limit_mm,
                reject_count=self._reject_count,
            )

        return FilterResult(
            filtered_mm=self._last_good_mm,
            action=reject_reason,
            accepted=False,
            delta_mm=delta_mm,
            jump_limit_mm=jump_limit_mm,
            reject_count=self._reject_count,
        )

    def _accept(self, distance_mm: int, monotonic_s: float) -> None:
        self._last_good_mm = distance_mm
        self._last_good_t = monotonic_s
        self._last_rejected_mm = None
        self._reject_count = 0


class SpeedEstimator:
    """Track signed speed of the transmitted distance signal."""

    def __init__(self, listlen: int = 40):
        self._external = ExternalAltimeterSpeed() if ExternalAltimeterSpeed else None
        self._times: Deque[float] = deque(maxlen=listlen)
        self._distances: Deque[float] = deque(maxlen=listlen)

    def update(self, time_s: float, distance_m: float) -> Tuple[float, float]:
        if self._external is not None:
            self._external.set_time_depth(time_s, distance_m)
            ispeed, aspeed = self._external.get_speeds()
            return float(ispeed), float(aspeed)

        # Fallback if AltimeterSpeed.py or numpy is not available. This keeps
        # the bridge running and logs simple instantaneous/average speeds.
        self._times.append(time_s)
        self._distances.append(distance_m)
        if len(self._times) < 2:
            return 0.0, 0.0
        dt_i = self._times[-1] - self._times[-2]
        ispeed = 0.0 if dt_i <= 0 else (self._distances[-1] - self._distances[-2]) / dt_i
        dt_a = self._times[-1] - self._times[0]
        aspeed = 0.0 if dt_a <= 0 else (self._distances[-1] - self._distances[0]) / dt_a
        return ispeed, aspeed


class CsvDebugLogger:
    """Background CSV writer for physical debug logs."""

    FIELDNAMES = [
        "software_version",
        "iso_time_utc",
        "epoch_time_s",
        "monotonic_time_s",
        "elapsed_s",
        "loop_dt_s",
        "sensor_read_duration_s",
        "serial_write_duration_s",
        "loop_duration_s",
        "device",
        "baudrate",
        "udp",
        "raw_distance_mm",
        "raw_distance_m",
        "confidence",
        "filtered_distance_mm",
        "filtered_distance_m",
        "output_distance_mm",
        "output_distance_m",
        "filter_action",
        "filter_accepted",
        "filter_delta_mm",
        "filter_jump_limit_mm",
        "filter_reject_count",
        "mode",
        "mode_switch_counter",
        "mode_min_mm",
        "mode_max_mm",
        "gain_setting",
        "ping_interval_ms",
        "range_set_ok",
        "interval_set_ok",
        "gain_set_ok",
        "mode_apply_errors",
        "instant_speed_m_s",
        "average_speed_m_s",
        "instant_speed_m_min",
        "average_speed_m_min",
        "instant_descent_rate_m_min",
        "average_descent_rate_m_min",
        "low_conf_threshold",
        "close_zone_mm",
        "close_jump_reject_mm",
        "max_physical_speed_mps",
        "min_jump_reject_mm",
        "recovery_count",
        "spike_reject_mm",
        "near_up_m",
        "near_down_m",
        "mid_up_m",
        "mid_down_m",
        "mode_switch_count",
        "packet",
        "target_address",
        "modem_port",
        "modem_baud",
        "modem_write_ok",
        "modem_error",
        "sensor_error",
        "data_keys",
        "no_data_streak",
        "led_pulse_sec",
        "logger_queue_size",
        "logger_dropped_rows",
    ]

    def __init__(self, log_dir: str):
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.path = os.path.abspath(
            os.path.join(log_dir, f"altimeter_debug_{stamp}.csv")
        )
        self._queue: queue.Queue[Dict[str, Any]] = queue.Queue(
            maxsize=LOG_QUEUE_MAX_ROWS
        )
        self._stop = threading.Event()
        self._dropped_rows = 0
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    @property
    def dropped_rows(self) -> int:
        return self._dropped_rows

    def write(self, row: Dict[str, Any]) -> None:
        row = dict(row)
        row["logger_queue_size"] = self._queue.qsize()
        row["logger_dropped_rows"] = self._dropped_rows
        try:
            self._queue.put_nowait(row)
        except queue.Full:
            self._dropped_rows += 1

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)

    def _worker(self) -> None:
        with open(self.path, "w", newline="", buffering=1) as f:
            writer = csv.DictWriter(f, fieldnames=self.FIELDNAMES)
            writer.writeheader()
            f.flush()
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    row = self._queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                writer.writerow({name: row.get(name, "") for name in self.FIELDNAMES})
                f.flush()
                self._queue.task_done()


# ----------------------- Adaptive Mode Controller ----------------------- #


class AdaptiveController:
    """
    Manages NEAR/MID/FAR modes with hysteresis.
    Applies new (range, gain, interval) only when a mode transition occurs.
    """

    class Mode:
        NEAR = "NEAR"
        MID = "MID"
        FAR = "FAR"

    def __init__(self, ping: Ping1D):
        self.ping = ping
        self.mode = self.Mode.NEAR
        self._switch_counter = 0
        self._last_applied: Optional[Tuple[int, int, int, int]] = None
        self.range_set_ok = ""
        self.interval_set_ok = ""
        self.gain_set_ok = ""
        self.mode_apply_errors = ""
        self.apply_mode(self.mode, force=True)

    @property
    def switch_counter(self) -> int:
        return self._switch_counter

    def current_config(self) -> Tuple[int, int, int, int]:
        return {
            self.Mode.NEAR: NEAR_CFG,
            self.Mode.MID: MID_CFG,
            self.Mode.FAR: FAR_CFG,
        }[self.mode]

    def classify(self, meters: float) -> str:
        if self.mode == self.Mode.NEAR:
            if meters > NEAR_UP_M:
                return self.Mode.MID
            return self.Mode.NEAR
        if self.mode == self.Mode.MID:
            if meters > MID_UP_M:
                return self.Mode.FAR
            if meters < NEAR_DN_M:
                return self.Mode.NEAR
            return self.Mode.MID
        if meters < MID_DN_M:
            return self.Mode.MID
        return self.Mode.FAR

    def step(self, meters: float) -> None:
        """Advance state machine using hysteresis + consecutive-sample voting."""
        desired = self.classify(meters)
        if desired == self.mode:
            self._switch_counter = 0
            return
        self._switch_counter += 1
        if self._switch_counter >= MODE_SWITCH_COUNT:
            self.mode = desired
            self._switch_counter = 0
            self.apply_mode(self.mode, force=False)

    def apply_mode(self, mode: str, force: bool = False) -> None:
        cfg = {
            self.Mode.NEAR: NEAR_CFG,
            self.Mode.MID: MID_CFG,
            self.Mode.FAR: FAR_CFG,
        }[mode]
        min_mm, max_mm, gain, interval_ms = cfg

        if not force and self._last_applied == cfg:
            return

        errors = []
        self.range_set_ok = ""
        self.interval_set_ok = ""
        self.gain_set_ok = ""

        try:
            self.range_set_ok = repr(self.ping.set_range(min_mm, max_mm))
        except Exception as exc:
            self.range_set_ok = "False"
            errors.append(f"set_range:{exc}")
            logging.debug("set_range unsupported; continuing")
        try:
            self.interval_set_ok = repr(self.ping.set_ping_interval(interval_ms))
        except Exception as exc:
            self.interval_set_ok = "False"
            errors.append(f"set_ping_interval:{exc}")
            logging.debug("set_ping_interval unsupported; continuing")
        try:
            self.gain_set_ok = repr(self.ping.set_gain_setting(gain, verify=True))
        except Exception as exc:
            self.gain_set_ok = "False"
            errors.append(f"set_gain_setting:{exc}")
            logging.debug("set_gain_setting unsupported; continuing")

        self.mode_apply_errors = ";".join(errors)
        self._last_applied = cfg
        logging.info(
            "Mode -> %s  range=%0.2f-%0.2f m  gain=%s  interval=%d ms",
            mode,
            min_mm / 1000.0,
            max_mm / 1000.0,
            gain,
            interval_ms,
        )


# ---------------------------- Main Program ---------------------------- #


def _base_log_row(
    args: argparse.Namespace,
    controller: AdaptiveController,
    start_epoch: float,
    start_monotonic: float,
    loop_start: float,
    previous_loop_start: Optional[float],
) -> Dict[str, Any]:
    epoch_s = time.time()
    monotonic_s = time.monotonic()
    min_mm, max_mm, gain, interval_ms = controller.current_config()
    return {
        "software_version": SOFTWARE_VERSION,
        "iso_time_utc": utc_iso(epoch_s),
        "epoch_time_s": f"{epoch_s:.6f}",
        "monotonic_time_s": f"{monotonic_s:.6f}",
        "elapsed_s": f"{monotonic_s - start_monotonic:.6f}",
        "loop_dt_s": "" if previous_loop_start is None else f"{loop_start - previous_loop_start:.6f}",
        "device": args.device or "",
        "baudrate": args.baudrate,
        "udp": args.udp or "",
        "mode": controller.mode,
        "mode_switch_counter": controller.switch_counter,
        "mode_min_mm": min_mm,
        "mode_max_mm": max_mm,
        "gain_setting": gain,
        "ping_interval_ms": interval_ms,
        "range_set_ok": controller.range_set_ok,
        "interval_set_ok": controller.interval_set_ok,
        "gain_set_ok": controller.gain_set_ok,
        "mode_apply_errors": controller.mode_apply_errors,
        "low_conf_threshold": LOW_CONF_THRESHOLD,
        "close_zone_mm": CLOSE_ZONE_MM,
        "close_jump_reject_mm": CLOSE_JUMP_REJECT_MM,
        "max_physical_speed_mps": MAX_PHYSICAL_SPEED_MPS,
        "min_jump_reject_mm": MIN_JUMP_REJECT_MM,
        "recovery_count": RECOVERY_COUNT,
        "spike_reject_mm": "" if SPIKE_REJECT_MM is None else SPIKE_REJECT_MM,
        "near_up_m": NEAR_UP_M,
        "near_down_m": NEAR_DN_M,
        "mid_up_m": MID_UP_M,
        "mid_down_m": MID_DN_M,
        "mode_switch_count": MODE_SWITCH_COUNT,
        "target_address": TARGET_ADDRESS,
        "modem_port": MODEM_PORT,
        "modem_baud": MODEM_BAUD,
        "led_pulse_sec": LED_PULSE_SEC,
    }


def run(device: Optional[str], baud: int, udp: Optional[str], log_dir: str) -> int:
    setup_logging()

    # Package args for logging without changing the existing run signature style.
    args = argparse.Namespace(device=device, baudrate=baud, udp=udp)

    # GPIO setup
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)

    # Small grace period (hardware settle / boot races)
    logging.info("Starting in 3 seconds...")
    time.sleep(3)

    # Connect Ping1D
    ping = Ping1D()
    try:
        if device:
            logging.info("Opening Ping1D on %s @ %d bps", device, baud)
            ping.connect_serial(device, baud)
        else:
            host, port = udp.split(":")
            logging.info("Opening Ping1D via UDP %s:%s", host, port)
            ping.connect_udp(host, int(port))
    except Exception:
        logging.error("Altimeter connection failed:\n%s", traceback.format_exc())
        return 2

    if ping.initialize() is False:
        logging.error("Failed to initialize Ping1D")
        return 3

    # Apply initial NEAR mode; will adapt as distance changes
    controller = AdaptiveController(ping)

    # Open modem (write-only, short timeout)
    modem: Optional[serial.Serial] = None
    try:
        modem = serial.Serial(
            MODEM_PORT,
            MODEM_BAUD,
            timeout=0,
            write_timeout=0.2,
        )
    except Exception:
        logging.error("Failed to open modem port %s:\n%s", MODEM_PORT, traceback.format_exc())
        return 4

    # Filters, speed, and background CSV log
    filt = RealtimeFilter(
        low_conf=LOW_CONF_THRESHOLD,
        close_zone_mm=CLOSE_ZONE_MM,
        close_jump_mm=CLOSE_JUMP_REJECT_MM,
        max_speed_mps=MAX_PHYSICAL_SPEED_MPS,
        min_jump_mm=MIN_JUMP_REJECT_MM,
        recovery_count=RECOVERY_COUNT,
    )
    speed = SpeedEstimator()
    csv_log = CsvDebugLogger(log_dir)

    start_epoch = time.time()
    start_monotonic = time.monotonic()
    previous_loop_start: Optional[float] = None
    no_data_streak = 0

    logging.info("-" * 42)
    logging.info("Streaming Ping1D -> modem (adaptive v3). Ctrl+C to exit.")
    logging.info("Debug CSV log: %s", csv_log.path)
    logging.info("-" * 42)

    try:
        while True:
            loop_start = time.monotonic()
            row = _base_log_row(
                args=args,
                controller=controller,
                start_epoch=start_epoch,
                start_monotonic=start_monotonic,
                loop_start=loop_start,
                previous_loop_start=previous_loop_start,
            )
            previous_loop_start = loop_start

            sensor_start = time.monotonic()
            data = ping.get_distance()
            sensor_duration = time.monotonic() - sensor_start
            row["sensor_read_duration_s"] = f"{sensor_duration:.6f}"

            if not data:
                no_data_streak += 1
                row.update(
                    {
                        "sensor_error": "no_data",
                        "no_data_streak": no_data_streak,
                        "loop_duration_s": f"{time.monotonic() - loop_start:.6f}",
                    }
                )
                csv_log.write(row)
                logging.warning("No data from altimeter")
                time.sleep(NO_DATA_SLEEP_SEC)
                continue

            no_data_streak = 0
            data_keys = ";".join(sorted(str(k) for k in data.keys()))
            distance_mm = max(0, int(data["distance"]))
            confidence = int(data["confidence"])

            now_monotonic = time.monotonic()
            filter_result = filt.update(distance_mm, confidence, now_monotonic)
            filtered_mm = filter_result.filtered_mm

            # Drive adaptive controller using filtered meters.
            meters = filtered_mm / 1000.0
            controller.step(meters)

            # Clamp output to the currently configured mode range.
            min_mm, max_mm, gain, interval_ms = controller.current_config()
            clamped_mm = max(min_mm, min(max_mm, filtered_mm))
            packet = build_packet(TARGET_ADDRESS, clamped_mm, confidence)

            # Speed is computed from the exact distance being transmitted.
            epoch_for_speed = time.time()
            ispeed_m_s, aspeed_m_s = speed.update(epoch_for_speed, clamped_mm / 1000.0)
            ispeed_m_min = ispeed_m_s * 60.0
            aspeed_m_min = aspeed_m_s * 60.0
            instant_descent_m_min = max(0.0, -ispeed_m_min)
            average_descent_m_min = max(0.0, -aspeed_m_min)

            # Immediate console + modem write. Packet format is unchanged.
            print(f"Sent: {packet}", flush=True)
            modem_write_ok = True
            modem_error = ""
            serial_start = time.monotonic()
            try:
                modem.write(packet.encode("ascii", errors="ignore"))
                modem.flush()
            except Exception:
                modem_write_ok = False
                modem_error = traceback.format_exc()
                logging.error("Serial write error:\n%s", modem_error)
            serial_duration = time.monotonic() - serial_start

            # Same LED pulse behavior as v2.
            brief_led_pulse(LED_PIN, LED_PULSE_SEC)

            row.update(
                {
                    "serial_write_duration_s": f"{serial_duration:.6f}",
                    "raw_distance_mm": distance_mm,
                    "raw_distance_m": f"{distance_mm / 1000.0:.6f}",
                    "confidence": confidence,
                    "filtered_distance_mm": filtered_mm,
                    "filtered_distance_m": f"{filtered_mm / 1000.0:.6f}",
                    "output_distance_mm": clamped_mm,
                    "output_distance_m": f"{clamped_mm / 1000.0:.6f}",
                    "filter_action": filter_result.action,
                    "filter_accepted": int(filter_result.accepted),
                    "filter_delta_mm": filter_result.delta_mm,
                    "filter_jump_limit_mm": filter_result.jump_limit_mm,
                    "filter_reject_count": filter_result.reject_count,
                    "mode": controller.mode,
                    "mode_switch_counter": controller.switch_counter,
                    "mode_min_mm": min_mm,
                    "mode_max_mm": max_mm,
                    "gain_setting": gain,
                    "ping_interval_ms": interval_ms,
                    "range_set_ok": controller.range_set_ok,
                    "interval_set_ok": controller.interval_set_ok,
                    "gain_set_ok": controller.gain_set_ok,
                    "mode_apply_errors": controller.mode_apply_errors,
                    "instant_speed_m_s": f"{ispeed_m_s:.6f}",
                    "average_speed_m_s": f"{aspeed_m_s:.6f}",
                    "instant_speed_m_min": f"{ispeed_m_min:.6f}",
                    "average_speed_m_min": f"{aspeed_m_min:.6f}",
                    "instant_descent_rate_m_min": f"{instant_descent_m_min:.6f}",
                    "average_descent_rate_m_min": f"{average_descent_m_min:.6f}",
                    "packet": packet,
                    "modem_write_ok": int(modem_write_ok),
                    "modem_error": modem_error,
                    "sensor_error": "",
                    "data_keys": data_keys,
                    "no_data_streak": no_data_streak,
                    "loop_duration_s": f"{time.monotonic() - loop_start:.6f}",
                }
            )
            csv_log.write(row)

            # No extra sleep: Ping1D interval governs rate.

    except KeyboardInterrupt:
        logging.info("Terminated by user")
        return 0
    except Exception:
        logging.error("Runtime error:\n%s", traceback.format_exc())
        return 5
    finally:
        try:
            csv_log.close()
            logging.info("Closed debug CSV log: %s", csv_log.path)
        except Exception:
            pass
        try:
            if modem is not None:
                modem.close()
        except Exception:
            pass
        GPIO.cleanup()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ping1D -> Delphis serial bridge (adaptive, low-lag v3)."
    )
    parser.add_argument(
        "--device",
        type=str,
        default="/dev/serial0",
        help="Ping serial device, e.g. /dev/serial0 or /dev/ttyUSB0",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Ping serial baudrate (default: 115200)",
    )
    parser.add_argument(
        "--udp",
        type=str,
        help="Ping UDP server as IP:PORT (use instead of --device/--baudrate)",
    )
    parser.add_argument(
        "--log-dir",
        type=str,
        default=DEFAULT_LOG_DIR,
        help=f"Directory for CSV debug logs (default: {DEFAULT_LOG_DIR})",
    )
    return parser.parse_args()


if __name__ == "__main__":
    # Retry loop (matches the original behavior)
    while True:
        cli_args = parse_args()
        rc = run(
            device=cli_args.device,
            baud=cli_args.baudrate,
            udp=cli_args.udp,
            log_dir=cli_args.log_dir,
        )
        if rc == 0:
            sys.exit(0)
        print("Retrying in 20 seconds...", flush=True)
        time.sleep(20)
