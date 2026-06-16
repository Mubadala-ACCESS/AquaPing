#!/usr/bin/env python3
"""
Ping1D -> Delphis bridge (adaptive, low-lag, CSV debug logging)

This v4 keeps the same modem packet and GPIO/serial communication path as v2:
- Reads distance/confidence from Blue Robotics Ping1D (brping).
- Sends "$U{addr}{len:02d}{distance_mm},{confidence}" to Delphis modem.
- Pulses the LED for each sent sample.

Measurement/debugging additions:
- Adds a VERY_NEAR adaptive mode for close work: nominal 0.10-1.10 m.
  Note: Blue Robotics lists the typical minimum range as 0.30 m, so the code
  attempts 0.10 m but logs a below_typical_min flag for anything below 0.30 m.
- Uses explicit scan_start + scan_length values for Ping1D manual range setup.
- Keeps live output low-lag: normal samples pass through immediately; only
  physically implausible single-frame jumps are held.
- Logs raw, filtered, and transmitted distances, confidence, gain, interval,
  scan window, speed, descent rate, quality flags, and timing to a physical
  CSV file using a background writer thread.
- Logs beam/range motion diagnostics that can reveal current-induced swing or
  non-vertical descent symptoms. A single-beam altimeter cannot measure
  horizontal current or tilt directly without an IMU/current meter; the log
  therefore marks these as beam-only indicators rather than corrected vertical
  truth.
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import os
import queue
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from statistics import median
from typing import Any, Deque, Dict, Iterable, List, Optional, Tuple

import serial
import RPi.GPIO as GPIO
from brping import Ping1D

try:
    from AltimeterSpeed import AltimeterSpeed as ExternalAltimeterSpeed
except Exception:  # pragma: no cover - used only when the helper is missing
    ExternalAltimeterSpeed = None  # type: ignore[assignment]

# ---------------------------- Constants ---------------------------- #

SOFTWARE_VERSION = "echo_altimeter_v3_very_near"

LED_PIN = 12                       # BCM numbering
MODEM_PORT = "/dev/ttyUSB0"
MODEM_BAUD = 9600
TARGET_ADDRESS = 2                 # A002 -> "002" in packet

# Ping1D / sonar physical assumptions
ABSOLUTE_MIN_RANGE_MM = 100        # requested experimental close limit
TYPICAL_MIN_RANGE_MM = 300         # Blue Robotics typical minimum range
PING_PROTOCOL_MIN_SCAN_LENGTH_MM = 1000

# Low-lag close-range gate. Normal values pass through immediately.
LOW_CONF_THRESHOLD = 60            # %; used by spike gate
VERY_NEAR_LOW_CONF_THRESHOLD = 70  # closer ranges need cleaner locks
VERY_NEAR_ZONE_MM = 1100
CLOSE_ZONE_MM = 1500               # extra protection below 1.5 m
VERY_NEAR_JUMP_REJECT_MM = 120     # strict single-frame jump rejection
CLOSE_JUMP_REJECT_MM = 250         # reject single-frame jumps near target
MID_JUMP_REJECT_MM = 1000
MAX_PHYSICAL_SPEED_MPS = 3.0       # permissive bound for real motion
MIN_JUMP_REJECT_MM = 180           # minimum rate gate, regardless of dt
RECOVERY_COUNT = 4                 # accept a new level if it persists
SPIKE_REJECT_MM: Optional[int] = 1000  # kept for log compatibility with v2

# Adaptive mode thresholds (with hysteresis)
VERY_NEAR_UP_M = 0.95              # leave VERY_NEAR above this level
VERY_NEAR_DN_M = 0.75              # enter VERY_NEAR below this level
NEAR_UP_M = 4.0
NEAR_DN_M = 3.0
MID_UP_M = 25.0
MID_DN_M = 20.0
MODE_SWITCH_COUNT = 5
VERY_NEAR_SWITCH_COUNT = 2
NO_DATA_MODE_ESCAPE_COUNT = 8

# Mode configurations.
# scan_start and scan_length are the values sent to Ping1D set_range().
# output_min/output_max are the local clamp bounds for the Delphis packet.
# Gain indices follow Ping1D docs: 0:0.6, 1:1.8, 2:5.5, 3:12.9, 4:30.2,
# 5:66.1, 6:144. VERY_NEAR uses lowest gain to reduce near-field ringing.

@dataclass(frozen=True)
class ModeConfig:
    scan_start_mm: int
    output_min_mm: int
    output_max_mm: int
    gain: int
    interval_ms: int

    @property
    def scan_length_mm(self) -> int:
        length = self.output_max_mm - self.scan_start_mm
        return max(PING_PROTOCOL_MIN_SCAN_LENGTH_MM, length)


VERY_NEAR_CFG = ModeConfig(100, 100, 1100, 0, 30)
NEAR_CFG = ModeConfig(100, 100, 5000, 1, 40)
MID_CFG = ModeConfig(300, 300, 25000, 3, 80)
FAR_CFG = ModeConfig(300, 300, 60000, 5, 150)

# LED pulse (same pulse length as v2)
LED_PULSE_SEC = 0.01

# Logging
DEFAULT_LOG_DIR = "altimeter_logs"
LOG_QUEUE_MAX_ROWS = 10000
NO_DATA_SLEEP_SEC = 0.02

# Signal-quality heuristics for logs only. These do not change the packet
# unless the physical gate explicitly rejects a sample.
QUALITY_WINDOW = 12
NOISY_RANGE_STD_MM = 80
NOISY_RANGE_MAD_MM = 60
FAST_ACCEL_M_S2 = 1.5
SIGN_CHANGE_WINDOW = 8
SIGN_CHANGE_LIMIT = 3

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


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def sample_std(values: Iterable[float]) -> float:
    vals = list(values)
    if len(vals) < 2:
        return 0.0
    mean = sum(vals) / len(vals)
    var = sum((x - mean) ** 2 for x in vals) / (len(vals) - 1)
    return math.sqrt(var)


def mad(values: Iterable[float]) -> float:
    vals = list(values)
    if not vals:
        return 0.0
    med = median(vals)
    return float(median(abs(x - med) for x in vals))


def sign_change_count(values: List[float], deadband: float = 0.01) -> int:
    """Count sign changes while ignoring very small values."""
    signs: List[int] = []
    for val in values:
        if abs(val) <= deadband:
            continue
        signs.append(1 if val > 0 else -1)
    if len(signs) < 2:
        return 0
    return sum(1 for prev, cur in zip(signs, signs[1:]) if prev != cur)


def get_optional(data: Dict[str, Any], key: str) -> str:
    value = data.get(key, "")
    return "" if value is None else str(value)


# ---------------------------- Filtering ---------------------------- #


@dataclass
class FilterResult:
    filtered_mm: int
    action: str
    accepted: bool
    delta_mm: int
    jump_limit_mm: int
    reject_count: int
    zone: str
    confidence_gate: int


class RealtimeFilter:
    """
    Low-lag physical gate for close-range measurements.

    The default path is immediate pass-through. A sample is held only when it is
    physically implausible relative to the previous accepted sample, especially
    in VERY_NEAR/CLOSE zones where ghost echoes and transducer ring-down are
    likely. Repeated new levels are accepted so the filter cannot stay locked.
    """

    def __init__(
        self,
        low_conf: int,
        very_near_low_conf: int,
        very_near_zone_mm: int,
        close_zone_mm: int,
        very_near_jump_mm: int,
        close_jump_mm: int,
        mid_jump_mm: int,
        max_speed_mps: float,
        min_jump_mm: int,
        recovery_count: int,
    ):
        self._low_conf = low_conf
        self._very_near_low_conf = very_near_low_conf
        self._very_near_zone_mm = very_near_zone_mm
        self._close_zone_mm = close_zone_mm
        self._very_near_jump_mm = very_near_jump_mm
        self._close_jump_mm = close_jump_mm
        self._mid_jump_mm = mid_jump_mm
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
        mode: str,
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
                zone="initial",
                confidence_gate=self._low_conf,
            )

        dt = max(monotonic_s - self._last_good_t, 0.001)
        rate_limit_mm = int(self._max_speed_mps * 1000.0 * dt)
        jump_limit_mm = max(self._min_jump_mm, rate_limit_mm)
        delta_mm = distance_mm - self._last_good_mm
        abs_delta_mm = abs(delta_mm)
        min_seen = min(self._last_good_mm, distance_mm)
        max_seen = max(self._last_good_mm, distance_mm)

        if mode == "VERY_NEAR" or min_seen <= self._very_near_zone_mm:
            zone = "very_near"
            zone_jump_mm = self._very_near_jump_mm
            confidence_gate = self._very_near_low_conf
        elif min_seen <= self._close_zone_mm:
            zone = "close"
            zone_jump_mm = self._close_jump_mm
            confidence_gate = self._low_conf
        else:
            zone = "normal"
            zone_jump_mm = self._mid_jump_mm
            confidence_gate = self._low_conf

        reject_reason: Optional[str] = None

        # Values below the experimental absolute floor are not sent as-is.
        if distance_mm < ABSOLUTE_MIN_RANGE_MM:
            reject_reason = "hold_below_absolute_min"

        # Close-range false locks often show up as a one-frame jump. Apply this
        # rule regardless of confidence, because false close echoes can be
        # reported with high confidence.
        elif zone in ("very_near", "close") and abs_delta_mm > zone_jump_mm:
            reject_reason = f"hold_{zone}_jump"

        # For non-close measurements, be permissive unless confidence drops.
        elif confidence < confidence_gate and abs_delta_mm > jump_limit_mm:
            reject_reason = "hold_low_conf_jump"

        elif (
            SPIKE_REJECT_MM is not None
            and abs_delta_mm > SPIKE_REJECT_MM
            and confidence < 85
        ):
            reject_reason = "hold_large_jump"

        # If the new value jumps from very close to a much larger number, it is
        # often a wall/sidelobe/multiple reflection. Hold it once unless it
        # repeats, so real movement is still accepted quickly.
        elif self._last_good_mm <= self._very_near_zone_mm and max_seen > CLOSE_ZONE_MM:
            reject_reason = "hold_escape_from_very_near"

        if reject_reason is None:
            self._accept(distance_mm, monotonic_s)
            return FilterResult(
                filtered_mm=distance_mm,
                action="accept",
                accepted=True,
                delta_mm=delta_mm,
                jump_limit_mm=jump_limit_mm,
                reject_count=0,
                zone=zone,
                confidence_gate=confidence_gate,
            )

        # Do not stay locked forever: if a new level repeats consistently,
        # accept it as a real movement or a real new bottom lock.
        if (
            self._last_rejected_mm is not None
            and abs(distance_mm - self._last_rejected_mm) <= max(zone_jump_mm, self._min_jump_mm)
        ):
            self._reject_count += 1
        else:
            self._reject_count = 1
            self._last_rejected_mm = distance_mm

        if self._reject_count >= self._recovery_count and confidence >= confidence_gate:
            self._accept(distance_mm, monotonic_s)
            return FilterResult(
                filtered_mm=distance_mm,
                action="accept_after_repeated_jump",
                accepted=True,
                delta_mm=delta_mm,
                jump_limit_mm=jump_limit_mm,
                reject_count=self._reject_count,
                zone=zone,
                confidence_gate=confidence_gate,
            )

        return FilterResult(
            filtered_mm=self._last_good_mm,
            action=reject_reason,
            accepted=False,
            delta_mm=delta_mm,
            jump_limit_mm=jump_limit_mm,
            reject_count=self._reject_count,
            zone=zone,
            confidence_gate=confidence_gate,
        )

    def _accept(self, distance_mm: int, monotonic_s: float) -> None:
        self._last_good_mm = distance_mm
        self._last_good_t = monotonic_s
        self._last_rejected_mm = None
        self._reject_count = 0


# ---------------------------- Speed / quality ---------------------------- #


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


@dataclass
class QualityMetrics:
    raw_instant_speed_m_s: float = 0.0
    filtered_instant_speed_m_s: float = 0.0
    output_instant_speed_m_s: float = 0.0
    output_accel_m_s2: float = 0.0
    rolling_output_std_mm: float = 0.0
    rolling_output_mad_mm: float = 0.0
    rolling_confidence_mean: float = 0.0
    rolling_confidence_min: float = 0.0
    residual_to_recent_median_mm: float = 0.0
    speed_sign_changes: int = 0
    possible_current_or_swing_flag: int = 0
    below_typical_min_flag: int = 0
    near_absolute_min_flag: int = 0
    quality_flags: str = "OK"
    motion_note: str = "single_beam_range_only_no_horizontal_current_or_tilt_sensor"


class SignalQualityMonitor:
    """
    Computes diagnostics that help interpret accuracy.

    These are log-only diagnostics. They do not correct vertical distance,
    because a single-beam altimeter cannot distinguish vertical lowering from
    tilt, lateral current drift, or swing without an IMU/current meter.
    """

    def __init__(self, window: int):
        self._window = max(3, window)
        self._samples: Deque[Tuple[float, int, int, int, int, float]] = deque(
            maxlen=self._window
        )
        self._last: Optional[Tuple[float, int, int, int, float]] = None

    def update(
        self,
        monotonic_s: float,
        raw_mm: int,
        filtered_mm: int,
        output_mm: int,
        confidence: int,
        filter_action: str,
    ) -> QualityMetrics:
        raw_speed = 0.0
        filtered_speed = 0.0
        output_speed = 0.0
        accel = 0.0

        if self._last is not None:
            last_t, last_raw, last_filtered, last_output, last_output_speed = self._last
            dt = max(monotonic_s - last_t, 0.001)
            raw_speed = ((raw_mm - last_raw) / 1000.0) / dt
            filtered_speed = ((filtered_mm - last_filtered) / 1000.0) / dt
            output_speed = ((output_mm - last_output) / 1000.0) / dt
            accel = (output_speed - last_output_speed) / dt

        self._last = (monotonic_s, raw_mm, filtered_mm, output_mm, output_speed)
        self._samples.append((monotonic_s, raw_mm, filtered_mm, output_mm, confidence, output_speed))

        outputs = [float(sample[3]) for sample in self._samples]
        confs = [float(sample[4]) for sample in self._samples]
        speeds = [float(sample[5]) for sample in self._samples]
        med_output = median(outputs) if outputs else float(output_mm)

        rolling_std = sample_std(outputs)
        rolling_mad = mad(outputs)
        confidence_mean = sum(confs) / len(confs) if confs else 0.0
        confidence_min = min(confs) if confs else 0.0
        residual = float(output_mm) - float(med_output)
        sign_changes = sign_change_count(speeds[-SIGN_CHANGE_WINDOW:])

        flags: List[str] = []
        below_typical = int(output_mm < TYPICAL_MIN_RANGE_MM)
        near_abs_min = int(output_mm <= ABSOLUTE_MIN_RANGE_MM + 20)

        if confidence < LOW_CONF_THRESHOLD:
            flags.append("LOW_CONFIDENCE")
        if below_typical:
            flags.append("BELOW_TYPICAL_MIN_RANGE")
        if near_abs_min:
            flags.append("NEAR_ABSOLUTE_MIN_RANGE")
        if rolling_std > NOISY_RANGE_STD_MM:
            flags.append("NOISY_RANGE_STD")
        if rolling_mad > NOISY_RANGE_MAD_MM:
            flags.append("NOISY_RANGE_MAD")
        if abs(accel) > FAST_ACCEL_M_S2:
            flags.append("FAST_RANGE_ACCELERATION")
        if filter_action.startswith("hold_"):
            flags.append("FILTER_HOLDING_SAMPLE")
        if sign_changes >= SIGN_CHANGE_LIMIT and (rolling_std > 40 or rolling_mad > 30):
            flags.append("POSSIBLE_CURRENT_SWING_OR_NONVERTICAL_DESCENT")

        possible_current_or_swing = int(
            "POSSIBLE_CURRENT_SWING_OR_NONVERTICAL_DESCENT" in flags
        )

        return QualityMetrics(
            raw_instant_speed_m_s=raw_speed,
            filtered_instant_speed_m_s=filtered_speed,
            output_instant_speed_m_s=output_speed,
            output_accel_m_s2=accel,
            rolling_output_std_mm=rolling_std,
            rolling_output_mad_mm=rolling_mad,
            rolling_confidence_mean=confidence_mean,
            rolling_confidence_min=confidence_min,
            residual_to_recent_median_mm=residual,
            speed_sign_changes=sign_changes,
            possible_current_or_swing_flag=possible_current_or_swing,
            below_typical_min_flag=below_typical,
            near_absolute_min_flag=near_abs_min,
            quality_flags=";".join(flags) if flags else "OK",
        )


# ---------------------------- CSV logging ---------------------------- #


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
        "vertical_distance_unverified_m",
        "beam_range_m",
        "filter_action",
        "filter_accepted",
        "filter_delta_mm",
        "filter_jump_limit_mm",
        "filter_reject_count",
        "filter_zone",
        "filter_confidence_gate",
        "mode",
        "desired_mode",
        "mode_switch_counter",
        "mode_switch_needed_count",
        "mode_scan_start_mm",
        "mode_scan_length_mm",
        "mode_output_min_mm",
        "mode_output_max_mm",
        "mode_min_mm",
        "mode_max_mm",
        "gain_setting",
        "gain_multiplier_nominal",
        "ping_interval_ms",
        "mode_auto_set_ok",
        "range_set_ok",
        "interval_set_ok",
        "gain_set_ok",
        "mode_apply_errors",
        "mode_reapply_count",
        "mode_reapply_reason",
        "no_data_mode_action",
        "sensor_ping_number",
        "sensor_scan_start_mm",
        "sensor_scan_length_mm",
        "sensor_gain_setting",
        "sensor_transmit_duration_us",
        "sensor_speed_of_sound_mm_s",
        "sensor_processor_temp_cC",
        "sensor_pcb_temp_cC",
        "sensor_voltage_5_mV",
        "raw_instant_speed_m_s",
        "filtered_instant_speed_m_s",
        "output_instant_speed_m_s",
        "beam_range_speed_m_s",
        "output_accel_m_s2",
        "instant_speed_m_s",
        "average_speed_m_s",
        "instant_speed_m_min",
        "average_speed_m_min",
        "instant_descent_rate_m_min",
        "average_descent_rate_m_min",
        "raw_instant_speed_m_min",
        "filtered_instant_speed_m_min",
        "output_instant_speed_m_min",
        "beam_range_speed_m_min",
        "rolling_output_std_mm",
        "rolling_output_mad_mm",
        "rolling_confidence_mean",
        "rolling_confidence_min",
        "residual_to_recent_median_mm",
        "speed_sign_changes",
        "possible_current_or_swing_flag",
        "motion_observability_note",
        "quality_flags",
        "below_typical_min_flag",
        "near_absolute_min_flag",
        "absolute_min_range_mm",
        "typical_min_range_mm",
        "low_conf_threshold",
        "very_near_low_conf_threshold",
        "very_near_zone_mm",
        "close_zone_mm",
        "very_near_jump_reject_mm",
        "close_jump_reject_mm",
        "mid_jump_reject_mm",
        "max_physical_speed_mps",
        "min_jump_reject_mm",
        "recovery_count",
        "spike_reject_mm",
        "very_near_up_m",
        "very_near_down_m",
        "near_up_m",
        "near_down_m",
        "mid_up_m",
        "mid_down_m",
        "mode_switch_count",
        "very_near_switch_count",
        "no_data_mode_escape_count",
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
    Manages VERY_NEAR/NEAR/MID/FAR modes with hysteresis.
    Applies new (scan range, gain, interval) only when a transition occurs,
    except for occasional reapply attempts when sensor telemetry shows the mode
    did not stick.
    """

    class Mode:
        VERY_NEAR = "VERY_NEAR"
        NEAR = "NEAR"
        MID = "MID"
        FAR = "FAR"

    GAIN_MULTIPLIERS = {
        0: 0.6,
        1: 1.8,
        2: 5.5,
        3: 12.9,
        4: 30.2,
        5: 66.1,
        6: 144.0,
    }

    def __init__(self, ping: Ping1D):
        self.ping = ping
        self.mode = self.Mode.VERY_NEAR
        self.desired_mode = self.Mode.VERY_NEAR
        self._switch_counter = 0
        self._last_applied: Optional[ModeConfig] = None
        self.mode_auto_set_ok = ""
        self.range_set_ok = ""
        self.interval_set_ok = ""
        self.gain_set_ok = ""
        self.mode_apply_errors = ""
        self.mode_reapply_count = 0
        self.mode_reapply_reason = ""
        self._mismatch_count = 0
        self.apply_mode(self.mode, force=True)

    @property
    def switch_counter(self) -> int:
        return self._switch_counter

    def mode_switch_needed_count(self, desired: Optional[str] = None) -> int:
        target = desired or self.desired_mode
        if target == self.Mode.VERY_NEAR or self.mode == self.Mode.VERY_NEAR:
            return VERY_NEAR_SWITCH_COUNT
        return MODE_SWITCH_COUNT

    def current_config(self) -> ModeConfig:
        return self.config_for_mode(self.mode)

    def config_for_mode(self, mode: str) -> ModeConfig:
        return {
            self.Mode.VERY_NEAR: VERY_NEAR_CFG,
            self.Mode.NEAR: NEAR_CFG,
            self.Mode.MID: MID_CFG,
            self.Mode.FAR: FAR_CFG,
        }[mode]

    def gain_multiplier(self) -> float:
        return self.GAIN_MULTIPLIERS.get(self.current_config().gain, float("nan"))

    def classify(self, meters: float) -> str:
        if self.mode == self.Mode.VERY_NEAR:
            if meters > VERY_NEAR_UP_M:
                return self.Mode.NEAR
            return self.Mode.VERY_NEAR
        if self.mode == self.Mode.NEAR:
            if meters < VERY_NEAR_DN_M:
                return self.Mode.VERY_NEAR
            if meters > NEAR_UP_M:
                return self.Mode.MID
            return self.Mode.NEAR
        if self.mode == self.Mode.MID:
            if meters > MID_UP_M:
                return self.Mode.FAR
            if meters < NEAR_DN_M:
                return self.Mode.NEAR
            return self.Mode.MID
        # FAR mode
        if meters < MID_DN_M:
            return self.Mode.MID
        return self.Mode.FAR

    def step(self, meters: float) -> None:
        """Advance state machine using hysteresis + consecutive-sample voting."""
        desired = self.classify(meters)
        self.desired_mode = desired
        if desired == self.mode:
            self._switch_counter = 0
            return
        self._switch_counter += 1
        if self._switch_counter >= self.mode_switch_needed_count(desired):
            self.mode = desired
            self._switch_counter = 0
            self.apply_mode(self.mode, force=False)

    def handle_no_data(self, no_data_streak: int) -> str:
        """
        Expand the scan window if the current mode repeatedly sees no target.
        This prevents startup in VERY_NEAR from getting stuck when the real
        bottom is outside the 0.1-1.1 m close-range window.
        """
        if no_data_streak < NO_DATA_MODE_ESCAPE_COUNT:
            return ""

        next_mode = ""
        if self.mode == self.Mode.VERY_NEAR:
            next_mode = self.Mode.NEAR
        elif self.mode == self.Mode.NEAR:
            next_mode = self.Mode.MID
        elif self.mode == self.Mode.MID:
            next_mode = self.Mode.FAR

        if next_mode:
            old = self.mode
            self.mode = next_mode
            self.desired_mode = next_mode
            self._switch_counter = 0
            self.apply_mode(self.mode, force=True, reason=f"no_data_escape_from_{old}")
            return f"{old}_to_{next_mode}"
        return ""

    def verify_against_sensor_data(self, data: Dict[str, Any]) -> str:
        """
        If the device reports scan/gain values and they do not match the
        desired mode repeatedly, reapply the current mode. This combats cases
        where a command was ignored or the sensor returned to auto settings.
        """
        cfg = self.current_config()
        mismatches: List[str] = []

        sensor_scan_start = data.get("scan_start")
        sensor_scan_length = data.get("scan_length")
        sensor_gain = data.get("gain_setting")

        try:
            if sensor_scan_start not in (None, "") and int(sensor_scan_start) != cfg.scan_start_mm:
                mismatches.append("scan_start")
        except Exception:
            pass
        try:
            if sensor_scan_length not in (None, "") and int(sensor_scan_length) != cfg.scan_length_mm:
                mismatches.append("scan_length")
        except Exception:
            pass
        try:
            if sensor_gain not in (None, "") and int(sensor_gain) != cfg.gain:
                mismatches.append("gain")
        except Exception:
            pass

        if mismatches:
            self._mismatch_count += 1
        else:
            self._mismatch_count = 0
            self.mode_reapply_reason = ""
            return ""

        if self._mismatch_count >= 5:
            reason = "sensor_param_mismatch:" + "+".join(mismatches)
            self.apply_mode(self.mode, force=True, reason=reason)
            self._mismatch_count = 0
            return reason
        return "pending_mismatch:" + "+".join(mismatches)

    def apply_mode(self, mode: str, force: bool = False, reason: str = "") -> None:
        cfg = self.config_for_mode(mode)

        if not force and self._last_applied == cfg:
            return

        errors = []
        self.mode_auto_set_ok = ""
        self.range_set_ok = ""
        self.interval_set_ok = ""
        self.gain_set_ok = ""
        self.mode_reapply_reason = reason

        # Manual mode is required for manual scan range/gain settings.
        try:
            self.mode_auto_set_ok = repr(self.ping.set_mode_auto(0))
        except Exception as exc:
            self.mode_auto_set_ok = "False"
            errors.append(f"set_mode_auto:{exc}")
            logging.debug("set_mode_auto unsupported; continuing")

        try:
            self.range_set_ok = repr(
                self.ping.set_range(cfg.scan_start_mm, cfg.scan_length_mm)
            )
        except Exception as exc:
            self.range_set_ok = "False"
            errors.append(f"set_range:{exc}")
            logging.debug("set_range unsupported; continuing")
        try:
            self.interval_set_ok = repr(self.ping.set_ping_interval(cfg.interval_ms))
        except Exception as exc:
            self.interval_set_ok = "False"
            errors.append(f"set_ping_interval:{exc}")
            logging.debug("set_ping_interval unsupported; continuing")
        try:
            self.gain_set_ok = repr(self.ping.set_gain_setting(cfg.gain, verify=True))
        except Exception as exc:
            self.gain_set_ok = "False"
            errors.append(f"set_gain_setting:{exc}")
            logging.debug("set_gain_setting unsupported; continuing")

        self.mode_apply_errors = ";".join(errors)
        self._last_applied = cfg
        self.mode_reapply_count += 1 if force and reason else 0
        logging.info(
            "Mode -> %s  scan=%0.2f-%0.2f m  clamp=%0.2f-%0.2f m  gain=%s  interval=%d ms%s",
            mode,
            cfg.scan_start_mm / 1000.0,
            (cfg.scan_start_mm + cfg.scan_length_mm) / 1000.0,
            cfg.output_min_mm / 1000.0,
            cfg.output_max_mm / 1000.0,
            cfg.gain,
            cfg.interval_ms,
            f"  reason={reason}" if reason else "",
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
    _ = start_epoch  # retained for clarity; absolute epoch is logged below.
    epoch_s = time.time()
    monotonic_s = time.monotonic()
    cfg = controller.current_config()
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
        "desired_mode": controller.desired_mode,
        "mode_switch_counter": controller.switch_counter,
        "mode_switch_needed_count": controller.mode_switch_needed_count(),
        "mode_scan_start_mm": cfg.scan_start_mm,
        "mode_scan_length_mm": cfg.scan_length_mm,
        "mode_output_min_mm": cfg.output_min_mm,
        "mode_output_max_mm": cfg.output_max_mm,
        # Compatibility names with earlier v3 CSV columns.
        "mode_min_mm": cfg.output_min_mm,
        "mode_max_mm": cfg.output_max_mm,
        "gain_setting": cfg.gain,
        "gain_multiplier_nominal": controller.gain_multiplier(),
        "ping_interval_ms": cfg.interval_ms,
        "mode_auto_set_ok": controller.mode_auto_set_ok,
        "range_set_ok": controller.range_set_ok,
        "interval_set_ok": controller.interval_set_ok,
        "gain_set_ok": controller.gain_set_ok,
        "mode_apply_errors": controller.mode_apply_errors,
        "mode_reapply_count": controller.mode_reapply_count,
        "mode_reapply_reason": controller.mode_reapply_reason,
        "absolute_min_range_mm": ABSOLUTE_MIN_RANGE_MM,
        "typical_min_range_mm": TYPICAL_MIN_RANGE_MM,
        "low_conf_threshold": LOW_CONF_THRESHOLD,
        "very_near_low_conf_threshold": VERY_NEAR_LOW_CONF_THRESHOLD,
        "very_near_zone_mm": VERY_NEAR_ZONE_MM,
        "close_zone_mm": CLOSE_ZONE_MM,
        "very_near_jump_reject_mm": VERY_NEAR_JUMP_REJECT_MM,
        "close_jump_reject_mm": CLOSE_JUMP_REJECT_MM,
        "mid_jump_reject_mm": MID_JUMP_REJECT_MM,
        "max_physical_speed_mps": MAX_PHYSICAL_SPEED_MPS,
        "min_jump_reject_mm": MIN_JUMP_REJECT_MM,
        "recovery_count": RECOVERY_COUNT,
        "spike_reject_mm": "" if SPIKE_REJECT_MM is None else SPIKE_REJECT_MM,
        "very_near_up_m": VERY_NEAR_UP_M,
        "very_near_down_m": VERY_NEAR_DN_M,
        "near_up_m": NEAR_UP_M,
        "near_down_m": NEAR_DN_M,
        "mid_up_m": MID_UP_M,
        "mid_down_m": MID_DN_M,
        "mode_switch_count": MODE_SWITCH_COUNT,
        "very_near_switch_count": VERY_NEAR_SWITCH_COUNT,
        "no_data_mode_escape_count": NO_DATA_MODE_ESCAPE_COUNT,
        "target_address": TARGET_ADDRESS,
        "modem_port": MODEM_PORT,
        "modem_baud": MODEM_BAUD,
        "led_pulse_sec": LED_PULSE_SEC,
    }


def run(device: Optional[str], baud: int, udp: Optional[str], log_dir: str) -> int:
    setup_logging()

    # Package args for logging without changing the existing run signature style.
    args = argparse.Namespace(device=device, baudrate=baud, udp=udp)

    # GPIO setup: kept as in v2/v3.
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

    # Apply initial VERY_NEAR mode; no-data escape expands the scan if needed.
    controller = AdaptiveController(ping)

    # Open modem (write-only, short timeout): kept as in v2/v3.
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

    # Filters, speed, quality, and background CSV log.
    filt = RealtimeFilter(
        low_conf=LOW_CONF_THRESHOLD,
        very_near_low_conf=VERY_NEAR_LOW_CONF_THRESHOLD,
        very_near_zone_mm=VERY_NEAR_ZONE_MM,
        close_zone_mm=CLOSE_ZONE_MM,
        very_near_jump_mm=VERY_NEAR_JUMP_REJECT_MM,
        close_jump_mm=CLOSE_JUMP_REJECT_MM,
        mid_jump_mm=MID_JUMP_REJECT_MM,
        max_speed_mps=MAX_PHYSICAL_SPEED_MPS,
        min_jump_mm=MIN_JUMP_REJECT_MM,
        recovery_count=RECOVERY_COUNT,
    )
    speed = SpeedEstimator()
    quality = SignalQualityMonitor(window=QUALITY_WINDOW)
    csv_log = CsvDebugLogger(log_dir)

    start_epoch = time.time()
    start_monotonic = time.monotonic()
    previous_loop_start: Optional[float] = None
    no_data_streak = 0

    logging.info("-" * 42)
    logging.info("Streaming Ping1D -> modem (adaptive v3 VERY_NEAR). Ctrl+C to exit.")
    logging.info("Debug CSV log: %s", csv_log.path)
    logging.info("VERY_NEAR attempts %.2f-%.2f m; values below %.2f m are experimental.",
                 VERY_NEAR_CFG.output_min_mm / 1000.0,
                 VERY_NEAR_CFG.output_max_mm / 1000.0,
                 TYPICAL_MIN_RANGE_MM / 1000.0)
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
                no_data_action = controller.handle_no_data(no_data_streak)
                row.update(
                    {
                        "sensor_error": "no_data",
                        "no_data_streak": no_data_streak,
                        "no_data_mode_action": no_data_action,
                        "mode": controller.mode,
                        "desired_mode": controller.desired_mode,
                        "loop_duration_s": f"{time.monotonic() - loop_start:.6f}",
                    }
                )
                csv_log.write(row)
                logging.warning("No data from altimeter%s", f" ({no_data_action})" if no_data_action else "")
                time.sleep(NO_DATA_SLEEP_SEC)
                continue

            no_data_streak = 0
            data_keys = ";".join(sorted(str(k) for k in data.keys()))
            distance_mm = max(0, int(data["distance"]))
            confidence = int(data["confidence"])

            # Reapply mode if sensor telemetry says range/gain settings drifted.
            reapply_reason = controller.verify_against_sensor_data(data)

            now_monotonic = time.monotonic()
            filter_result = filt.update(distance_mm, confidence, now_monotonic, controller.mode)
            filtered_mm = filter_result.filtered_mm

            # Drive adaptive controller using filtered meters.
            meters = filtered_mm / 1000.0
            controller.step(meters)

            # Clamp output to the currently configured mode range.
            cfg = controller.current_config()
            clamped_mm = max(cfg.output_min_mm, min(cfg.output_max_mm, filtered_mm))
            packet = build_packet(TARGET_ADDRESS, clamped_mm, confidence)

            # Speed is computed from the exact distance being transmitted.
            epoch_for_speed = time.time()
            ispeed_m_s, aspeed_m_s = speed.update(epoch_for_speed, clamped_mm / 1000.0)
            ispeed_m_min = ispeed_m_s * 60.0
            aspeed_m_min = aspeed_m_s * 60.0
            instant_descent_m_min = max(0.0, -ispeed_m_min)
            average_descent_m_min = max(0.0, -aspeed_m_min)

            metrics = quality.update(
                monotonic_s=now_monotonic,
                raw_mm=distance_mm,
                filtered_mm=filtered_mm,
                output_mm=clamped_mm,
                confidence=confidence,
                filter_action=filter_result.action,
            )

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

            # Same LED pulse behavior as v2/v3.
            brief_led_pulse(LED_PIN, LED_PULSE_SEC)

            sensor_speed_of_sound = data.get("speed_of_sound", "")
            if sensor_speed_of_sound == "":
                # Some brping versions do not include this in distance messages.
                sensor_speed_of_sound = data.get("speed_of_sound_mm_s", "")

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
                    "vertical_distance_unverified_m": f"{clamped_mm / 1000.0:.6f}",
                    "beam_range_m": f"{clamped_mm / 1000.0:.6f}",
                    "filter_action": filter_result.action,
                    "filter_accepted": int(filter_result.accepted),
                    "filter_delta_mm": filter_result.delta_mm,
                    "filter_jump_limit_mm": filter_result.jump_limit_mm,
                    "filter_reject_count": filter_result.reject_count,
                    "filter_zone": filter_result.zone,
                    "filter_confidence_gate": filter_result.confidence_gate,
                    "mode": controller.mode,
                    "desired_mode": controller.desired_mode,
                    "mode_switch_counter": controller.switch_counter,
                    "mode_switch_needed_count": controller.mode_switch_needed_count(),
                    "mode_scan_start_mm": cfg.scan_start_mm,
                    "mode_scan_length_mm": cfg.scan_length_mm,
                    "mode_output_min_mm": cfg.output_min_mm,
                    "mode_output_max_mm": cfg.output_max_mm,
                    "mode_min_mm": cfg.output_min_mm,
                    "mode_max_mm": cfg.output_max_mm,
                    "gain_setting": cfg.gain,
                    "gain_multiplier_nominal": controller.gain_multiplier(),
                    "ping_interval_ms": cfg.interval_ms,
                    "mode_auto_set_ok": controller.mode_auto_set_ok,
                    "range_set_ok": controller.range_set_ok,
                    "interval_set_ok": controller.interval_set_ok,
                    "gain_set_ok": controller.gain_set_ok,
                    "mode_apply_errors": controller.mode_apply_errors,
                    "mode_reapply_count": controller.mode_reapply_count,
                    "mode_reapply_reason": reapply_reason or controller.mode_reapply_reason,
                    "sensor_ping_number": get_optional(data, "ping_number"),
                    "sensor_scan_start_mm": get_optional(data, "scan_start"),
                    "sensor_scan_length_mm": get_optional(data, "scan_length"),
                    "sensor_gain_setting": get_optional(data, "gain_setting"),
                    "sensor_transmit_duration_us": get_optional(data, "transmit_duration"),
                    "sensor_speed_of_sound_mm_s": sensor_speed_of_sound,
                    "sensor_processor_temp_cC": get_optional(data, "processor_temperature"),
                    "sensor_pcb_temp_cC": get_optional(data, "pcb_temperature"),
                    "sensor_voltage_5_mV": get_optional(data, "voltage_5"),
                    "raw_instant_speed_m_s": f"{metrics.raw_instant_speed_m_s:.6f}",
                    "filtered_instant_speed_m_s": f"{metrics.filtered_instant_speed_m_s:.6f}",
                    "output_instant_speed_m_s": f"{metrics.output_instant_speed_m_s:.6f}",
                    "beam_range_speed_m_s": f"{metrics.output_instant_speed_m_s:.6f}",
                    "output_accel_m_s2": f"{metrics.output_accel_m_s2:.6f}",
                    "instant_speed_m_s": f"{ispeed_m_s:.6f}",
                    "average_speed_m_s": f"{aspeed_m_s:.6f}",
                    "instant_speed_m_min": f"{ispeed_m_min:.6f}",
                    "average_speed_m_min": f"{aspeed_m_min:.6f}",
                    "instant_descent_rate_m_min": f"{instant_descent_m_min:.6f}",
                    "average_descent_rate_m_min": f"{average_descent_m_min:.6f}",
                    "raw_instant_speed_m_min": f"{metrics.raw_instant_speed_m_s * 60.0:.6f}",
                    "filtered_instant_speed_m_min": f"{metrics.filtered_instant_speed_m_s * 60.0:.6f}",
                    "output_instant_speed_m_min": f"{metrics.output_instant_speed_m_s * 60.0:.6f}",
                    "beam_range_speed_m_min": f"{metrics.output_instant_speed_m_s * 60.0:.6f}",
                    "rolling_output_std_mm": f"{metrics.rolling_output_std_mm:.3f}",
                    "rolling_output_mad_mm": f"{metrics.rolling_output_mad_mm:.3f}",
                    "rolling_confidence_mean": f"{metrics.rolling_confidence_mean:.3f}",
                    "rolling_confidence_min": f"{metrics.rolling_confidence_min:.3f}",
                    "residual_to_recent_median_mm": f"{metrics.residual_to_recent_median_mm:.3f}",
                    "speed_sign_changes": metrics.speed_sign_changes,
                    "possible_current_or_swing_flag": metrics.possible_current_or_swing_flag,
                    "motion_observability_note": metrics.motion_note,
                    "quality_flags": metrics.quality_flags,
                    "below_typical_min_flag": metrics.below_typical_min_flag,
                    "near_absolute_min_flag": metrics.near_absolute_min_flag,
                    "packet": packet,
                    "modem_write_ok": int(modem_write_ok),
                    "modem_error": modem_error,
                    "sensor_error": "",
                    "data_keys": data_keys,
                    "no_data_streak": no_data_streak,
                    "no_data_mode_action": "",
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
        description="Ping1D -> Delphis serial bridge (adaptive, low-lag v3 VERY_NEAR)."
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
