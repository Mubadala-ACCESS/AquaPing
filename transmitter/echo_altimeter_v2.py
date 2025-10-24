#!/usr/bin/env python3
"""
Ping1D → Delphis bridge (adaptive, PEP 8, real-time)

- Reads distance/confidence from Blue Robotics Ping1D (brping).
- Sends "$U{addr}{len:02d}{distance_mm},{confidence}" to Delphis modem.
- Adaptive modes:
    NEAR:   0.3–5 m   | low gain | fast ping (tames <2–3 m spikes)
    MID:    3–25 m    | mid gain | medium ping
    FAR:    6–60 m    | high gain| slower ping (accurate to ≥50 m)
  Mode switching has hysteresis (requires several consecutive samples) to avoid
  flapping. A tiny median/spike filter preserves output format while removing
  single-frame outliers, especially at close range.

- Console output is line-buffered/unbuffered; serial writes are immediate.
"""

from __future__ import annotations

import argparse
import sys
import time
import logging
import traceback
from collections import deque
from statistics import median
from typing import Optional, Tuple

import serial
import RPi.GPIO as GPIO
from brping import Ping1D

# ---------------------------- Constants ---------------------------- #

LED_PIN = 12                       # BCM numbering
MODEM_PORT = "/dev/ttyUSB0"
MODEM_BAUD = 9600
TARGET_ADDRESS = 2                 # A002 -> "002" in packet

# Filtering (low-latency, robust)
LOW_CONF_THRESHOLD = 60            # %; used by spike gate
MEDIAN_WINDOW = 5                  # samples; keep small for low latency
SPIKE_REJECT_MM: Optional[int] = 1000  # None to disable spike rejection

# Adaptive mode thresholds (with hysteresis)
# Switch NEAR→MID after samples > NEAR_UP_M (4 m), back at < NEAR_DN_M (3 m)
NEAR_UP_M = 4.0
NEAR_DN_M = 3.0
# Switch MID→FAR after samples > MID_UP_M (25 m), back at < MID_DN_M (20 m)
MID_UP_M = 25.0
MID_DN_M = 20.0
# Number of consecutive samples required to switch modes
MODE_SWITCH_COUNT = 5

# Mode configurations (min_mm, max_mm, gain, ping_interval_ms)
# Gain indices are typical Ping1D values; if not supported, calls are ignored.
NEAR_CFG = (300,   5000,  1,  40)     # 0.3–5 m, low gain, ~25 Hz
MID_CFG  = (300,  25000,  3,  80)     # 0.3–25 m, mid gain, ~12.5 Hz
FAR_CFG  = (300,  60000,  5, 150)     # 0.3–60 m, high gain, ~6–7 Hz

# LED pulse (short; tiny, non-blocking feel)
LED_PULSE_SEC = 0.01

# ---------------------------- Utilities ---------------------------- #

def setup_logging() -> None:
    """Configure concise, time-stamped console logging (line-buffered)."""
    try:
        # Make prints effectively unbuffered
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
    """
    Build "$U{addr}{len:02d}{distance},{confidence}" where addr is 3 digits.
    """
    payload = f"{distance_mm},{confidence}"
    length = len(payload)
    return f"$U{address:03d}{length:02d}{payload}"


class RealtimeFilter:
    """
    Tiny stateful filter:
      - optional spike hold: if a jump > SPIKE_REJECT_MM occurs with low
        confidence, hold the last value for this frame;
      - rolling median over a small window to remove single-frame outliers.
    Latency ≈ 1 frame; preserves units/output shape.
    """

    def __init__(self, window: int, spike_mm: Optional[int], low_conf: int):
        self._window = max(1, window)
        self._buf: deque[int] = deque(maxlen=self._window)
        self._spike_mm = spike_mm
        self._low_conf = low_conf

    def update(self, distance_mm: int, confidence: int) -> int:
        if not self._buf:
            self._buf.append(distance_mm)
            return distance_mm

        last = self._buf[-1]
        if (
            self._spike_mm is not None
            and confidence < self._low_conf
            and abs(distance_mm - last) > self._spike_mm
        ):
            candidate = last  # treat as spike: ignore this frame
        else:
            candidate = distance_mm

        self._buf.append(candidate)
        return int(median(self._buf))


def brief_led_pulse(pin: int, seconds: float) -> None:
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(seconds)
    GPIO.output(pin, GPIO.LOW)


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
        self.apply_mode(self.mode, force=True)

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
        # FAR mode
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
            self.Mode.MID:  MID_CFG,
            self.Mode.FAR:  FAR_CFG,
        }[mode]
        min_mm, max_mm, gain, interval_ms = cfg

        if not force and self._last_applied == cfg:
            return

        # Apply configuration (ignore if a setter isn't supported by firmware)
        try:
            self.ping.set_range(min_mm, max_mm)
        except Exception:
            logging.debug("set_range unsupported; continuing")
        try:
            self.ping.set_ping_interval(interval_ms)
        except Exception:
            logging.debug("set_ping_interval unsupported; continuing")
        try:
            self.ping.set_gain_setting(gain, verify=True)
        except Exception:
            logging.debug("set_gain_setting unsupported; continuing")

        self._last_applied = cfg
        logging.info(
            "Mode -> %s  range=%0.2f–%0.2f m  gain=%s  interval=%d ms",
            mode, min_mm / 1000.0, max_mm / 1000.0, gain, interval_ms
        )


# ---------------------------- Main Program ---------------------------- #

def run(device: Optional[str], baud: int, udp: Optional[str]) -> int:
    setup_logging()

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
            timeout=0,          # non-blocking reads (we don't read)
            write_timeout=0.2,  # short write timeout
        )
    except Exception:
        logging.error("Failed to open modem port %s:\n%s",
                      MODEM_PORT, traceback.format_exc())
        return 4

    # Filters
    filt = RealtimeFilter(
        window=MEDIAN_WINDOW,
        spike_mm=SPIKE_REJECT_MM,
        low_conf=LOW_CONF_THRESHOLD,
    )

    logging.info("-" * 42)
    logging.info("Streaming Ping1D → modem (adaptive). Ctrl+C to exit.")
    logging.info("-" * 42)

    try:
        while True:
            data = ping.get_distance()
            if not data:
                logging.warning("No data from altimeter")
                continue

            distance_mm = int(data["distance"])
            confidence = int(data["confidence"])

            # Clamp to physical minimum (avoid negatives/overflow)
            distance_mm = max(0, distance_mm)

            # Filter before mode logic to avoid reacting to a one-off spike
            filtered_mm = filt.update(distance_mm, confidence)

            # Drive adaptive controller using meters (filtered)
            meters = filtered_mm / 1000.0
            controller.step(meters)

            # Also clamp output to the currently configured mode's max
            # This prevents an out-of-range ghost value leaking to output
            min_mm, max_mm, _, _ = {
                AdaptiveController.Mode.NEAR: NEAR_CFG,
                AdaptiveController.Mode.MID:  MID_CFG,
                AdaptiveController.Mode.FAR:  FAR_CFG,
            }[controller.mode]
            clamped_mm = max(min_mm, min(max_mm, filtered_mm))

            packet = build_packet(TARGET_ADDRESS, clamped_mm, confidence)

            # Immediate console + modem write
            print(f"Sent: {packet}", flush=True)
            try:
                modem.write(packet.encode("ascii", errors="ignore"))
                modem.flush()
            except Exception:
                logging.error("Serial write error:\n%s", traceback.format_exc())

            # Non-blocking-feel LED pulse (very short)
            brief_led_pulse(LED_PIN, LED_PULSE_SEC)

            # No extra sleep: Ping1D interval governs rate

    except KeyboardInterrupt:
        logging.info("Terminated by user")
        return 0
    except Exception:
        logging.error("Runtime error:\n%s", traceback.format_exc())
        return 5
    finally:
        try:
            if modem is not None:
                modem.close()
        except Exception:
            pass
        GPIO.cleanup()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Ping1D → Delphis serial bridge (adaptive, real-time)."
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
    return parser.parse_args()


if __name__ == "__main__":
    # Retry loop (matches your original behavior)
    while True:
        args = parse_args()
        rc = run(device=args.device, baud=args.baudrate, udp=args.udp)
        if rc == 0:
            sys.exit(0)
        print("Retrying in 20 seconds...", flush=True)
        time.sleep(20)
