#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ping1D → Delphis bridge (FINE-TUNED: Granular gain, bidirectional, low-lag)

Enhanced with:
- Fine-grained gain control (0-6 with dynamic adjustment)
- Bidirectional movement tracking (up/down aware)
- Reduced lag with adaptive filtering
- Advanced reflection detection using pattern analysis
- Power/ping-length adaptive thresholds

Unchanged I/O:
- Packet: "$U{addr}{len:02d}{distance_mm},{confidence}"
- Console: prints "Sent: $U..." per sample, now with appended (~X.XXX m)
- Serial writes: immediate flush; tiny LED tick per send

Usage examples:
  python3 bridge.py --device /dev/serial0 --profile tank --log-file alt.csv
  python3 bridge.py --udp 192.168.2.2:9090 --profile deep
"""

from __future__ import annotations

import argparse
import csv
import logging
import sys
import time
import traceback
from collections import deque
from datetime import datetime, timezone
from statistics import median, stdev
from typing import Deque, Optional, Tuple

import serial
import RPi.GPIO as GPIO
from brping import Ping1D

# ---------------------------- Constants ---------------------------- #

LED_PIN = 12
MODEM_PORT = "/dev/ttyUSB0"
MODEM_BAUD = 9600
TARGET_ADDRESS = 2  # A002 -> "002"

# Filtering (REDUCED lag, adaptive)
LOW_CONF_THRESHOLD = 55                # %; main stability gate (lowered)
VERY_LOW_CONF = 25                     # %; poor lock
MEDIAN_WINDOW = 3                      # REDUCED for lower latency
SPIKE_REJECT_MM: Optional[int] = 1500  # Increased tolerance
ZERO_FLOOR_MM = 0

# DERIVATIVE-BASED DETECTION (RELAXED for legitimate movement)
MAX_DERIVATIVE_DOWN_MM = 1200          # Lowering: more tolerant
MAX_DERIVATIVE_UP_MM = 1500            # Pulling up: even more tolerant
MAX_ACCEL_MM = 800                     # Increased acceleration tolerance
DERIVATIVE_WINDOW = 4                  # Reduced window
UNREALISTIC_THRESHOLD_MM = 15000       # Lowered from 20m
BOTTOM_REFLECTION_THRESHOLD_MM = 2500  # Slightly reduced

# REFLECTION PATTERN DETECTION (NEW)
REFLECTION_PATTERN_WINDOW = 6          # Look for sudden doubling patterns
REFLECTION_MULTIPLIER_MIN = 1.8        # Jump >1.8x likely reflection
REFLECTION_MULTIPLIER_MAX = 3.0        # Jump <3x (higher = less sensitive)

# Band thresholds in meters (with hysteresis)
UN_UP_M, UN_DN_M = 1.2, 1.0           # ULTRA_NEAR <-> NEAR
NEAR_UP_M, NEAR_DN_M = 4.0, 3.0       # NEAR <-> MID
MID_UP_M, MID_DN_M = 25.0, 20.0       # MID  <-> FAR
FAR_UP_M, FAR_DN_M = 60.0, 50.0       # FAR  <-> ULTRA_FAR

# Band switching rules
MODE_SWITCH_COUNT = 4                  # REDUCED for faster switching
MODE_SWITCH_CONF = 45                  # Lowered threshold
GAIN_INCREASE_DERIV_LIMIT = 400        # More tolerant

# Band configs: (min_mm, max_mm, gain, ping_interval_ms, slew_limit_mm)
ULTRA_NEAR_CFG = (300,   1200,   0,   30,  200)   # Increased slew
NEAR_CFG      = (600,   5000,   1,   40,  450)    # Increased slew
MID_CFG       = (300,  25000,   3,   80, 1200)    # Increased slew
FAR_CFG       = (2000, 60000,   5,  150, 2500)    # Increased slew
ULTRA_FAR_CFG = (5000,100000,   6,  220, 3500)    # Increased slew

# SEARCH (reacquisition)
SEARCH_CFG = (300, 100000, 6, 240, 3000)
SEARCH_TRIGGER_MS = 1500               # Faster trigger
SEARCH_MIN_HOLD_MS = 1200              # Reduced hold
SEARCH_EXIT_STABLE_COUNT = 4           # Faster exit

# LOCK (close-range stabilization)
LOCK_ENTER_CONF = 75                   # Lowered
LOCK_STABLE_COUNT = 4                  # Reduced
LOCK_MAX_DERIV_MM = 80                 # Increased tolerance
LOCK_WINDOW_LOW_MM = 200
LOCK_WINDOW_HIGH_MM = 600
LOCK_PING_MS = 28
LOCK_MIN_HOLD_MS = 1000                # Reduced
LOCK_EXIT_CONF = 50                    # Lowered
LOCK_EXIT_DERIV_MM = 200               # Increased
LOCK_EDGE_EXIT_FRAMES = 3

# Watchdogs
NO_DATA_RESET_COUNT = 80
SERIAL_REOPEN_BACKOFF_S = 2.0

# LED tick
LED_PULSE_SEC = 0.005

# FINE-GRAINED GAIN CONTROL (NEW)
GAIN_MIN = 0
GAIN_MAX = 6
GAIN_STEP_UP = 1                       # Increase by 1
GAIN_STEP_DOWN = 1                     # Decrease by 1
GAIN_ADJUST_INTERVAL_MS = 500          # Min time between adjustments


# ---------------------------- Helpers ---------------------------- #

def now_epoch_ms() -> int:
    return int(time.time() * 1000)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def setup_logging() -> None:
    """Concise, time-stamped console logging (line-buffered)."""
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
    """$U + 3-digit address + 2-digit payload length + payload."""
    payload = f"{distance_mm},{confidence}"
    length = len(payload)
    return f"$U{address:03d}{length:02d}{payload}"


def brief_led_pulse(pin: int, seconds: float) -> None:
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(seconds)
    GPIO.output(pin, GPIO.LOW)


# ---------------------------- Derivative Tracker ---------------------------- #

class DerivativeTracker:
    """
    Tracks first and second derivatives with bidirectional awareness.
    """

    def __init__(self, window: int):
        self._history: Deque[int] = deque(maxlen=max(2, window))
        self._deriv1: Optional[int] = None  # First derivative (velocity)
        self._deriv2: Optional[int] = None  # Second derivative (acceleration)
        self._direction: str = "stable"     # "up", "down", "stable"

    def update(self, distance_mm: int) -> Tuple[Optional[int], Optional[int], str]:
        """
        Update with new distance, return (derivative1, derivative2, direction).
        Returns (None, None, "stable") until enough history.
        """
        self._history.append(distance_mm)
        
        if len(self._history) < 2:
            return None, None, "stable"

        # First derivative (velocity): change from previous sample
        self._deriv1 = self._history[-1] - self._history[-2]

        # Second derivative (acceleration): change in velocity
        if len(self._history) >= 3:
            prev_deriv = self._history[-2] - self._history[-3]
            self._deriv2 = self._deriv1 - prev_deriv
        else:
            self._deriv2 = None

        # Determine direction (with hysteresis)
        if self._deriv1 > 50:  # Moving away (down)
            self._direction = "down"
        elif self._deriv1 < -50:  # Moving closer (up)
            self._direction = "up"
        elif abs(self._deriv1) < 20:  # Stable
            self._direction = "stable"
        # else: maintain previous direction

        return self._deriv1, self._deriv2, self._direction

    def get_derivatives(self) -> Tuple[Optional[int], Optional[int], str]:
        """Return latest (derivative1, derivative2, direction)."""
        return self._deriv1, self._deriv2, self._direction

    def is_spike(self, direction: str) -> bool:
        """Check if current derivatives indicate a spike (direction-aware)."""
        if self._deriv1 is None:
            return False
        
        # Use direction-specific thresholds
        if direction == "down":
            max_deriv = MAX_DERIVATIVE_DOWN_MM
        elif direction == "up":
            max_deriv = MAX_DERIVATIVE_UP_MM
        else:
            max_deriv = (MAX_DERIVATIVE_DOWN_MM + MAX_DERIVATIVE_UP_MM) // 2
        
        # Check velocity
        if abs(self._deriv1) > max_deriv:
            return True
        
        # Check acceleration
        if self._deriv2 is not None and abs(self._deriv2) > MAX_ACCEL_MM:
            return True
        
        return False


# ---------------------------- Reflection Detector ---------------------------- #

class ReflectionDetector:
    """
    Detects bottom reflections using pattern analysis.
    """

    def __init__(self, window: int):
        self._history: Deque[int] = deque(maxlen=window)
        self._conf_history: Deque[int] = deque(maxlen=window)

    def update(self, distance_mm: int, confidence: int) -> bool:
        """
        Update and return True if current reading is likely a reflection.
        """
        self._history.append(distance_mm)
        self._conf_history.append(confidence)

        if len(self._history) < 3:
            return False

        # Check for sudden jump pattern
        recent = list(self._history)[-3:]
        conf_recent = list(self._conf_history)[-3:]

        # Pattern: stable readings, then sudden jump to ~2x or more
        if len(recent) >= 3:
            baseline = (recent[0] + recent[1]) / 2.0
            current = recent[2]
            
            if current > baseline * REFLECTION_MULTIPLIER_MIN and current < baseline * REFLECTION_MULTIPLIER_MAX:
                # Confidence dropped during jump?
                if conf_recent[2] < conf_recent[0] - 10:
                    return True
                
                # Very large jump with moderate confidence = likely reflection
                if current > baseline * 2.2 and conf_recent[2] < 80:
                    return True

        return False

    def get_stable_baseline(self) -> Optional[int]:
        """Return stable baseline if available."""
        if len(self._history) < 3:
            return None
        
        try:
            recent = list(self._history)[:-1]  # Exclude latest
            if len(recent) >= 2:
                std = stdev(recent)
                if std < 200:  # Stable
                    return int(sum(recent) / len(recent))
        except:
            pass
        
        return None


# ---------------------------- Enhanced Filter ---------------------------- #

class RealtimeFilter:
    """
    Low-latency filtering with adaptive strength based on movement and confidence.
    """

    def __init__(self, window: int):
        self._buf: Deque[int] = deque(maxlen=max(1, window))
        self._last: Optional[int] = None
        self._deriv_tracker = DerivativeTracker(DERIVATIVE_WINDOW)
        self._reflection_detector = ReflectionDetector(REFLECTION_PATTERN_WINDOW)
        self._stable_reading: Optional[int] = None

    def update(
        self,
        distance_mm: int,
        confidence: int,
        spike_mm: Optional[int],
        low_conf: int,
        slew_limit_mm: int,
        mode_range: Tuple[int, int],
    ) -> Tuple[int, bool, bool, bool, str]:
        """
        Returns:
            filtered_mm: int
            spike_held:  bool
            slew_limited: bool
            derivative_rejected: bool
            direction: str
        """
        spike_held = False
        slew_limited = False
        derivative_rejected = False

        # Update trackers
        deriv1, deriv2, direction = self._deriv_tracker.update(distance_mm)
        is_reflection = self._reflection_detector.update(distance_mm, confidence)

        if self._last is None:
            self._last = distance_mm
            self._buf.append(distance_mm)
            self._stable_reading = distance_mm
            return distance_mm, spike_held, slew_limited, derivative_rejected, direction

        val = distance_mm

        # (1) REFLECTION DETECTION (HIGHEST PRIORITY)
        if is_reflection:
            baseline = self._reflection_detector.get_stable_baseline()
            if baseline is not None:
                val = baseline
                derivative_rejected = True
                logging.info(f"Reflection detected: {distance_mm}mm → using baseline {baseline}mm")
            else:
                val = self._last
                derivative_rejected = True
                logging.info(f"Reflection detected: {distance_mm}mm → holding {self._last}mm")

        # (2) UNREALISTIC READING REJECTION
        min_range, max_range = mode_range
        if not derivative_rejected and val > max_range * 1.5 and confidence < 65:
            val = self._last
            derivative_rejected = True
            logging.debug(f"Unrealistic: {distance_mm}mm > {max_range * 1.5}mm")

        # (3) DERIVATIVE-BASED SPIKE DETECTION (direction-aware, relaxed)
        if not derivative_rejected:
            if self._deriv_tracker.is_spike(direction):
                # Only reject if confidence is also poor
                if confidence < LOW_CONF_THRESHOLD:
                    val = self._last
                    derivative_rejected = True
                    logging.debug(f"Deriv spike ({direction}): d1={deriv1}, d2={deriv2}")

        # (4) CONFIDENCE-GATED SPIKE HOLD (original, but relaxed)
        if (
            not derivative_rejected
            and spike_mm is not None
            and confidence < low_conf
            and abs(val - self._last) > spike_mm
        ):
            val = self._last
            spike_held = True

        # (5) ADAPTIVE SLEW LIMITER (more tolerant during fast movement with good confidence)
        delta = val - self._last
        effective_slew = slew_limit_mm
        
        # Increase slew limit if moving with good confidence
        if confidence > 70 and abs(delta) > slew_limit_mm * 0.8:
            effective_slew = int(slew_limit_mm * 1.5)
        
        if abs(delta) > effective_slew:
            val = self._last + (effective_slew if delta > 0 else -effective_slew)
            slew_limited = True

        # (6) ADAPTIVE MEDIAN FILTER (reduced influence during fast movement)
        self._buf.append(val)
        
        if confidence > 65 and len(self._buf) >= 2:
            # High confidence: trust recent readings more
            med = int(median(list(self._buf)[-2:]))
        else:
            # Low confidence: use full median
            med = int(median(self._buf))
        
        self._last = med

        # Update stable reading
        if confidence >= LOW_CONF_THRESHOLD and not derivative_rejected:
            self._stable_reading = med

        return med, spike_held, slew_limited, derivative_rejected, direction

    def get_stable_reading(self) -> Optional[int]:
        """Return last known stable reading."""
        return self._stable_reading


# ----------------------- Adaptive Mode Controller ----------------------- #

class AdaptiveController:
    """
    Manages bands + SEARCH and shallow LOCK overlay.
    NOW WITH FINE-GRAINED GAIN CONTROL (0-6 continuous).
    """

    class Mode:
        ULTRA_NEAR = "ULTRA_NEAR"
        NEAR = "NEAR"
        MID = "MID"
        FAR = "FAR"
        ULTRA_FAR = "ULTRA_FAR"
        SEARCH = "SEARCH"

    def __init__(self, ping: Ping1D, profile: str = "auto"):
        self.ping = ping
        self.profile = profile
        self.mode = self.Mode.NEAR
        self._switch_counter = 0
        self._last_cfg: Optional[Tuple[int, int, int, int]] = None
        self._current_gain = 1  # Fine-grained 0-6

        # SEARCH state
        self._unstable_since_ms: Optional[int] = None
        self._search_started_ms: Optional[int] = None
        self._search_stable = 0

        # LOCK overlay state
        self._lock_active = False
        self._lock_started_ms: Optional[int] = None
        self._lock_edge_hits = 0
        self._lock_range: Tuple[int, int] = (0, 0)
        self._prev_filtered_mm: Optional[int] = None
        self._lock_steady_count = 0

        # AUTO handling
        self.auto_enabled = False

        # Fine-grained gain control
        self._deriv_history: Deque[int] = deque(maxlen=5)
        self._gain_last_adjust_ms = 0
        self._consecutive_low_conf = 0
        self._consecutive_high_conf = 0

        self._apply_profile_limits()
        if self.try_auto_mode():
            self.auto_enabled = True
            logging.info("AUTO active; deferring manual band application")
        else:
            self.apply_mode(self.mode, force=True)

    def _apply_profile_limits(self) -> None:
        """Adjust initial mode by profile."""
        if self.profile == "tank":
            self.mode = self.Mode.ULTRA_NEAR
        elif self.profile == "deep":
            self.mode = self.Mode.MID
        else:
            self.mode = self.Mode.NEAR

    def classify_band(self, meters: float) -> str:
        """Classify band with profile ceiling."""
        def ceiling_ok(target: str) -> bool:
            if self.profile == "tank":
                return target in (self.Mode.ULTRA_NEAR, self.Mode.NEAR, self.Mode.MID)
            if self.profile in ("marina", "auto"):
                return target in (
                    self.Mode.ULTRA_NEAR, self.Mode.NEAR, self.Mode.MID, self.Mode.FAR
                )
            return True  # deep

        current = self.mode if self.mode != self.Mode.SEARCH else self.Mode.NEAR

        if current == self.Mode.ULTRA_NEAR:
            target = self.Mode.NEAR if meters > UN_UP_M else self.Mode.ULTRA_NEAR
            return target if ceiling_ok(target) else current

        if current == self.Mode.NEAR:
            if meters > NEAR_UP_M:
                target = self.Mode.FAR if meters > (MID_UP_M + 2) else self.Mode.MID
                return target if ceiling_ok(target) else current
            if meters < UN_DN_M:
                return self.Mode.ULTRA_NEAR
            return self.Mode.NEAR

        if current == self.Mode.MID:
            if meters > MID_UP_M:
                target = self.Mode.FAR
                return target if ceiling_ok(target) else current
            if meters < NEAR_DN_M:
                return self.Mode.NEAR
            return self.Mode.MID

        if current == self.Mode.FAR:
            if meters > FAR_UP_M:
                target = self.Mode.ULTRA_FAR
                return target if ceiling_ok(target) else current
            if meters < MID_DN_M:
                return self.Mode.MID
            return self.Mode.FAR

        if current == self.Mode.ULTRA_FAR:
            return self.Mode.FAR if meters < FAR_DN_M else self.Mode.ULTRA_FAR

        return self.Mode.NEAR

    # ---------- FINE-GRAINED GAIN CONTROL ---------- #

    def adjust_gain_fine(
        self,
        filtered_mm: int,
        confidence: int,
        deriv1: Optional[int],
        derivative_rejected: bool,
        direction: str,
    ) -> None:
        """
        Fine-grained gain adjustment (0-6 continuous) based on:
        - Confidence trends
        - Distance vs band range
        - Derivative stability
        - Reflection detection
        """
        if self.auto_enabled:
            return

        now_ms = now_epoch_ms()
        if now_ms - self._gain_last_adjust_ms < GAIN_ADJUST_INTERVAL_MS:
            return

        min_mm, max_mm = self.current_range()
        
        # Track confidence trends
        if confidence < LOW_CONF_THRESHOLD:
            self._consecutive_low_conf += 1
            self._consecutive_high_conf = 0
        elif confidence > 75:
            self._consecutive_high_conf += 1
            self._consecutive_low_conf = 0
        else:
            self._consecutive_low_conf = max(0, self._consecutive_low_conf - 1)
            self._consecutive_high_conf = max(0, self._consecutive_high_conf - 1)

        # AGGRESSIVE REDUCTION for unrealistic readings
        if filtered_mm > UNREALISTIC_THRESHOLD_MM and self.mode in (
            self.Mode.ULTRA_NEAR, self.Mode.NEAR, self.Mode.MID
        ):
            target_gain = 0 if self.mode == self.Mode.ULTRA_NEAR else 1
            if self._current_gain > target_gain:
                self._set_gain(target_gain)
                logging.warning(f"Unrealistic reading - forcing gain to {target_gain}")
                self._gain_last_adjust_ms = now_ms
                return

        # REDUCE gain on reflection detection
        if derivative_rejected and self._current_gain > 2:
            self._set_gain(max(GAIN_MIN, self._current_gain - GAIN_STEP_DOWN))
            logging.info(f"Reflection/spike - reducing gain to {self._current_gain}")
            self._gain_last_adjust_ms = now_ms
            return

        # REDUCE gain on persistent low confidence
        if self._consecutive_low_conf >= 5 and self._current_gain > GAIN_MIN:
            self._set_gain(max(GAIN_MIN, self._current_gain - GAIN_STEP_DOWN))
            logging.debug(f"Low confidence - reducing gain to {self._current_gain}")
            self._gain_last_adjust_ms = now_ms
            self._consecutive_low_conf = 0
            return

        # INCREASE gain if at range edge with high confidence
        if self._consecutive_high_conf >= 4:
            # Check if near max range (need more gain to see further)
            if filtered_mm > max_mm * 0.85 and self._current_gain < GAIN_MAX:
                # Verify derivatives are stable
                if deriv1 is not None:
                    self._deriv_history.append(abs(deriv1))
                if len(self._deriv_history) >= 3:
                    avg_deriv = sum(self._deriv_history) / len(self._deriv_history)
                    if avg_deriv < GAIN_INCREASE_DERIV_LIMIT:
                        self._set_gain(min(GAIN_MAX, self._current_gain + GAIN_STEP_UP))
                        logging.debug(f"Near range edge - increasing gain to {self._current_gain}")
                        self._gain_last_adjust_ms = now_ms
                        self._consecutive_high_conf = 0
                        return

        # AUTO-ADJUST based on band and distance
        band_cfg = self.config_for(self.mode)
        band_gain = band_cfg[2]
        
        # Gradually converge to band-appropriate gain
        if confidence > 60 and abs(self._current_gain - band_gain) > 0:
            if self._current_gain < band_gain:
                self._set_gain(min(band_gain, self._current_gain + GAIN_STEP_UP))
                logging.debug(f"Converging to band gain: {self._current_gain}")
                self._gain_last_adjust_ms = now_ms
            elif self._current_gain > band_gain and direction == "stable":
                self._set_gain(max(band_gain, self._current_gain - GAIN_STEP_DOWN))
                logging.debug(f"Converging to band gain: {self._current_gain}")
                self._gain_last_adjust_ms = now_ms

    def _set_gain(self, gain: int) -> None:
        """Set gain with bounds checking."""
        gain = max(GAIN_MIN, min(GAIN_MAX, gain))
        if gain == self._current_gain:
            return
        try:
            self.ping.set_gain_setting(gain, verify=True)
            self._current_gain = gain
        except Exception as e:
            logging.debug(f"Gain set failed: {e}")

    # ---------- LOCK overlay ---------- #

    def _maybe_enter_lock(self, filtered_mm: int, confidence: int) -> None:
        if self._lock_active:
            return
        if self.mode not in (self.Mode.ULTRA_NEAR, self.Mode.NEAR):
            return
        if self.auto_enabled:
            return
        if self._prev_filtered_mm is None:
            return
        deriv = abs(filtered_mm - self._prev_filtered_mm)
        if confidence >= LOCK_ENTER_CONF and deriv <= LOCK_MAX_DERIV_MM:
            self._lock_steady_count += 1
        else:
            self._lock_steady_count = 0

        if self._lock_steady_count >= LOCK_STABLE_COUNT:
            low = max(filtered_mm - LOCK_WINDOW_LOW_MM, ULTRA_NEAR_CFG[0])
            high = filtered_mm + LOCK_WINDOW_HIGH_MM
            band_min, band_max = self.current_range()
            low = max(low, band_min)
            high = min(high, band_max)
            self._lock_range = (low, high)
            self._lock_active = True
            self._lock_started_ms = now_epoch_ms()
            self._lock_edge_hits = 0
            try:
                self.ping.set_range(low, high)
            except Exception:
                logging.debug("lock set_range unsupported")
            try:
                self.ping.set_ping_interval(LOCK_PING_MS)
            except Exception:
                logging.debug("lock set_ping_interval unsupported")
            logging.info("LOCK engaged: range=%0.2f–%0.2f m",
                         low / 1000.0, high / 1000.0)

    def _maybe_exit_lock(self, filtered_mm: int, confidence: int) -> None:
        if not self._lock_active:
            return
        now_ms = now_epoch_ms()
        in_min_hold = (
            self._lock_started_ms is not None and
            (now_ms - self._lock_started_ms) < LOCK_MIN_HOLD_MS
        )
        deriv = abs(filtered_mm - (self._prev_filtered_mm or filtered_mm))
        on_edge = (
            filtered_mm <= self._lock_range[0] + 5 or
            filtered_mm >= self._lock_range[1] - 5
        )
        if on_edge:
            self._lock_edge_hits += 1
        else:
            self._lock_edge_hits = max(0, self._lock_edge_hits - 1)

        should_exit = (
            (confidence < LOCK_EXIT_CONF) or
            (deriv > LOCK_EXIT_DERIV_MM) or
            (self._lock_edge_hits >= LOCK_EDGE_EXIT_FRAMES)
        )
        if should_exit and not in_min_hold:
            self._lock_active = False
            self._lock_started_ms = None
            self._lock_steady_count = 0
            self.apply_mode(self.mode, force=True)
            logging.info("LOCK released")

    # ---------- Public step ---------- #

    def step(
        self,
        filtered_mm: int,
        confidence: int,
        deriv1: Optional[int] = None,
        derivative_rejected: bool = False,
        direction: str = "stable",
    ) -> None:
        meters = filtered_mm / 1000.0
        now_ms = now_epoch_ms()

        # Fine-grained gain adjustment
        self.adjust_gain_fine(filtered_mm, confidence, deriv1, derivative_rejected, direction)

        # Detect instability -> SEARCH
        if confidence >= LOW_CONF_THRESHOLD:
            self._unstable_since_ms = None
        else:
            if self._unstable_since_ms is None:
                self._unstable_since_ms = now_ms
            elif now_ms - self._unstable_since_ms > SEARCH_TRIGGER_MS:
                self.enter_search(now_ms)

        # SEARCH lifecycle
        if self.mode == self.Mode.SEARCH:
            if confidence >= LOW_CONF_THRESHOLD:
                self._search_stable += 1
            else:
                self._search_stable = 0

            if (
                self._search_started_ms is not None
                and (now_ms - self._search_started_ms) > SEARCH_MIN_HOLD_MS
                and self._search_stable >= SEARCH_EXIT_STABLE_COUNT
            ):
                target = self.classify_band(meters)
                self.mode = target
                if not self.auto_enabled:
                    self.apply_mode(self.mode, force=False)
                self._search_started_ms = None
                self._search_stable = 0
            self._prev_filtered_mm = filtered_mm
            return

        # Band switching
        desired = self.classify_band(meters)
        if desired == self.mode:
            self._switch_counter = 0
        else:
            if confidence >= MODE_SWITCH_CONF:
                self._switch_counter += 1
                if self._switch_counter >= MODE_SWITCH_COUNT:
                    self.mode = desired
                    self._switch_counter = 0
                    if not self.auto_enabled:
                        self.apply_mode(self.mode, force=False)
            else:
                self._switch_counter = 0

        # LOCK overlay
        if not self.auto_enabled:
            self._maybe_enter_lock(filtered_mm, confidence)
            self._maybe_exit_lock(filtered_mm, confidence)

        self._prev_filtered_mm = filtered_mm

    # ---------- Device config ---------- #

    def config_for(self, mode: str) -> Tuple[int, int, int, int, int]:
        table = {
            self.Mode.ULTRA_NEAR: ULTRA_NEAR_CFG,
            self.Mode.NEAR: NEAR_CFG,
            self.Mode.MID: MID_CFG,
            self.Mode.FAR: FAR_CFG,
            self.Mode.ULTRA_FAR: ULTRA_FAR_CFG,
            self.Mode.SEARCH: SEARCH_CFG,
        }
        return table[mode]

    def apply_mode(self, mode: str, force: bool = False) -> None:
        if self.auto_enabled:
            logging.debug("AUTO enabled; skipping manual apply")
            return
        min_mm, max_mm, gain, interval_ms, _ = self.config_for(mode)
        cfg = (min_mm, max_mm, gain, interval_ms)
        if not force and getattr(self, "_last_cfg", None) == cfg:
            return
        try:
            self.ping.set_range(min_mm, max_mm)
        except Exception:
            logging.debug("set_range unsupported")
        try:
            self.ping.set_ping_interval(interval_ms)
        except Exception:
            logging.debug("set_ping_interval unsupported")
        
        # Set gain to band default if not already close
        if abs(self._current_gain - gain) > 1:
            self._set_gain(gain)

        self._last_cfg = cfg
        logging.info(
            "Mode → %s  range=%.2f–%.2f m  gain=%s  interval=%d ms",
            mode, min_mm / 1000.0, max_mm / 1000.0, self._current_gain, interval_ms
        )

    def current_range(self) -> Tuple[int, int]:
        if self.mode == self.Mode.SEARCH:
            min_mm, max_mm, *_ = self.config_for(self.Mode.ULTRA_FAR)
            return min_mm, max_mm
        if self._lock_active:
            return self._lock_range
        min_mm, max_mm, *_ = self.config_for(self.mode)
        return min_mm, max_mm

    def current_slew_limit(self) -> int:
        _, _, _, _, slew = self.config_for(self.mode)
        return min(slew, LOCK_MAX_DERIV_MM if self._lock_active else slew)

    def enter_search(self, now_ms: int) -> None:
        if self.mode == self.Mode.SEARCH:
            return
        self.mode = self.Mode.SEARCH
        self._search_started_ms = now_ms
        self._search_stable = 0
        self._lock_active = False
        self._lock_started_ms = None
        if not self.auto_enabled:
            self.apply_mode(self.mode, force=True)
        if self.try_auto_mode():
            self.auto_enabled = True
        logging.info("Entering SEARCH (reacquisition)")

    def try_auto_mode(self) -> bool:
        """Attempt device AUTO mode."""
        try:
            self.ping.set_mode_auto(1, verify=True)  # type: ignore[attr-defined]
            logging.info("Device AUTO mode enabled")
            return True
        except Exception:
            logging.info("Device AUTO mode not available; using manual bands")
            return False


# ---------------------------- Main Program ---------------------------- #

def _open_modem() -> Optional[serial.Serial]:
    try:
        return serial.Serial(
            MODEM_PORT,
            MODEM_BAUD,
            timeout=0,
            write_timeout=0.2,
        )
    except Exception:
        logging.error("Failed to open modem port %s:\n%s",
                      MODEM_PORT, traceback.format_exc())
        return None


def run(
    device: Optional[str],
    baud: int,
    udp: Optional[str],
    zero_at_mm: int,
    log_file: Optional[str],
    profile: str,
) -> int:
    setup_logging()

    # GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)

    logging.info("Starting in 2 seconds...")
    time.sleep(2)

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

    controller = AdaptiveController(ping, profile=profile)
    filt = RealtimeFilter(window=MEDIAN_WINDOW)

    # Modem
    modem = _open_modem()
    if modem is None:
        return 4

    # Optional CSV logger
    csv_writer = None
    csv_file = None
    if log_file:
        try:
            csv_file = open(log_file, "a", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)
            if csv_file.tell() == 0:
                csv_writer.writerow([
                    "epoch_ms", "iso_time", "mode", "locked", "gain",
                    "min_mm", "max_mm", "interval_ms",
                    "raw_mm", "filtered_mm", "clamped_mm", "display_mm", "display_m",
                    "confidence", "deriv1", "deriv2", "direction",
                    "spike_held", "slew_limited", "deriv_rejected",
                    "searching", "profile",
                ])
        except Exception:
            logging.error("Failed to open log file %s:\n%s",
                          log_file, traceback.format_exc())
            csv_writer = None
            csv_file = None

    logging.info("-" * 52)
    logging.info("Streaming Ping1D → modem (fine-tuned, low-lag). Ctrl+C to exit.")
    logging.info("-" * 52)

    no_data_count = 0

    try:
        while True:
            data = ping.get_distance()
            if not data:
                no_data_count += 1
                logging.warning("No data from altimeter (%d)", no_data_count)
                if no_data_count >= NO_DATA_RESET_COUNT:
                    logging.warning("Reinitializing Ping1D after repeated no-data")
                    try:
                        ping.initialize()
                        if not controller.auto_enabled:
                            controller.apply_mode(controller.mode, force=True)
                    except Exception:
                        logging.error("Reinitialize failed:\n%s", traceback.format_exc())
                    no_data_count = 0
                continue
            no_data_count = 0

            raw_mm = int(data.get("distance", 0))
            conf = int(data.get("confidence", 0))
            raw_mm = max(0, raw_mm)

            # Adaptive filtering
            mode_range = controller.current_range()
            slew_lim = controller.current_slew_limit()
            filtered_mm, spike_held, slew_limited, deriv_rejected, direction = filt.update(
                distance_mm=raw_mm,
                confidence=conf,
                spike_mm=SPIKE_REJECT_MM,
                low_conf=LOW_CONF_THRESHOLD,
                slew_limit_mm=slew_lim,
                mode_range=mode_range,
            )

            # Get derivatives
            deriv1, deriv2, _ = filt._deriv_tracker.get_derivatives()

            # Controller step
            controller.step(filtered_mm, conf, deriv1, deriv_rejected, direction)

            # Clamp to range
            min_mm, max_mm = controller.current_range()
            clamped_mm = max(min_mm, min(max_mm, filtered_mm))

            # Zero offset
            display_mm = clamped_mm - zero_at_mm if zero_at_mm > 0 else clamped_mm
            if display_mm < ZERO_FLOOR_MM:
                display_mm = ZERO_FLOOR_MM
            display_m = display_mm / 1000.0

            # Packet and transmission
            packet = build_packet(TARGET_ADDRESS, int(display_mm), conf)
            print(f"Sent: {packet}  (~{display_m:.3f} m)", flush=True)
            try:
                assert modem is not None
                modem.write(packet.encode("ascii", errors="ignore"))
                modem.flush()
            except Exception:
                logging.error("Serial write error; attempting reopen:\n%s",
                              traceback.format_exc())
                try:
                    if modem:
                        try:
                            modem.close()
                        except Exception:
                            pass
                    time.sleep(SERIAL_REOPEN_BACKOFF_S)
                    modem = _open_modem()
                except Exception:
                    modem = None

            # LED tick
            brief_led_pulse(LED_PIN, LED_PULSE_SEC)

            # CSV log
            if csv_writer:
                min_cfg, max_cfg, gain_cfg, interval_ms, _ = controller.config_for(
                    controller.mode
                )
                csv_writer.writerow([
                    now_epoch_ms(), now_iso(), controller.mode,
                    int(getattr(controller, "_lock_active", False)),
                    controller._current_gain,
                    min_cfg if not controller._lock_active else controller._lock_range[0],
                    max_cfg if not controller._lock_active else controller._lock_range[1],
                    interval_ms,
                    raw_mm, filtered_mm, clamped_mm, int(display_mm), f"{display_m:.3f}",
                    conf, deriv1 or 0, deriv2 or 0, direction,
                    int(spike_held), int(slew_limited), int(deriv_rejected),
                    int(controller.mode == controller.Mode.SEARCH), profile,
                ])
                try:
                    csv_file.flush()
                except Exception:
                    pass

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
        if csv_file:
            try:
                csv_file.close()
            except Exception:
                pass
        GPIO.cleanup()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Ping1D → Delphis bridge (fine-tuned, low-lag)."
    )
    p.add_argument("--device", type=str, default="/dev/serial0",
                   help="Ping serial device")
    p.add_argument("--baudrate", type=int, default=115200,
                   help="Ping serial baudrate")
    p.add_argument("--udp", type=str,
                   help="Ping UDP server as IP:PORT")
    p.add_argument("--zero-at-mm", type=int, default=0,
                   help="Subtract offset from measured range")
    p.add_argument("--log-file", type=str, default="",
                   help="CSV log file path")
    p.add_argument("--profile", type=str, default="auto",
                   choices=["auto", "tank", "marina", "deep"],
                   help="Environment preset")
    return p.parse_args()


if __name__ == "__main__":
    while True:
        args = parse_args()
        rc = run(
            device=args.device,
            baud=args.baudrate,
            udp=args.udp,
            zero_at_mm=args.zero_at_mm,
            log_file=(args.log_file or None),
            profile=args.profile,
        )
        if rc == 0:
            sys.exit(0)
        print("Retrying in 20 seconds...", flush=True)
        time.sleep(20)
