# Echo Altimeter v4 CSV Log Explanation

This document explains the debug CSV produced by `echo_altimeter_v4.py`, why version 4 exists, and how to interpret the extra columns added for close-range debugging, descent-speed tracking, gain/range verification, and current/swing symptoms.

## Executive summary

Version 4 keeps the existing bridge behavior: it reads Ping1D distance/confidence, builds the same Delphis packet, writes to the same modem link, pulses the same GPIO LED, and keeps the same CLI style. The changes are focused on measurement logic and diagnostics.

The main v4 additions are:

- A `VERY_NEAR` adaptive mode for close-range work, nominally clamped to `0.10-1.10 m`.
- Explicit Ping1D manual scan windows using `scan_start_mm` and `scan_length_mm`.
- A low-lag physical gate that normally passes samples through immediately but holds implausible one-frame jumps.
- CSV logging of raw, filtered, and transmitted distance, gain, scan window, speed, descent rate, timing, quality flags, modem status, and sensor telemetry.
- Beam-range motion diagnostics for possible current-induced swing or non-straight descent. These are indicators only, because a single-beam altimeter cannot directly measure horizontal current or tilt.

## Important hardware/protocol caveats

- The Ping Sonar product page lists a typical minimum range of `0.3 m`. v4 attempts to work down to `0.1 m`, but anything below `0.3 m` is experimental and is marked using `below_typical_min_flag`.
- The Ping1D protocol defines `scan_length` as the length of the scan range and gives a minimum of `1000 mm`. Therefore v4 cannot create a true 0.10-0.30 m-only scan window; it requests `scan_start=100 mm` and `scan_length >= 1000 mm`, then locally clamps the transmitted output.
- The Ping Sonar is a single-beam echosounder. It measures distance along the acoustic beam. If the instrument is tilted by current or swing, the beam range can differ from true vertical distance.

References:

- Blue Robotics Ping1D protocol: https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/
- Blue Robotics Ping Sonar product page: https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/

## What happens during each CSV row

Each row is one measurement loop. The loop is:

```text
Ping1D get_distance()
-> raw distance + confidence
-> compare sensor-reported settings with requested mode
-> reapply mode if scan/gain settings appear to drift
-> physical plausibility filter
-> adaptive mode update
-> clamp final output to active mode range
-> build Delphis packet
-> write packet to modem
-> pulse LED
-> queue one CSV row to background logger
```

The CSV is intentionally verbose. It is meant to answer not only "what distance was sent?" but also "why was that distance sent?", "what mode was active?", "what gain was requested?", "did the sensor accept the settings?", "was the reading filtered?", and "was the loop lagging?"

## Version 4 behavior

### 1. Adaptive modes

| Mode | Scan start | Output clamp | Gain | Ping interval | Purpose |
|---|---:|---:|---:|---:|---|
| `VERY_NEAR` | `100 mm` | `100-1100 mm` | `0` | `30 ms` | Experimental close-range mode for 0.1-1.1 m. Lowest gain to reduce near-field ringing. |
| `NEAR` | `100 mm` | `100-5000 mm` | `1` | `40 ms` | Close/shallow operation while still allowing escape from VERY_NEAR. |
| `MID` | `300 mm` | `300-25000 mm` | `3` | `80 ms` | Middle-range operation. |
| `FAR` | `300 mm` | `300-60000 mm` | `5` | `150 ms` | Long-range operation with higher gain and slower pinging. |

### 2. Why VERY_NEAR was added

The earlier code still showed close-range spike behavior around approximately 0.6 m. Version 4 adds a dedicated `VERY_NEAR` mode so the sensor uses the lowest gain and a tight local output clamp near the bottom. This is meant to reduce false close echoes and transducer ring-down effects while preserving low-lag output.

Because the protocol minimum scan length is 1000 mm, `VERY_NEAR` cannot scan only 100-300 mm. Instead, it requests a scan starting at 100 mm with a 1000 mm minimum scan length, then logs flags when the output is below the typical 300 mm operating range.

### 3. No-data escape

Starting in `VERY_NEAR` can fail when the bottom is outside the 0.1-1.1 m window. To prevent startup lock, v4 counts no-data rows. After `no_data_mode_escape_count` consecutive no-data rows, it expands the mode:

```text
VERY_NEAR -> NEAR -> MID -> FAR
```

The action is written to `no_data_mode_action`.

### 4. Sensor setting verification

Some rows include sensor-reported `scan_start`, `scan_length`, and `gain_setting`. v4 compares those to the requested mode. If they mismatch repeatedly, it reapplies the current mode and logs the reason in `mode_reapply_reason`.

### 5. Low-lag filtering

v4 does not use a rolling median as the live output. A rolling median can hide spikes, but it adds lag. Instead, v4 uses a physical gate:

- Accept normal measurements immediately.
- Hold impossible one-frame jumps in VERY_NEAR and CLOSE zones even if confidence is high.
- Hold large jumps when confidence is low.
- Accept a repeated new level after `recovery_count` similar rejections, so the filter cannot remain stuck forever.

### 6. Current and non-straight descent

v4 cannot truly correct for current because the Ping1D does not measure instrument tilt or horizontal drift. What v4 can do is log symptoms: range oscillation, speed sign changes, noisy output, and large accelerations. When these symptoms are strong enough, `possible_current_or_swing_flag` becomes `1` and `quality_flags` includes `POSSIBLE_CURRENT_SWING_OR_NONVERTICAL_DESCENT`.

## Column-by-column explanation

### Program and timing fields

| Field | Meaning |
|---|---|
| `software_version` | Code version string written by the script. It identifies which program produced the row. In the uploaded v4 code this currently says `echo_altimeter_v3_very_near`, so the file is functionally v4 but the constant should ideally be renamed to `echo_altimeter_v4` to avoid confusion. |
| `iso_time_utc` | Human-readable absolute UTC timestamp for this measurement row. Use it to match the log with field notes, RBR logs, boat notes, or video. |
| `epoch_time_s` | Unix time in seconds. This is best for plotting, merging with other instruments, and computing absolute timing differences. |
| `monotonic_time_s` | Python monotonic clock in seconds. It is not affected by system clock changes, so it is safer for measuring loop timing. |
| `elapsed_s` | Seconds since this script started streaming. Useful for quick plots of the run without converting absolute timestamps. |
| `loop_dt_s` | Time between the start of this loop and the start of the previous loop. This shows the actual sampling rhythm. |
| `sensor_read_duration_s` | How long the call to `ping.get_distance()` took. If this grows, the delay is coming from the Ping1D read path. |
| `serial_write_duration_s` | How long the modem serial write and flush took. If this grows, the Delphis modem path is blocking the loop. |
| `loop_duration_s` | Total time spent in one full measurement loop, including sensor read, filtering, modem write, LED pulse, and log queueing. |

### Connection fields

| Field | Meaning |
|---|---|
| `device` | Serial device used for the Ping1D connection, normally `/dev/serial0`. |
| `baudrate` | Baud rate used for the Ping1D connection, normally `115200`. |
| `udp` | UDP address if the Ping1D is connected over UDP instead of serial. It is blank in normal serial operation. |

### Distance fields

| Field | Meaning |
|---|---|
| `raw_distance_mm` | Raw distance returned directly by the Ping1D, in millimeters, before filtering and clamping. |
| `raw_distance_m` | Raw Ping1D distance converted to meters. |
| `confidence` | Ping1D confidence value for the returned distance. Low confidence means the bottom lock or echo selection may be unreliable. |
| `filtered_distance_mm` | Distance after the v4 physical plausibility filter. The filter normally passes samples through immediately; it only holds samples that look like implausible single-frame jumps or below-limit values. |
| `filtered_distance_m` | Filtered distance converted to meters. |
| `output_distance_mm` | Final distance sent to the Delphis packet after filtering and clamping to the active mode output range. |
| `output_distance_m` | Final transmitted distance converted to meters. |
| `vertical_distance_unverified_m` | Same value as the transmitted distance, but named carefully because the Ping1D only measures along its acoustic beam. Without tilt/IMU data, this cannot be guaranteed to be true vertical height. |
| `beam_range_m` | Beam-line range measured by the sonar and transmitted after filtering/clamping. This is the physically honest measurement if the instrument is tilted or swinging. |

### Filter fields

| Field | Meaning |
|---|---|
| `filter_action` | Decision made by the real-time filter. Common values include `accept`, `accept_initial`, `hold_low_conf_jump`, `hold_large_jump`, `hold_very_near_jump`, `hold_close_jump`, `hold_below_absolute_min`, `hold_escape_from_very_near`, and `accept_after_repeated_jump`. |
| `filter_accepted` | `1` if the raw sample was accepted into the filtered stream; `0` if the filter held the previous good value. |
| `filter_delta_mm` | Difference between the current raw reading and the previous accepted reading. Large values indicate jumps, false locks, wall echoes, or rapid real movement. |
| `filter_jump_limit_mm` | Dynamic jump limit allowed for this row based on elapsed time and `max_physical_speed_mps`. It prevents impossible motion from being accepted just because the sample interval is small. |
| `filter_reject_count` | Number of consecutive similar rejected samples. If a new value repeats enough times and confidence is acceptable, v4 can accept it as a real new level instead of staying locked forever. |
| `filter_zone` | Distance zone used by the filter: `initial`, `very_near`, `close`, or `normal`. Close zones use stricter jump thresholds. |
| `filter_confidence_gate` | Confidence threshold used for this row. VERY_NEAR uses a stricter threshold than the general mode because close-range echoes can be unstable. |

### Adaptive mode fields

| Field | Meaning |
|---|---|
| `mode` | Current active adaptive mode: `VERY_NEAR`, `NEAR`, `MID`, or `FAR`. |
| `desired_mode` | Mode the controller currently wants based on filtered distance. It may differ from `mode` while hysteresis is counting consecutive samples. |
| `mode_switch_counter` | Number of consecutive samples voting for a mode change. This prevents rapid flapping between modes. |
| `mode_switch_needed_count` | Number of consecutive samples required before the controller actually changes mode. VERY_NEAR transitions use a shorter count than normal transitions. |
| `mode_scan_start_mm` | Scan start sent to Ping1D for the active mode. This is the beginning of the acoustic scan window from the transducer. |
| `mode_scan_length_mm` | Scan length sent to Ping1D. v4 enforces the Ping protocol minimum scan length of 1000 mm. |
| `mode_output_min_mm` | Local minimum clamp for the value sent to Delphis in the active mode. |
| `mode_output_max_mm` | Local maximum clamp for the value sent to Delphis in the active mode. |
| `mode_min_mm` | Compatibility alias for `mode_output_min_mm`, kept so older plotting scripts using v3 column names still work. |
| `mode_max_mm` | Compatibility alias for `mode_output_max_mm`, kept so older plotting scripts using v3 column names still work. |
| `gain_setting` | Manual Ping1D gain setting requested for the active mode. Lower gain is used close to the transducer to reduce ringing and false echoes; higher gain is used for long range. |
| `gain_multiplier_nominal` | Nominal gain multiplier corresponding to `gain_setting`. v4 uses the documented Ping1D gain scale: 0=0.6, 1=1.8, 2=5.5, 3=12.9, 4=30.2, 5=66.1, 6=144. |
| `ping_interval_ms` | Requested time between acoustic measurements. Small values give faster close-range updates; larger values give slower but more stable long-range operation. |

### Mode-setting success and reapply fields

| Field | Meaning |
|---|---|
| `mode_auto_set_ok` | Result of calling `set_mode_auto(0)`. Manual mode is needed so scan range and gain settings can be controlled by the script. |
| `range_set_ok` | Result of calling `set_range(scan_start, scan_length)` for the active mode. |
| `interval_set_ok` | Result of calling `set_ping_interval(interval_ms)` for the active mode. |
| `gain_set_ok` | Result of calling `set_gain_setting(gain, verify=True)` for the active mode. |
| `mode_apply_errors` | Any exceptions raised while applying mode settings. Blank means no exception was caught. |
| `mode_reapply_count` | How many times v4 force-reapplied mode settings because of no-data escape or sensor telemetry mismatch. |
| `mode_reapply_reason` | Reason for reapplying mode settings, such as `no_data_escape_from_VERY_NEAR`, `sensor_param_mismatch:scan_start+gain`, or `pending_mismatch:gain`. |
| `no_data_mode_action` | Mode expansion action caused by repeated no-data frames, such as `VERY_NEAR_to_NEAR`. Blank means no no-data escape happened on that row. |

### Sensor telemetry fields

| Field | Meaning |
|---|---|
| `sensor_ping_number` | Ping number returned by the sensor if available. It can help detect repeated or skipped sensor measurements. |
| `sensor_scan_start_mm` | Actual scan start reported by the sensor in the distance message. |
| `sensor_scan_length_mm` | Actual scan length reported by the sensor in the distance message. |
| `sensor_gain_setting` | Actual gain setting reported by the sensor in the distance message. |
| `sensor_transmit_duration_us` | Acoustic transmit duration reported by the sensor. Longer transmit durations can occur with longer-range operation. |
| `sensor_speed_of_sound_mm_s` | Speed of sound setting reported by the sensor if available. Some `brping` versions do not include this in distance messages. |
| `sensor_processor_temp_cC` | Processor temperature in centi-degrees C if reported by the sensor. |
| `sensor_pcb_temp_cC` | PCB temperature in centi-degrees C if reported by the sensor. |
| `sensor_voltage_5_mV` | 5 V supply voltage in millivolts if reported by the sensor. |

### Speed and motion fields

| Field | Meaning |
|---|---|
| `raw_instant_speed_m_s` | Direct sample-to-sample speed computed from raw distance, in m/s. It is very sensitive to spikes. |
| `filtered_instant_speed_m_s` | Direct sample-to-sample speed computed from filtered distance, in m/s. |
| `output_instant_speed_m_s` | Direct sample-to-sample speed computed from final transmitted distance, in m/s. |
| `beam_range_speed_m_s` | Speed along the acoustic beam. In v4 this is the same numeric value as `output_instant_speed_m_s`, but the name reminds us that the sonar measures beam range, not confirmed vertical motion. |
| `output_accel_m_s2` | Acceleration of the output distance signal. Large values usually indicate spikes, mode-clamp transitions, swing, or poor bottom lock. |
| `instant_speed_m_s` | Instantaneous speed estimated by `AltimeterSpeed` from the transmitted output distance. If the external helper is missing, v4 falls back to simple finite differences. |
| `average_speed_m_s` | Average speed over the recent `AltimeterSpeed` history window. |
| `instant_speed_m_min` | Instantaneous speed converted to m/min. |
| `average_speed_m_min` | Average speed converted to m/min. |
| `instant_descent_rate_m_min` | Positive descent rate in m/min. It is only positive when the signed speed indicates descent according to the estimator convention. |
| `average_descent_rate_m_min` | Positive average descent rate in m/min. |
| `raw_instant_speed_m_min` | Direct raw sample-to-sample speed converted to m/min. Best for spotting raw spikes. |
| `filtered_instant_speed_m_min` | Filtered sample-to-sample speed converted to m/min. |
| `output_instant_speed_m_min` | Final output sample-to-sample speed converted to m/min. |
| `beam_range_speed_m_min` | Beam-range speed converted to m/min. |

### Rolling stability fields

| Field | Meaning |
|---|---|
| `rolling_output_std_mm` | Rolling standard deviation of recent output distances. High values mean unstable range output. |
| `rolling_output_mad_mm` | Rolling median absolute deviation of recent output distances. More robust than standard deviation when spikes are present. |
| `rolling_confidence_mean` | Rolling average confidence across the recent quality window. |
| `rolling_confidence_min` | Minimum confidence in the recent quality window. |
| `residual_to_recent_median_mm` | Current output distance minus the recent median output distance. Large residuals identify sudden deviations. |
| `speed_sign_changes` | Number of recent beam-range speed sign changes, ignoring very small values. Repeated sign changes can indicate bobbing, swing, or non-straight descent. |

### Current, swing, and quality interpretation fields

| Field | Meaning |
|---|---|
| `possible_current_or_swing_flag` | `1` if the output shows symptoms consistent with current-induced swing or non-vertical descent. This is a heuristic, not a direct current measurement. |
| `motion_observability_note` | Reminder string: the single-beam altimeter cannot measure horizontal current or tilt directly without an IMU/current meter. |
| `quality_flags` | Semicolon-separated row warnings. `OK` means no quality heuristic was triggered on that row. |
| `below_typical_min_flag` | `1` if the output is below the typical Ping Sonar minimum range of 300 mm. v4 allows the attempt but marks it experimental. |
| `near_absolute_min_flag` | `1` if the output is near the experimental 100 mm lower clamp. |
| `absolute_min_range_mm` | Experimental lower limit requested by the v4 code. This is set to 100 mm. |
| `typical_min_range_mm` | Typical practical minimum range used for warning flags. This is set to 300 mm. |

### Threshold and configuration fields

| Field | Meaning |
|---|---|
| `low_conf_threshold` | General confidence threshold used by the filter and quality flags. |
| `very_near_low_conf_threshold` | Stricter confidence threshold used in VERY_NEAR conditions. |
| `very_near_zone_mm` | Distance below which the filter treats samples as very-near and applies the strictest jump protection. |
| `close_zone_mm` | Distance below which the filter applies close-range jump protection. |
| `very_near_jump_reject_mm` | Maximum allowed single-frame jump in VERY_NEAR before the filter holds the previous good value. |
| `close_jump_reject_mm` | Maximum allowed single-frame jump in the close zone before holding. |
| `mid_jump_reject_mm` | Maximum jump threshold for normal/mid/far regions when other confidence rules apply. |
| `max_physical_speed_mps` | Maximum physically plausible range-rate used to compute the dynamic jump limit. |
| `min_jump_reject_mm` | Minimum jump threshold used even when the time step is very small. |
| `recovery_count` | Number of repeated similar rejected samples required before v4 accepts a new level. This prevents permanent filter lock. |
| `spike_reject_mm` | General large-spike threshold retained for compatibility with earlier v3 logging and comparison. |
| `very_near_up_m` | If in VERY_NEAR and filtered distance rises above this value, the controller starts voting to leave VERY_NEAR. |
| `very_near_down_m` | If in NEAR and filtered distance falls below this value, the controller starts voting to enter VERY_NEAR. |
| `near_up_m` | If in NEAR and filtered distance rises above this value, the controller starts voting to enter MID. |
| `near_down_m` | If in MID and filtered distance falls below this value, the controller starts voting to return to NEAR. |
| `mid_up_m` | If in MID and filtered distance rises above this value, the controller starts voting to enter FAR. |
| `mid_down_m` | If in FAR and filtered distance falls below this value, the controller starts voting to return to MID. |
| `mode_switch_count` | Normal number of consecutive samples required for NEAR/MID/FAR transitions. |
| `very_near_switch_count` | Number of consecutive samples required for transitions involving VERY_NEAR. |
| `no_data_mode_escape_count` | Number of consecutive no-data rows required before v4 expands to a wider scan mode. |

### Modem/output fields

| Field | Meaning |
|---|---|
| `packet` | Exact Delphis packet sent on this row. The format remains `$U{address}{length}{distance_mm},{confidence}`. |
| `target_address` | Delphis target address used in the packet. The code uses address `2`, which is formatted as `002`. |
| `modem_port` | Serial device used for the Delphis modem, normally `/dev/ttyUSB0`. |
| `modem_baud` | Baud rate used for the Delphis modem, normally `9600`. |
| `modem_write_ok` | `1` if the packet was written and flushed to the modem without exception; `0` if a serial error occurred. |
| `modem_error` | Full serial/modem error text if a modem write failed. Blank means no modem error was caught. |

### Sensor/error/logging fields

| Field | Meaning |
|---|---|
| `sensor_error` | Sensor-side error marker. `no_data` means `ping.get_distance()` returned no data on that loop. |
| `data_keys` | List of keys present in the returned Ping1D data dictionary. Useful for confirming which telemetry fields the installed `brping` version returns. |
| `no_data_streak` | Number of consecutive loops with no distance data from the sensor. |
| `led_pulse_sec` | LED pulse duration after each sent packet. This preserves the existing GPIO behavior. |
| `logger_queue_size` | Number of rows waiting in the background CSV logger queue at the time this row was queued. |
| `logger_dropped_rows` | Number of rows dropped because the logger queue filled. This should remain 0; if it grows, disk logging is not keeping up. |

## How to interpret common `quality_flags`

| Flag | Meaning | What to check next |
|---|---|---|
| `LOW_CONFIDENCE` | Confidence is below `low_conf_threshold`. | Check water conditions, orientation, range mode, gain, and whether the target is in the scan window. |
| `BELOW_TYPICAL_MIN_RANGE` | Output is below the typical 300 mm minimum range. | Treat as experimental; verify in a controlled tank or pier test. |
| `NEAR_ABSOLUTE_MIN_RANGE` | Output is close to the 100 mm experimental lower clamp. | Be careful: this is below normal product spec and may be ring-down or geometry dependent. |
| `NOISY_RANGE_STD` | Rolling standard deviation is high. | Look for unstable bottom lock, current/swing, bubbles, wall reflections, vegetation, or fish. |
| `NOISY_RANGE_MAD` | Rolling median absolute deviation is high. | More robust indication of instability when isolated spikes exist. |
| `FAST_RANGE_ACCELERATION` | Output speed changed too quickly. | Often a spike, clamp transition, mode switch, swing, or poor lock. |
| `FILTER_HOLDING_SAMPLE` | The filter rejected the raw reading and reused the previous good value. | Check `filter_action`, `filter_delta_mm`, `confidence`, and `filter_reject_count`. |
| `POSSIBLE_CURRENT_SWING_OR_NONVERTICAL_DESCENT` | Beam range oscillates or changes direction repeatedly while noisy. | Treat `beam_range_m` as beam range, not corrected vertical distance. Add IMU/tilt data if accurate vertical correction is required. |

## How to read a row quickly

For quick debugging, start with these columns in order:

1. `iso_time_utc` and `elapsed_s` - when did this happen?
2. `raw_distance_m`, `filtered_distance_m`, `output_distance_m` - what changed through the processing path?
3. `confidence` and `quality_flags` - should the row be trusted?
4. `mode`, `gain_setting`, `mode_scan_start_mm`, `mode_scan_length_mm` - what was the sensor asked to do?
5. `sensor_scan_start_mm`, `sensor_scan_length_mm`, `sensor_gain_setting` - did the sensor report matching settings?
6. `filter_action`, `filter_delta_mm`, `filter_reject_count` - did the filter hold a spike?
7. `instant_descent_rate_m_min`, `beam_range_speed_m_min`, `speed_sign_changes` - what was the apparent motion?
8. `modem_write_ok`, `logger_dropped_rows`, `loop_duration_s` - was the system keeping up?

## Interpreting a run

A good near-bottom test should show:

- Mode mostly `VERY_NEAR` or `NEAR`.
- `output_distance_mm` in the expected close range, especially below 1200 mm for VERY_NEAR validation.
- Confidence consistently above threshold, especially in VERY_NEAR where the threshold is stricter.
- Few `FILTER_HOLDING_SAMPLE` flags.
- Low `rolling_output_std_mm` and `rolling_output_mad_mm`.
- No increasing `logger_dropped_rows`.
- `modem_write_ok = 1`.

A poor or inconclusive run may show:

- Low confidence across most rows.
- Frequent `hold_low_conf_jump`, `hold_large_jump`, `hold_very_near_jump`, or `hold_close_jump`.
- Mode rapidly expanding to `FAR` because the measured range is far outside the close window.
- Large early speed values caused by mode clamp transitions, not real descent.
- `POSSIBLE_CURRENT_SWING_OR_NONVERTICAL_DESCENT` or many `speed_sign_changes`.

## Notes on speed sign and descent

The log includes signed speed and positive descent-rate fields. The signed speed convention comes from the speed estimator and the distance signal. Because the altimeter measures beam range, the speed is best understood as range-rate along the beam unless the instrument is known to be vertical. The positive descent-rate fields are convenient for field use, but they should not be treated as corrected vertical descent when current, tilt, or swing is present.

## Known limitations

- v4 cannot correct for tilt because no IMU angle is logged.
- v4 cannot measure horizontal current directly.
- Readings below 0.3 m are experimental even though v4 attempts to log and transmit them.
- Very large speed values during startup or mode transitions are usually artifacts of output clamping and scan-window expansion.
- Sensor telemetry fields depend on what the installed `brping` version returns in `get_distance()`. Blank telemetry fields do not always mean the sensor lacks the parameter; they may mean that message did not include it.

## Recommended pier/tank validation procedure

1. Start with the sensor in water and a known fixed distance above a flat target.
2. Record a short stationary log at 1.0 m, 0.75 m, 0.5 m, 0.3 m, and, only experimentally, 0.1-0.2 m.
3. For each distance, check `mode`, `confidence`, `output_distance_mm`, `quality_flags`, `rolling_output_std_mm`, and `filter_action`.
4. A valid close-range setting should have stable output, few filter holds, and confidence above the relevant threshold.
5. If the value is below 0.3 m, mark it as experimental even if it looks stable.
6. If the instrument is lowered by hand or cable in current, interpret `beam_range_speed_m_min` as beam/range motion, not guaranteed vertical descent.

## Suggested small cleanup for the code

The uploaded v4 file has the v4 behavior, but the constant still says:

```python
SOFTWARE_VERSION = "echo_altimeter_v3_very_near"
```

For clarity, change it to:

```python
SOFTWARE_VERSION = "echo_altimeter_v4"
```

That will make future CSV logs self-identify as v4.

