# CALDIAG v2 — first-anchor calibration diagnostic firmware

Purpose: identify the active control phase when calibration stalls. Motor direction,
TR encoder reversal, PID gains, PWM limits, geometry, and existing tension logic are unchanged.
This is a diagnostic build, not a confirmed calibration fix.

Build: `pio run -e wifi_s3` from `firmware`.
Artifact: `.pio/build/wifi_s3/firmware.bin` (application image, not a merged USB-flash image).

## Logs

`CALDIAG v2 enabled` is printed when starting a fresh calibration from Belts Extended.
Phase changes and approximately every 500 ms produce a header plus four axis lines.

- `stage`: orientation_start, orientation_extend, orientation_return,
  measurement_start, horizontal_initial_tighten, horizontal_measure_bottom, measure, move.
- `wp`: zero-based waypoint; `run`: measurement repetition (-1 before measurements).
- `elapsed_ms`: current phase age (observation only).
- `tight_mask`: latched initial horizontal measurement flags, TL=1, TR=2, BL=4, BR=8.
  These reflect software flags, not independently measured physical belt tension.
  Only interpret this mask for the first six horizontal measurement points.
- `threshold_adc`: current measurement threshold, not amperes.
- `pos`, `target`: belt length in mm. Targets can be stale in open-loop tightening phases.
- `pwm`: last software command, not measured voltage or motor speed. Phase-entry snapshots
  may show the preceding phase's output. Periodic snapshots show the ongoing command.
- `current_adc`: averaged ADC reading, not calibrated amperes.
- `enc_fail_run`: cumulative failed encoder update calls since this calibration began,
  including mux-selection failures. Does not include failures before calibration.

## Logging-only scope

All v1 diagnostic phase deadlines (3/10/20/30 seconds), the 90-second waypoint deadline,
and diagnostic-triggered stop, error latch, Alarm transition, and early return are removed.
Original firmware safety mechanisms remain unchanged. Logging does not impose a timeout,
change targets, drive motors, or alter measurement sequencing. It still takes CPU/output
bandwidth, so identical real-time timing cannot be guaranteed.

## Validation

- ESP32-S3 `wifi_s3` build.
- `python3 FluidNC/test/Maslow/test_calibration_diagnostics.py` exercises extracted production
  diagnostic methods beyond all former deadlines, verifies calibration state is unchanged,
  and checks phase timing, 500 ms rate limiting, and inactive gating. The mock machine exposes
  no motor-stop, panic, or error-writing API.
- No physical motor or hardware validation has been performed.

## Tightening success event

`CALDIAG tight_success` is emitted after the existing stop/reset whenever `pull_tight`
succeeds, including Retract All and calibration. It does not change the decision or
perform an additional ADC read.

- `axis`: physical axis label selected by the existing encoder address mapping.
- `reason`: `absolute` (instantaneous ADC > threshold), `increment` (delta condition
  met for more than two consecutive evaluated calls), or `both`.
- `adc`, `threshold`: instantaneous sample used by the successful decision and its limit.
- `baseline`: updated baseline used for that same decision, captured before resetting it.
- `delta`, `delta_threshold`, `hits`: sample minus baseline, increment threshold, and
  consecutive qualifying call count. The increment condition uses a strict `>` comparison.
- `pwm_before_stop`: software command captured before the original stop call.
- `pos`: belt length at the event, before any subsequent Retract All zeroing.

The periodic snapshot remains an averaged current reading. It should not be confused
with this event's instantaneous ADC sample. Logging has CPU/output overhead, but adds
no motor commands, timeouts, or changes to the original success/reset conditions.
