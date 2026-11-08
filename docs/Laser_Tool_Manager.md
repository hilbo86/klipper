# Laser tool manager

## ID filtering

The custom `laser_tool_manager` reads the timestamped MCU ADC reports of
`analog_input ID`. Motor operation can introduce noise near an ID window
boundary. A median filter and retention hysteresis stabilize the classification;
they do not alter the ADC input or the temperature channels.

The following options are optional in the existing `[laser_tool_manager]`
section. Their defaults also apply to configurations that omit them:

| Option | Default | Meaning |
| --- | --- | --- |
| `id_filter_samples` | `5` | Odd median window size, from 1 to 9. A size of 1 disables smoothing. |
| `id_hysteresis` | `0.5` | Retention margin on each side of an established ID window, in percentage points of the scaled ID value. Allowed range: 0 to 1. |
| `stable_samples` | `3` | Consecutive classifications of new, full median windows required for recognition. |
| `max_sample_age` | `0.6` | Maximum age of the latest ADC report, in seconds. |

## Acquisition and retention

Each distinct, fresh ADC report contributes at most once to the median window.
Repeated status requests cannot fill the window or advance recognition. The
median is an observed sample, rather than an average between resistor IDs.

Recognition requires a full window followed by the configured number of stable
classifications. With the defaults, seven distinct reports are required: the
fifth report produces the first full median, and the seventh confirms the ID.
At a 0.1-second report interval this spans about 0.6 seconds from the first report,
plus the report and host polling phases.

A new identity must be acquired inside its open acquisition interval. BLUE_LASER
uses 77 to 83, centered on the measured idle value of approximately 80 percent.
This preserves the previous +/-3 percentage point tolerance while correcting the
old nominal center of 82.12. The observed motor-on range of just below 79 to just
above 81 lies within the acquisition interval, including at startup with motors
already enabled. The other ID intervals are unchanged.

Only an already established classification uses hysteresis. Once recognized,
BLUE_LASER is retained while reports remain strictly between 76.5 and 83.5 with
the default hysteresis. The median suppresses individual outliers within this
retention interval.

A persistent value outside the acquisition interval cannot acquire a
tool merely through smoothing. Measure the motor-on and motor-off ranges before
changing the calibrated intervals or increasing hysteresis.

## Revocation and freshness

A new raw report outside the current retention interval clears the old filter
immediately, including NONE, conflicting tool combinations and another tool's
ID. ADC rails and nonfinite values invalidate recognition without filtering.
During an active job or calibration grant, the existing shutdown path therefore
acts on the first such report processed by the periodic check. Positive output
commands also check the latest report before accepting power. The filter adds
no multi-report grace period to those failures; report and polling latency still
apply and need verification on the device.

Stale reports, backwards timestamps and gaps longer than `max_sample_age`
discard recognition and filter history. Recovery requires a complete new
acquisition. Unplugging and reconnecting a tool cannot resume a shutdown job.
IR motion protection retains its existing behavior during uncertain detection.

## Status and device check

`TOOL_STATUS` exposes:

- `id_value`: latest scaled raw ADC report, as before; `None` for nonfinite data.
- `id_filtered_value`: current median, or `None` after an invalid/stale report.
- `id_filter_samples`: number of reports currently buffered.
- `id_filter_size`: configured median window size.
- `stable_samples`: consecutive classifications after the window is full.
- `id_sample_age`: age of the latest report.

Keep machining power physically off for the first check. Compare status with
motors disabled and enabled, then repeat the idle tool changes. After recognition
settles, the buffer should contain five reports and the displayed tool should
remain stable during the measured motor noise. Confirm NONE and each individual
tool again. These host tests do not replace the later physical shutdown-latency
measurement.

## Power-off travel limit measurement

`TOOL_BOUNDS_BEGIN TOOL=<tool> AXIS=X|Y|Z LIMIT=<machine-coordinate>` opens an
independent measurement session. It requires homed XYZ, a matching stable tool
ID, no job/pause and no other calibration. It disables both PWMs and tool_enable;
it never changes G-code offsets or the applied fixed tool correction. IR may
measure X or Y with Z held at the upper reference; IR Z measurement is rejected.

Optional configuration ceilings `boundary_x_max`, `boundary_y_max` and
`boundary_z_min` bound the permissible LIMIT. Their defaults are the native
machine limits. The laser configuration explicitly enables 270 / 298 / -2 mm for
the first hardware test. These settings do not change normal travel limits.

Only `TOOL_BOUNDS_MOVE POSITION=<absolute-machine-coordinate> SPEED=<mm/s>` may
move during the session. Each command permits at most 1 mm on the selected axis,
with speed 0.1 to 10 mm/s. Other axes stay fixed. A pressed measurement button
permits retreat only. The command uses ordinary Cartesian movement, velocity and
acceleration checks, temporarily extending only that axis's native limits until
the individual move has finished. It restores them even on a move error.
Normal jogging, homing, file starts and calibration power grants are blocked.

The periodic monitor shuts down on changed/invalid ID, lost reference, unexpected
power, another axis moving, or 60 seconds without another measurement step.
`TOOL_BOUNDS_END` disables outputs and restores the idle action. Ending beyond
the original native range additionally clears the homing reference; it never
pretends the physical position moved. The external script normally retracts
into the original range before ending. No persistent machine limits change.

`boundary_measurement` exposes tool, axis and the current search interval;
`boundary_search_limits` exposes the configured ceilings. The companion
configuration repository owns `scripts/measure-laser-bounds.py` and its tests.
It searches X_MAX/Y_MAX in the positive direction and Z_MIN in the negative
direction, with completed moves, settling waits and fresh button queries.

## Measured bounds before tool offsets

`<tool>_bounds_calibrated` defaults to `<tool>_calibrated`. Setting only
`bounds_calibrated: True` requires the four finite, ordered XY bounds; offsets
may remain unset and `calibrated` stays false. These bounds then apply to jogging
and offset-calibration movement. Full calibrated geometry still requires both
known offsets and measured bounds before a production job can be granted.
The status field `profile_bounds_calibrated` lets clients display known bounds
without presenting unknown tool offsets as calibrated.
