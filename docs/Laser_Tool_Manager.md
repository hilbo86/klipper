# Laser tool manager

## ID filtering

The custom `laser_tool_manager` reads the timestamped MCU ADC reports of
`analog_input ID`. Motor operation can introduce noise near an ID window
boundary. A moving mean, rejection of isolated outliers and retention hysteresis
stabilize the classification. They do not alter the ADC input or temperature
channels.

The following options are optional in the existing `[laser_tool_manager]`
section. Their defaults also apply to configurations that omit them:

| Option | Default | Meaning |
| --- | --- | --- |
| `id_filter_samples` | `20` | Moving mean window size, from 1 to 50, even or odd. A size of 1 disables averaging, but keeps outlier confirmation. |
| `id_hysteresis` | `0.5` | Retention margin on each side of an established ID window, in percentage points of the scaled ID value. Allowed range: 0 to 1. |
| `stable_samples` | `3` | Consecutive accepted classifications with a full mean window required for recognition. |
| `max_sample_age` | `0.6` | Maximum age of the latest ADC report, in seconds. |

## Acquisition and retention

Each distinct, fresh ADC report contributes at most once to the mean window.
Repeated status requests cannot fill the window, advance recognition or confirm
an outlier. Only values belonging to the same ID class enter a window. A class
change clears the previous window; unknown values cannot acquire a tool just
because their arithmetic mean falls inside a tool's interval.

Recognition requires a full window followed by the configured number of stable
classifications. With the defaults, 22 accepted reports are required: the 20th
report produces the first full mean, and the 22nd confirms the ID. At a
0.1-second report interval, the mean normally covers about two seconds and first
recognition spans about 2.1 seconds from the first report, plus report and host
polling phases. Changing an established ID can additionally ignore its first
deviating report. Old explicit window sizes remain valid; omit the option or set
it to 20 to use the larger default.

A new identity must be acquired inside its open acquisition interval. BLUE_LASER
uses 77 to 83, centered on the measured idle value of approximately 80 percent.
This preserves the previous +/-3 percentage point tolerance while correcting the
old nominal center of 82.12. The observed motor-on range of just below 79 to just
above 81 lies within the acquisition interval, including at startup with motors
already enabled.

IR_LASER uses 54.4 to 60.4, centered on the measured idle value of 57.4 percent.
Its previous interval of 56.09 to 62.09 was centered at 59.09, above the measured
idle value. Recentring preserves the six-percentage-point width and includes
the reported motor-on range of just below 55 to just above 60 percent. These
reports can now enter the mean as ordinary IR samples, including at startup;
they are not repeatedly treated as outliers. Other ID intervals are unchanged.

Only an already established classification uses hysteresis. Once recognized,
BLUE_LASER is retained while reports remain strictly between 76.5 and 83.5 with
the default hysteresis; IR_LASER is retained between 53.9 and 60.9. The default
IR retention range remains separate from the neighboring BLUE_LASER+DREMEL
conflict interval of 47 to 53. The moving mean smooths noise within the retained
class.

A persistent value outside the acquisition interval cannot acquire a
tool merely through smoothing. Measure the motor-on and motor-off ranges before
changing the calibrated intervals or increasing hysteresis.

## Revocation and freshness

Once an ID is established, a single finite raw report outside its retention
interval is omitted from the mean without dropping the recognized tool. This
also applies to a single finite ADC rail, NONE, a conflicting combination or
another tool ID. Before recognition there is no trusted ID to retain.

Two deviations within the last three distinct reports discard recognition and
the old mean. They need not name the same other tool; alternating bad/good/bad
reports cannot keep the previous tool indefinitely. A persistent tool change or
disconnection is therefore detected on its second report, independently of the
20-report mean window. During a job, calibration grant or boundary measurement,
the periodic monitor then follows the existing shutdown path. Positive output
and boundary movement commands also check new reports before proceeding.

This deliberately adds one report of confirmation for a persistent change, or
two report intervals when the deviations are separated by a matching report.
With 0.1-second reports these are about 0.1 or 0.2 seconds after the first
deviation, plus report and host polling phases. Physical shutdown latency still
needs verification on the device. The long mean is never used to average a real
tool change into a different resistor ID.

Stale reports, backwards timestamps and gaps longer than `max_sample_age`
discard recognition and filter history immediately, as do nonfinite values.
Recovery requires a complete new
acquisition. Unplugging and reconnecting a tool cannot resume a shutdown job.
IR motion protection retains its existing behavior during uncertain detection.

## Status and device check

`TOOL_STATUS` exposes:

- `id_value`: latest scaled raw ADC report, as before; `None` for nonfinite data.
- `id_filtered_value`: moving arithmetic mean, or `None` after an invalid/stale report.
- `id_filter_samples`: number of accepted reports currently buffered.
- `id_filter_size`: configured mean window size.
- `id_ignored_samples`: cumulative number of reports ignored while retaining an
  established ID since Klippy startup; an initial report of a real change also counts.
- `id_last_sample_ignored`: whether the latest report was omitted as a possible
  isolated outlier. Its raw value remains visible in `id_value`.
- `stable_samples`: consecutive classifications after the window is full.
- `id_sample_age`: age of the latest report.
- `ready`: false while startup has not completed or after shutdown. Early
  queries return unknown identity, no available controls and no power grant;
  they never read ADCs, temperatures or toolhead status before `klippy:ready`.
  Status also remains safe if initialization fails or shutdown occurs early.

Keep machining power physically off for the first check. Compare status with
motors disabled and enabled, then repeat the idle tool changes. After recognition
settles, the buffer should contain 20 reports and the displayed tool should
remain stable during the measured motor noise. Confirm NONE and each individual
tool again. These host tests do not replace the later physical shutdown-latency
measurement. A boundary shutdown now includes the detected state, raw value and
current mean so repeated failures provide useful diagnostic values.

The November 9 IR approach log also showed a roughly 4 ms Y_MAX pulse, followed
by a release. The laser configuration now uses the existing button input's
`debounce_delay: 0.020` to suppress these short transitions before the approach
monitor sees them. A persistent trigger still reaches the shutdown monitor;
confirmation adds 20 ms plus host scheduling latency. The ADC ID filter is
independent of this digital input filtering.

Routine ID reports do not switch the Y measurement laser. Confirmed invalid ID
aborts the boundary session via MCU shutdown, which applies Y_Laser's configured
zero shutdown value. The script also explicitly disables Y_Laser during cleanup,
including after an error caught before the periodic shutdown monitor runs. An
extinguished measurement laser can therefore be a consequence of an ID failure;
it is not proof of the initiating fault. Noise regression tests cover the full
IR Y scan and approach with Y_Laser enabled throughout normal measurement.

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

Before the first measurement step, optional
`TOOL_BOUNDS_APPROACH POSITION=<absolute-machine-coordinate>` permits one X/Y
approach within the original native limits and the chosen search interval. It
uses `positioning_speed` (default 20 mm/s), capped by the current velocity limit,
and ordinary Cartesian acceleration checks. It never extends native limits or
changes another axis. Z and repeating an approach after any measurement step
are rejected. The button must be released; the host monitor shuts down on a
trigger during approach. This remains a host-monitored positioning move, not an
MCU endstop homing move. Status `boundary_approach_supported` advertises support.

After that, only
`TOOL_BOUNDS_MOVE POSITION=<absolute-machine-coordinate> SPEED=<mm/s>`
may move during the session. Each command permits at most 1 mm on the selected axis,
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
The optional `--start-offset` starts the slow X/Y search at native maximum minus
the supplied positive offset, approached with `TOOL_BOUNDS_APPROACH`; Z does not
support it. After each coarse or fine trigger the script retreats 1 mm, checks
the button, and if necessary retreats a second 1 mm before failing. Retraction
never crosses the homing end. Reports keep the fine forward trigger interval
separate from the released retraction and final machine positions.

## Measured bounds before tool offsets

`<tool>_bounds_calibrated` defaults to `<tool>_calibrated`. Setting only
`bounds_calibrated: True` requires the four finite, ordered XY bounds; offsets
may remain unset and `calibrated` stays false. These bounds then apply to jogging
and offset-calibration movement. Full calibrated geometry still requires both
known offsets and measured bounds before a production job can be granted.
The status field `profile_bounds_calibrated` lets clients display known bounds
without presenting unknown tool offsets as calibrated.
