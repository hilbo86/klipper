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
| `id_filter_time` | Unset | Moving mean duration with all drivers off, in seconds, 0.1 to 5. Overrides the sample count in this mode. |
| `id_filter_time_steppers_on` | Value of `id_filter_time` | Moving mean duration while any driver is enabled, in seconds, 0.1 to 5. |
| `id_hysteresis` | `0.5` | Retention margin on each side of an established ID window, in percentage points of the scaled ID value. Allowed range: 0 to 1. |
| `stable_samples` | `3` | Consecutive accepted classifications with a full mean window required for recognition. |
| `max_sample_age` | `0.6` | Maximum age of the latest ADC report, in seconds. |

Each ID class also has three optional settings. Replace `<prefix>` with a name
from the table below:

| Option | Default | Meaning |
| --- | --- | --- |
| `<prefix>_id_center` | See table | Center of the scaled ADC interval, in percent. |
| `<prefix>_id_tolerance` | `3.0` | Acquisition half-width while all stepper drivers are disabled, in percentage points. |
| `<prefix>_id_tolerance_steppers_on` | Value of `<prefix>_id_tolerance` | Acquisition half-width while any stepper driver is enabled, including stationary energized motors. |

| ID class | Prefix | Default center |
| --- | --- | --- |
| BLUE_LASER | `blue_laser` | `80.0` |
| IR_LASER | `ir_laser` | `57.4` |
| DREMEL | `dremel` | `17.58` |
| BLUE_LASER+IR_LASER | `blue_laser_ir_laser` | `68.79` |
| BLUE_LASER+DREMEL | `blue_laser_dremel` | `50.0` |
| IR_LASER+DREMEL | `ir_laser_dremel` | `41.82` |
| NONE | `none` | `31.21` |

The laser configuration explicitly selects a narrow idle band and a wider motor
band for all seven classes, for example:

```ini
[laser_tool_manager]
ir_laser_id_center: 57.4
ir_laser_id_tolerance: 2.0
ir_laser_id_tolerance_steppers_on: 5.0
id_hysteresis: 1.0
id_filter_time: 0.75
id_filter_time_steppers_on: 2.5
```

Older configurations that omit these options retain the previous +/-3 bands in
both modes. Centers and tolerances must be finite, tolerances positive and
intervals within 0..100. The enabled tolerance cannot be smaller than the idle
tolerance. With drivers off, no acquisition interval or its retention extension
may overlap another class's acquisition interval. Retention extensions may
overlap only within an unassigned gap. Motor-on windows may overlap, including
acquisition intervals: a confirmed identity takes priority inside its own
retention interval. Without a confirmed identity, multiple matching acquisition
intervals yield UNKNOWN, regardless of table order. Recognition is never guessed
from the mean of ambiguous reports. Outside the overlap, a uniquely matching
motor interval can still acquire an identity. Update the Klipper module before
loading configurations with these new options. After Python changes, restart
the **Klipper service**, for example `sudo systemctl restart klipper`, so the
new process imports the changed module. G-code `RESTART` and `FIRMWARE_RESTART`
reuse the host process and its Python module cache; they do not reload changed
Python source. If the MCU still reports a previous shutdown after the service
restart, issue `FIRMWARE_RESTART` then wait for ready. Config-only changes can
use the ordinary restart commands once the updated module is running.

## Acquisition and retention

Each distinct, fresh ADC report contributes at most once to the mean window.
Repeated status requests cannot fill the window, advance recognition or confirm
an outlier. Only values belonging to the same ID class enter a window. A class
change clears the previous window; unknown values cannot acquire a tool just
because their arithmetic mean falls inside a tool's interval.

The laser configuration selects time windows of **0.75 s off / 2.5 s on**. MCU
report timestamps determine expiry; neither polling frequency nor an assumed
report count determines the duration. Each accepted report in the current time
window has equal weight. Old reports expire as new accepted reports arrive.
With a 0.1 s report interval this usually retains 8 idle reports or 26 motor
reports (both endpoints included when aligned). Other report rates may retain
more than 50 samples. The window must have elapsed and contain at least two
reports before stable confirmation begins. Three stable full-window
classifications normally acquire an idle ID in about 1.0 s from the first
accepted report, or a motor ID in about 2.7 s, plus reporting/polling phases.

The filtered ID remains the arithmetic mean. RMS of the ADC signal itself
includes noise energy and would bias the resistor ID upwards. Instead,
`id_noise_rms` reports the RMS deviation from the mean, equivalent to population
standard deviation, in percentage points. It uses only the accepted reports in
the same window; isolated rejected spikes do not enter either statistic.

For configurations omitting both time options, the legacy sample window remains
in use. With its defaults, 22 accepted reports are required: the 20th
report produces the first full mean, and the 22nd confirms the ID. At a
0.1-second report interval, the mean normally covers about two seconds and first
recognition spans about 2.1 seconds from the first report, plus report and host
polling phases. Changing an established ID can additionally ignore its first
deviating report. Old explicit window sizes remain valid; omit the option or set
it to 20 to use the larger count default. Configured time windows override this
count independently in each driver mode.

A new identity must be acquired inside its open acquisition interval in the
current driver mode. With the explicit laser configuration, BLUE_LASER acquires
between 78 and 82 with motors off, or 75 and 85 with motors on. IR_LASER has
acquisition intervals 55.4..59.4 off, or 52.4..62.4 on, subject to the ambiguity
rule above. The measured idle centers are
approximately 80 and 57.4 percent; the old nominal centers were 82.12 and 59.09.
The observed motor-on ranges of just below 79 to just above 81 (blue), and just
below 55 to just above 60 (IR), fit the wider bands.

Only an established classification uses retention hysteresis. With the explicit
configuration and 1-point hysteresis, blue retains 77..83 off and
74..86 on; IR retains 54.4..60.4 off and 51.4..63.4 on. All bounds are strict.
The moving mean smooths noise within the retained class.

The mode comes from `stepper_enable` driver status, not movement speed or the
idle timeout state. Enabling the first motor widens the intervals without
resetting a confirmed identity or its mean. It also expands the time window;
confirmed identity and power grants remain valid during this warmup. Only the
retained idle history counts towards filling it, even after long idle operation.
Disabling the last enabled motor
clears the filter and requires a complete new acquisition in the narrow band;
wide-band history and a repeated cached ADC report cannot confirm an idle ID.
Partial driver disable does not narrow the bands while any other driver stays
enabled. Detection also works when acquisition begins with motors already on.

The lid does not freeze identification. ADC sampling, conflict detection and
disconnect monitoring continue with the lid closed and in the wider motor band.
Tool changes during offset/raster calibration remain detectable; active grants
must still be ended before changing tools.

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
mean window, including the longer 2.5 s motor window. With overlapping motor
windows a report inside the held ID's retention interval does not count as a
deviation, even if another class also matches; those voltages alone cannot
distinguish noise from a physical change into that overlap. Reports outside the
held interval, finite rails and ADC failures retain their confirmation rules.
During a job, calibration grant or boundary measurement,
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
- `id_filter_size`: configured sample count in count mode; `None` in time mode.
- `id_filter_time`: active time window in seconds; `None` in count mode.
- `id_filter_span`: timestamp difference between oldest and latest buffered
  reports. With 0.75 s configured and 0.1 s reports this is usually 0.7 s.
- `id_filter_ready`: whether the selected window has filled. It can be false
  while a previously confirmed ID is held during motor-on warmup.
- `id_noise_rms`: RMS deviation of accepted values from their arithmetic mean,
  in percentage points; `None` after reset.
- `id_ignored_samples`: cumulative number of reports ignored while retaining an
  established ID since Klippy startup; an initial report of a real change also counts.
- `id_last_sample_ignored`: whether the latest report was omitted as a possible
  isolated outlier. Its raw value remains visible in `id_value`.
- `stable_samples`: consecutive classifications after the window is full.
- `id_sample_age`: age of the latest report.
- `id_candidate`: ID class currently filling the filter, even before confirmation.
- `id_matches`: all acquisition intervals matching the latest raw value,
  independently of retention priority; multiple entries expose ambiguity.
- `id_steppers_on`: true if any stepper driver is enabled; false before ready.
- `id_windows`: effective configuration per class, including center, both
  configured tolerances, active `tolerance`, acquisition `min`/`max` and
  retention `hold_min`/`hold_max`.
- `id_last_reset`: retained context of lost recognition: reason, previous ID,
  raw value, old mean, buffered report count, driver mode and the last three
  distinct raw reports. Later filter refills do not overwrite the original
  cause. A newly confirmed identity can record a subsequent loss.
- `ready`: false while startup has not completed or after shutdown. Early
  queries return unknown identity, no available controls and no power grant;
  they never read ADCs, temperatures or toolhead status before `klippy:ready`.
  Status also remains safe if initialization fails or shutdown occurs early.

Keep machining power physically off for the first check. Compare status with
motors disabled and enabled, then repeat the idle tool changes. After recognition
settles, `id_filter_ready` should be true, the configured duration should match
the driver mode, and the displayed tool should
remain stable during the measured motor noise. Confirm NONE and each individual
tool again. These host tests do not replace the later physical shutdown-latency
measurement. A boundary shutdown includes the detected state, candidate, raw
value, current mean, filter occupancy, driver mode, effective acquisition limits
and last reset reason/recent reports. A message showing UNSTABLE with raw and
mean both 54.774 does not prove that value fell outside the IR interval: it can
be the first sample after prior outliers or an ADC reporting gap cleared the
filter. Inspect the retained reset context instead of widening an already
matching interval.

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
