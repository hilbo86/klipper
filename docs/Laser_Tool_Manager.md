# Laser tool ID filtering

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
