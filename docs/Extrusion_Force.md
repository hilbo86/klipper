# Extrusion force monitoring and control

This extension reuses the Renkforce probe load cell as a sensor for the complete
extrusion system. It time-aligns every ADC sample with the extruder TrapQ at the
sample's original Klipper `print_time`, derives volumetric flow, and compares
measured force with a material/hotend/nozzle profile.

## Safety model

The protection layers remain independent:

1. `max_abs_force` is the MCU-adjacent final force limit.
2. `extrusion_force_guard` confirms delivery failure or jam and may pause.
3. Adaptive speed reduces only positive-extrusion move speed.
4. Optional adaptive temperature reacts only after sustained speed limiting.

Never use adaptive control as a replacement for `max_abs_force`. Guard and
control thresholds deliberately have no guessed detection defaults; derive them
from recorded data before enabling either module.

## Initial setup

1. Configure and calibrate `[load_cell_probe_renkforce]`. `force_calibration`
   is grams per ADC unit; use `sensor_orientation` so applied extrusion force
   has the intended sign. Confirm `printer.load_cell.is_calibrated` and
   `printer.load_cell.force_g` in Mainsail.
2. Add `[extrusion_force_monitor]` only. Leave guard and adaptive features off.
3. Record the `extrusion_force/dump` stream during safe test extrusions. Confirm
   that force follows `flow_mm3_s`, baseline remains stable during extrusion,
   and `EXTRUSION_STEADY` is plausible.
4. Create one `[extrusion_force_profile <name>]` per material/extruder/hotend/
   nozzle combination and run `FORCE_FLOW_CALIBRATE` with conservative
   `ABORT_FORCE`, flows, and temperatures.
5. Review the generated points and recommended limits, then run `SAVE_CONFIG`.
   Select profiles from filament start G-code with
   `SET_EXTRUSION_FORCE_PROFILE`.

Calibration data uses piecewise-linear flow interpolation followed by linear
temperature interpolation. Values outside the calibrated domain return no
expected force; automatic control does not silently extrapolate them.

## Fault detection

The guard evaluates only sufficiently confident `EXTRUSION_STEADY` samples.
Delivery failure requires all of the following: commanded flow, meaningful
expected force, measured underload, minimum elapsed time, and minimum commanded
filament distance. It therefore does not classify travel, retract, a single
flow transient, or very slow extrusion as runout.

A soft positive excess-force margin emits an overload event for adaptive
control. A separately configured hard margin must persist before it is called a
jam. The long-term health EWMA is informative and does not pause by itself.

## Z sensing

With a valid profile, `z_sense_offset` controls on
`measured_force - expected_dynamic_force`. Its threshold is the maximum of a
minimum margin, measured-noise multiple, and relative expected-force margin.
It reacts only to confident steady extrusion below `max_z_height` and can only
raise Z. Without a usable profile, the existing absolute `force_threshold`
remains available as a compatibility fallback.

`Z_FORCE_CALIBRATE` is intentionally conservative but physically moves the
nozzle closer in stepped test lines. Start from a known safe first layer, keep
the MCU force limit active, provide a conservative `ABORT_FORCE`, and inspect
the printed lines before accepting the staged configuration.

## Motor current and torque fuse

`EXTRUDER_CURRENT_CALIBRATE` measures stable and peak force at a current sweep,
then chooses the lowest current that exceeds profile-required force plus
reserve. An optional measured `grind_force_limit` can reject candidates that do
not preserve a mechanical safety margin. Load-cell force alone cannot prove
rotor stall versus drive-gear slip; status therefore uses the neutral concept
of drive-force limit. StallGuard data is included only when the driver exposes
it. The original current is restored even after an error.

## Adaptive control

Speed control is a queue-latency-aware state machine (`NORMAL`, `LIMITING`,
`RECOVERY`, `HARD_FAULT`). Its chained move transform multiplies speed only when
the E coordinate increases; travel, retract, and the user's M220 setting remain
independent. Recovery is deliberately slower than limiting.

Temperature assistance is disabled by default. It steps upward only after
sustained overload at minimum speed and respects the heater limit, profile
material limit, and `base_target + max_temperature_increase`. An M104/M109
target change resets the adaptive baseline. Disabling temperature assistance
restores the current base target.

## Existing load-cell operations

`PRESSURE_PRIME`, `LOAD_FILAMENT`, and `UNLOAD_FILAMENT` subscribe to
timestamped samples, establish private baselines from `absolute_force_g`, and
never call global tare. They and all calibration commands use a shared
operation lock, so only one mechanical load-cell operation can run at once.
Subscribers are removed and original targets/state are restored in `finally`
paths.

## Diagnostics and replay

`EXTRUSION_FORCE_DIAGNOSTIC` compares a repeatable reference extrusion against
the active profile. The optional collision detector is experimental and log
only; it requires a force impulse, force derivative, XY motion, and negligible
flow while excluding extrusion transients.

`EXTRUSION_FORCE_PA_ANALYZE` compares force rise, overshoot, settling, and decay
for a requested PA list. It restores the original value and reports a range for
subsequent visual prints; it never runs `SAVE_CONFIG` because the optically best
PA is not necessarily the force-response optimum.

Offline tools may feed CSV-like rows containing `print_time`, `force`, `flow`,
`temperature`, and `extruder` through
`extrusion_force_monitor.replay_rows()`. This uses the same processor as live
monitoring, allowing filter and detector changes to be evaluated against real
recordings without reprinting.

## Troubleshooting

- No Mainsail force: verify explicit `force_calibration`; the legacy internal
  default does not claim calibrated grams.
- Expected force is null: select a matching profile and stay inside both its
  calibrated flow and temperature ranges.
- Confidence is low: wait for a safe baseline and stable temperature/flow, and
  inspect `noise_g`.
- False delivery failures: increase the measured underload time/distance or
  minimum monitored flow; do not weaken MCU force protection.
- Transforms conflict: both included transforms chain the previous transform.
  A third-party transform must do the same instead of replacing it silently.
