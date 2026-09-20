# Extrusion force monitoring and control

This extension reuses the Renkforce probe load cell as a sensor for the complete
extrusion system. It time-aligns every ADC sample with the extruder TrapQ at the
sample's original Klipper `print_time`, derives volumetric flow, and compares
measured force with a filament/nozzle profile.

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
4. Put printer-independent material data in `[filament_profile <name>]`
   sections, typically in an included `filaments.cfg`. Create one
   `[extrusion_force_profile <name>]` per filament/nozzle combination
   and reference the filament with `valid_for`. Equivalent tools can share it
   with a comma-separated `extruder` list; use a single-extruder profile only
   for a measured hardware difference. The force profile's `nozzle_diameter`
   must match the relevant extruder section. Run
   `FORCE_FLOW_CALIBRATE` with conservative `ABORT_FORCE`, flows, and
   temperatures. Specify `EXTRUDER` when calibrating a shared profile.
   Store pressure advance in the printer-specific force profile when it should
   be applied automatically with the filament selection; do not put it in the
   portable filament profile.
5. Review the generated points and recommended limits, then run `SAVE_CONFIG`.
   Select the portable filament identity from filament start G-code with, for
   example, `SET_FILAMENT_PROFILE PROFILE=F01_ASA_Apollox`. The matching
   printer-specific force profile is selected for the active extruder. Direct
   selection with `SET_EXTRUSION_FORCE_PROFILE` remains available for
   calibration and diagnostics.

Calibration data uses piecewise-linear flow interpolation followed by linear
temperature interpolation. Values outside the calibrated domain return no
expected force; automatic control does not silently extrapolate them.
In normal length-based G-code, the slicer has already converted the intended
extrusion volume into E-axis filament length; the diameter itself is not
available to the firmware. An optional `filament_diameter` in the filament
profile must therefore match the slicer setting. The extrusion-force monitor
and its calibration commands use it to reconstruct volumetric flow. If it is
omitted, they use `[extruder]`'s nominal diameter. Klipper's core kinematics
and extrusion limits remain unchanged. Do not enable the slicer's volumetric-E
or `M200` mode for Klipper. `temperature_tolerance` belongs to the force profile
because it controls use of the calibrated temperature domain.

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

### Filament that moves below the target force

Loading and unloading also compare force **during** a short extruder move with
settled idle samples before and after it. Compression must increase for loading;
tension must increase for unloading. Two consecutive probes must show a
repeatable difference above both the configured minimum and three times the
measured idle noise. The moving-force median rejects isolated impulses. All
samples must remain below the requested force magnitude for this alternative
completion criterion. A static elastic preload, sensor drift, insufficient
samples, or motion in the wrong force direction does not confirm transport.

`UNLOAD_FILAMENT` proceeds with the remaining pull when motion is detected,
without requiring the holding force or a higher starting temperature.
`LOAD_FILAMENT` leaves the contact search and holds the starting temperature if
filament already moves. Feed from the first of the confirming probes counts
toward `LENGTH`. If contact initially requires the usual target force, the
temperature ramp still runs; subsequent low-force motion can stop that ramp.
The absolute force safety limit is checked on individual samples, including
those received during motion, and prevents further moves after a violation.

Both `[extrusion_force_filament_changer]` and `[extrusion_force_priming]` accept
these optional settings (defaults shown):

```ini
motion_force_delta: 25.0
# Minimum directional moving/idle force difference, in grams.
motion_confirmations: 2
# Consecutive qualifying probes; at least two.
motion_sample_time: 0.3
# Minimum duration of a probe move and each idle sampling window, in seconds.
motion_settle_time: 0.2
# Wait before sampling stationary force, in seconds.
motion_sample_timeout: 2.0
# Maximum wait for delivery of timestamped samples after a window has ended.
```

Probe speeds are reduced as needed to obtain multiple measurements even with a
16 SPS cell; they remain within the configured extrusion speed/flow limits.
Each window needs at least three samples. Increase `motion_sample_time` for
slower sensors and `motion_settle_time` if force takes longer to relax. Force
windows use the ADC sample's `print_time`, including delayed callbacks.

### Low-force priming and heater demand

`PRESSURE_PRIME` still succeeds on two stable segments above `THRESHOLD`.
Below that threshold, motion/idle force evidence must additionally coincide
with a sustained increase in heater power over idle demand. This prevents
friction from filament that has not yet reached the melt zone from being enough
to report successful priming.

After heating to `TARGET_TEMP`, priming records a stable idle temperature and
mean heater power before feeding. During priming it samples power every 100 ms
and compares a rolling mean, including short measurement pauses, with that
baseline. Including pauses accommodates the heater's delayed response to cold
filament. Temperature must stay near the same target, and the power increase
must exceed both the configured minimum and three times idle power noise in
both halves of the observation window, so one PWM peak is insufficient.
The command reports idle power and the required increase for tuning.

Additional optional `[extrusion_force_priming]` settings (defaults shown):

```ini
thermal_baseline_time: 5.0
# Stable idle observation duration, in seconds.
thermal_settle_timeout: 30.0
# Maximum baseline settling time; must exceed thermal_baseline_time.
thermal_confirm_time: 2.0
# Duration of the rolling heater-demand comparison, in seconds.
thermal_power_delta: 0.02
# Minimum increase in heater PWM duty, from 0 to 1: 0.02 is 2 percentage points.
thermal_temperature_tolerance: 2.0
# Permitted deviation from TARGET_TEMP, in degrees C.
```

The idle baseline also requires a temperature range no greater than half the
tolerance, a net temperature change no greater than a quarter of it, and a
difference between the first and second halves' mean power no greater than
`thermal_power_delta`. This excludes the tail of the initial heating ramp.
If no stable baseline is available before the settling timeout, only the
original force-threshold criterion can succeed. Low force alone, increased
power alone, or a temperature drop alone does not confirm priming. Length and
force limits remain active, including a fractional final millimeter of `LENGTH`.
Tune the new thresholds against recorded behavior of the actual hotend; this
is an inference of transport, not a direct measurement of filament motion.

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
