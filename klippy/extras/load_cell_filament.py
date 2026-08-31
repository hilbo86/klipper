# Load / unload filament using the RFx000 load cell as force feedback.
#
# Designed for use with load_cell_probe_renkforce.py.  The algorithm is
# inspired by the RFx000 CommunityMod M3913/M3914 filament handling, but uses
# Klipper heater/toolhead APIs and configurable force/temperature limits.
#
# Copyright (C) 2026  Timo Hilbig <timo@t-hilbig.de>
# This file may be distributed under the terms of the GNU GPLv3 license.

import collections
import logging


class LoadCellFilament:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.pheaters = self.printer.load_object(config, "heaters")
        self.load_cell = self.printer.lookup_object("load_cell")
        self.monitor_name = config.get(
            "monitor", "extrusion_force_monitor")

        # General thermal / sampling settings
        self.start_temp = config.getfloat("start_temp", 170.0, above=0.0)
        self.max_temp = config.getfloat("max_temp", 260.0, above=0.0)
        if self.max_temp <= self.start_temp:
            raise config.error("max_temp must be greater than start_temp")
        self.temp_step = config.getfloat("temperature_step", 1.0, above=0.0)
        self.temp_step_time = config.getfloat(
            "temperature_step_time", 1.0, above=0.0
        )
        self.sample_time = config.getfloat("sample_time", 0.08, above=0.0)
        self.force_samples = config.getint("force_samples", 3, minval=1)
        self.zero_samples = config.getint("zero_samples", 10, minval=2)
        self.force_safety_limit = config.getfloat(
            "force_safety_limit", 3000.0, above=0.0
        )
        self.operation_timeout = config.getfloat(
            "operation_timeout", 300.0, above=0.0
        )
        self.max_temp_stuck_time = config.getfloat(
            "max_temp_stuck_time", 20.0, above=0.0
        )

        # Unload defaults
        self.unload_target_force = config.getfloat(
            "unload_target_force", -1000.0
        )
        if self.unload_target_force >= 0.0:
            raise config.error("unload_target_force must be negative")
        self.unload_force_tolerance = config.getfloat(
            "unload_force_tolerance", 100.0, above=0.0
        )
        self.unload_release_drop = config.getfloat(
            "unload_release_drop", 300.0, above=0.0
        )
        self.unload_release_motion = config.getfloat(
            "unload_release_motion", 1.0, above=0.0
        )
        self.unload_total_length = config.getfloat(
            "unload_total_length", 90.0, above=0.0
        )
        self.unload_heater_off_after = config.getfloat(
            "unload_heater_off_after", 5.0, minval=0.0
        )
        self.unload_preload_step = config.getfloat(
            "unload_preload_step", 0.25, above=0.0
        )
        self.unload_control_step = config.getfloat(
            "unload_control_step", 0.10, above=0.0
        )
        self.unload_probe_step = config.getfloat(
            "unload_probe_step", 0.05, above=0.0
        )
        self.unload_preload_max = config.getfloat(
            "unload_preload_max", 10.0, above=0.0
        )
        self.unload_control_speed = config.getfloat(
            "unload_control_speed", 1.0, above=0.0
        )
        self.unload_speed = config.getfloat(
            "unload_speed", 8.0, above=0.0
        )
        self.unload_chunk = config.getfloat(
            "unload_chunk", 2.0, above=0.0
        )

        # Load defaults
        self.load_target_force = config.getfloat("load_target_force", 500.0)
        if self.load_target_force <= 0.0:
            raise config.error("load_target_force must be positive")
        self.load_force_tolerance = config.getfloat(
            "load_force_tolerance", 75.0, above=0.0
        )
        self.load_overforce = config.getfloat(
            "load_overforce", 1000.0, above=0.0
        )
        if self.load_overforce <= self.load_target_force:
            raise config.error("load_overforce must exceed load_target_force")
        self.load_total_length = config.getfloat(
            "load_total_length", 50.0, above=0.0
        )
        self.load_heat_hold_length = config.getfloat(
            "load_heat_hold_length", 3.0, above=0.0
        )
        self.load_seek_step = config.getfloat(
            "load_seek_step", 0.50, above=0.0
        )
        self.load_seek_max = config.getfloat(
            "load_seek_max_length", 120.0, above=0.0
        )
        self.load_control_step = config.getfloat(
            "load_control_step", 0.10, above=0.0
        )
        self.load_probe_step = config.getfloat(
            "load_probe_step", 0.05, above=0.0
        )
        self.load_feed_step = config.getfloat(
            "load_feed_step", 1.0, above=0.0
        )
        self.load_min_feed_factor = config.getfloat(
            "load_min_feed_factor", 0.20, above=0.0, maxval=1.0
        )
        self.max_volumetric_speed = config.getfloat(
            "max_volumetric_speed", 10.0, above=0.0
        )
        self.restore_load_temperature = config.getboolean(
            "restore_load_temperature", True
        )
        self.status_interval = config.getfloat(
            "status_interval", 2.0, above=0.0
        )

        self.running = False
        self.tool = None
        self.monitor = None
        self.operation_owner = None
        self.force_baseline = None
        self.force_stream = collections.deque(maxlen=max(
            self.zero_samples * 4, self.force_samples * 4, 32))

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_command(
            "LOAD_FILAMENT", self.cmd_LOAD_FILAMENT,
            desc="Load filament using load-cell force feedback"
        )
        self.gcode.register_command(
            "UNLOAD_FILAMENT", self.cmd_UNLOAD_FILAMENT,
            desc="Unload filament using load-cell force feedback"
        )

    def _handle_ready(self):
        self.tool = self.printer.lookup_object("toolhead")
        self.monitor = self.printer.lookup_object(self.monitor_name, None)

    def _sample_callback(self, sample):
        self.force_stream.append(
            (sample["print_time"], sample["absolute_force_g"]))

    # ------------------------------------------------------------------
    # Common helpers
    # ------------------------------------------------------------------

    def _resolve_extruder(self, gcmd):
        value = gcmd.get("EXTRUDER", None)
        if value is None:
            extr = self.tool.get_extruder()
            name = extr.get_name()
            if not name:
                raise gcmd.error("No active extruder")
            return name, extr

        value = str(value).strip().lower()
        if value in ("0", "e0", "t0", "extruder"):
            name = "extruder"
        elif value in ("1", "e1", "t1", "extruder1"):
            name = "extruder1"
        elif value.startswith("extruder"):
            name = value
        else:
            raise gcmd.error(
                "EXTRUDER must be 0/1, E0/E1, T0/T1 or an extruder name"
            )

        extr = self.printer.lookup_object(name, None)
        if extr is None:
            raise gcmd.error("Extruder '%s' is not configured" % (name,))
        return name, extr

    def _activate_extruder(self, name):
        active = self.tool.get_extruder().get_name()
        if active != name:
            self.gcode.run_script_from_command(
                "ACTIVATE_EXTRUDER EXTRUDER=%s" % (name,)
            )
            self.tool.wait_moves()

    def _set_temperature(self, extr, target, wait=False):
        # Keep a small margin to Klipper's configured heater maximum.
        heater_max = extr.heater.max_temp
        target = min(float(target), heater_max - 1.0)
        target = max(0.0, target)
        self.pheaters.set_temperature(extr.get_heater(), target, wait=wait)
        return target

    def _temperature(self, extr):
        return extr.get_status(self.reactor.monotonic())["temperature"]

    def _target_temperature(self, extr):
        return extr.get_status(self.reactor.monotonic())["target"]

    def _wait_for_extrusion_ready(self, gcmd, extr, timeout=180.0):
        deadline = self.reactor.monotonic() + timeout
        while not extr.get_status(self.reactor.monotonic())["can_extrude"]:
            if self.reactor.monotonic() >= deadline:
                raise gcmd.error("Extruder did not become ready for extrusion")
            self.reactor.pause(self.reactor.monotonic() + 0.25)

    def _zero_force(self, gcmd):
        self.force_stream.clear()
        deadline = self.reactor.monotonic() + max(
            2.0, self.zero_samples * self.sample_time * 4.0)
        while len(self.force_stream) < self.zero_samples:
            if self.reactor.monotonic() >= deadline:
                raise gcmd.error("Timeout collecting load-cell baseline")
            self.reactor.pause(self.reactor.monotonic() + self.sample_time)
        values = [value for _, value
                  in list(self.force_stream)[-self.zero_samples:]]
        self.force_baseline = sum(values) / len(values)
        force = self._read_force()
        gcmd.respond_info(
            "Local load-cell baseline captured; force = %.1fg" % (force,))

    def _read_force(self, samples=None):
        if samples is None:
            samples = self.force_samples
        if self.force_baseline is None:
            raise self.printer.command_error("Load-cell baseline is not set")
        initial_time = self.force_stream[-1][0] if self.force_stream else None
        deadline = self.reactor.monotonic() + max(
            1.0, samples * self.sample_time * 4.0)
        while (len(self.force_stream) < samples
               or (initial_time is not None
                   and self.force_stream[-1][0] <= initial_time)):
            if self.reactor.monotonic() >= deadline:
                raise self.printer.command_error(
                    "Timeout collecting load-cell samples")
            self.reactor.pause(self.reactor.monotonic() + self.sample_time)
        values = [value - self.force_baseline
                  for _, value in list(self.force_stream)[-samples:]]
        return sum(values) / len(values)

    def _check_force(self, gcmd, force):
        if abs(force) >= self.force_safety_limit:
            raise gcmd.error(
                "Load-cell safety limit exceeded: %.1f (limit %.1f)"
                % (force, self.force_safety_limit)
            )

    def _check_timeout(self, gcmd, deadline):
        if self.reactor.monotonic() >= deadline:
            raise gcmd.error("Filament operation timed out")

    def _move_e(self, gcmd, extr, delta, speed):
        if not delta:
            return
        if not extr.get_status(self.reactor.monotonic())["can_extrude"]:
            raise gcmd.error(
                "%s is below min_extrude_temp during filament movement"
                % (extr.get_name(),)
            )
        speed = min(float(speed), extr.max_e_velocity)
        if speed <= 0.0:
            raise gcmd.error("Invalid extrusion speed")
        pos = self.tool.get_position()
        pos[3] += float(delta)
        self.tool.manual_move(pos, speed)
        self.tool.wait_moves()

    def _ramp_temperature(self, extr, target, max_temp):
        new_target = min(target + self.temp_step, max_temp,
                         extr.heater.max_temp - 1.0)
        if new_target > target + 1.0e-6:
            self._set_temperature(extr, new_target, wait=False)
        return new_target

    def _maybe_ramp_temperature(self, extr, set_temp, max_temp,
                                next_temp_step, now):
        """Apply at most one configured temperature step when it is due."""
        if now < next_temp_step or set_temp >= max_temp:
            return set_temp, next_temp_step
        set_temp = self._ramp_temperature(extr, set_temp, max_temp)
        return set_temp, now + self.temp_step_time

    def _status_due(self, now, next_status):
        return now >= next_status

    def _begin_operation(self, gcmd, extr_name, owner):
        if self.running:
            raise gcmd.error(
                "A load-cell filament operation is already running")
        status = self.load_cell.get_status(self.reactor.monotonic())
        if not status.get("is_calibrated", False):
            raise gcmd.error("Load cell must be calibrated in grams")
        previous_extruder = self.tool.get_extruder().get_name()
        if self.monitor is not None:
            self.monitor.claim_operation(owner)
        self.operation_owner = owner
        self.load_cell.add_client(self._sample_callback)
        self.force_stream.clear()
        self.force_baseline = None
        self.running = True
        try:
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE NAME=_LOAD_CELL_FILAMENT_STATE"
            )
            self._activate_extruder(extr_name)
        except Exception:
            self.load_cell.remove_client(self._sample_callback)
            if self.monitor is not None:
                self.monitor.release_operation(owner)
            self.operation_owner = None
            self.running = False
            raise
        return previous_extruder

    def _end_operation(self, previous_extruder):
        try:
            if previous_extruder and self.tool.get_extruder().get_name() != previous_extruder:
                self._activate_extruder(previous_extruder)
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=_LOAD_CELL_FILAMENT_STATE"
            )
        except Exception:
            logging.exception("Unable to restore G-code state after filament operation")
        self.load_cell.remove_client(self._sample_callback)
        if self.monitor is not None and self.operation_owner is not None:
            self.monitor.release_operation(self.operation_owner)
        self.operation_owner = None
        self.force_baseline = None
        self.running = False

    # ------------------------------------------------------------------
    # UNLOAD_FILAMENT
    # ------------------------------------------------------------------

    cmd_UNLOAD_FILAMENT_help = (
        "Unload filament using load-cell tension feedback and a temperature ramp"
    )

    def cmd_UNLOAD_FILAMENT(self, gcmd):
        name, extr = self._resolve_extruder(gcmd)
        start_temp = gcmd.get_float("START_TEMP", self.start_temp, above=0.0)
        max_temp = gcmd.get_float("MAX_TEMP", self.max_temp, above=start_temp)
        target_force = gcmd.get_float(
            "FORCE", self.unload_target_force, maxval=-1.0
        )
        total_length = gcmd.get_float(
            "LENGTH", self.unload_total_length, above=0.0
        )

        previous_extruder = self._begin_operation(
            gcmd, name, "UNLOAD_FILAMENT")
        deadline = self.reactor.monotonic() + self.operation_timeout
        heater_off = False

        try:
            gcmd.respond_info(
                "UNLOAD_FILAMENT %s: start %.1fC, max %.1fC, force %.0f, length %.1fmm"
                % (name, start_temp, max_temp, target_force, total_length)
            )

            # Phase 1: Heat to the cold-pull starting temperature and zero cell.
            self._set_temperature(extr, start_temp, wait=True)
            self._wait_for_extrusion_ready(gcmd, extr)
            self._zero_force(gcmd)
            origin_e = self.tool.get_position()[3]

            # Phase 2: Build the requested tensile force at 170C.
            stable = 0
            while stable < 2:
                self._check_timeout(gcmd, deadline)
                force = self._read_force()
                self._check_force(gcmd, force)
                retracted = origin_e - self.tool.get_position()[3]
                if retracted > self.unload_preload_max:
                    raise gcmd.error(
                        "Unable to build unload force before %.1fmm retraction"
                        % (self.unload_preload_max,)
                    )

                if force > target_force + self.unload_force_tolerance:
                    self._move_e(
                        gcmd, extr, -self.unload_preload_step,
                        self.unload_control_speed
                    )
                    stable = 0
                elif force < target_force - self.unload_force_tolerance:
                    self._move_e(
                        gcmd, extr, self.unload_control_step * 0.5,
                        self.unload_control_speed
                    )
                    stable = 0
                else:
                    stable += 1

            preload_e = self.tool.get_position()[3]
            gcmd.respond_info(
                "Unload preload established: force %.0f, retracted %.2fmm"
                % (self._read_force(), origin_e - preload_e)
            )

            # Phase 3: Slowly raise temperature while maintaining tension.
            # Release is detected either by a distinct force relaxation or by
            # sustained filament motion while the requested force remains held.
            set_temp = start_temp
            next_temp_step = self.reactor.monotonic() + self.temp_step_time
            next_status = self.reactor.monotonic()
            prev_force = self._read_force()
            max_temp_since = None
            released = False

            while not released:
                self._check_timeout(gcmd, deadline)
                now = self.reactor.monotonic()
                force = self._read_force()
                self._check_force(gcmd, force)

                force_relaxation = force - prev_force
                moved_since_preload = preload_e - self.tool.get_position()[3]
                actual_temp = self._temperature(extr)

                if (force_relaxation >= self.unload_release_drop
                        and prev_force <= target_force + self.unload_force_tolerance):
                    released = True
                    reason = "force drop %.0f" % (force_relaxation,)
                elif (moved_since_preload >= self.unload_release_motion
                      and actual_temp >= start_temp + 5.0):
                    released = True
                    reason = "net pull %.2fmm" % (moved_since_preload,)
                else:
                    reason = None

                if self._status_due(now, next_status):
                    gcmd.respond_info(
                        "UNLOAD heat: T=%.1f/%.1fC F=%.0f E=%.2fmm"
                        % (actual_temp, set_temp, force, moved_since_preload)
                    )
                    next_status = now + self.status_interval

                if released:
                    gcmd.respond_info(
                        "Filament release detected at %.1fC (%s), force %.0f"
                        % (actual_temp, reason, force)
                    )
                    break

                set_temp, next_temp_step = self._maybe_ramp_temperature(
                    extr, set_temp, max_temp, next_temp_step, now
                )

                # Closed-loop tension control with deliberate creep.  Without
                # the probe move in the dead band a static elastic preload can
                # remain indefinitely even after the filament has softened.
                if force > target_force + self.unload_force_tolerance:
                    self._move_e(
                        gcmd, extr, -self.unload_control_step,
                        self.unload_control_speed
                    )
                elif force < target_force - self.unload_force_tolerance:
                    self._move_e(
                        gcmd, extr, self.unload_control_step * 0.5,
                        self.unload_control_speed
                    )
                else:
                    self._move_e(
                        gcmd, extr, -self.unload_probe_step,
                        self.unload_control_speed
                    )

                if set_temp >= max_temp - 1.0e-6:
                    if actual_temp >= max_temp - 2.0:
                        if max_temp_since is None:
                            max_temp_since = now
                        elif now - max_temp_since > self.max_temp_stuck_time:
                            raise gcmd.error(
                                "Filament did not release at maximum temperature"
                            )
                else:
                    max_temp_since = None

                prev_force = force

            # Phase 4: Pull the filament out to the requested total distance.
            while origin_e - self.tool.get_position()[3] < total_length:
                self._check_timeout(gcmd, deadline)
                retracted = origin_e - self.tool.get_position()[3]
                remaining = total_length - retracted

                if (not heater_off
                        and retracted >= self.unload_heater_off_after):
                    self._set_temperature(extr, 0.0, wait=False)
                    heater_off = True
                    gcmd.respond_info(
                        "Unload: heater switched off after %.1fmm retraction"
                        % (retracted,)
                    )

                step = min(self.unload_chunk, remaining)
                self._move_e(gcmd, extr, -step, self.unload_speed)
                force = self._read_force(1)
                self._check_force(gcmd, force)

            if not heater_off:
                self._set_temperature(extr, 0.0, wait=False)
                heater_off = True

            gcmd.respond_info(
                "UNLOAD_FILAMENT completed: %.1fmm retracted"
                % (origin_e - self.tool.get_position()[3],)
            )

        except Exception:
            # On any fault, stop heating.  Do not attempt additional E moves.
            try:
                self._set_temperature(extr, 0.0, wait=False)
            except Exception:
                logging.exception("Unable to switch off heater after unload error")
            raise
        finally:
            self._end_operation(previous_extruder)

    # ------------------------------------------------------------------
    # LOAD_FILAMENT
    # ------------------------------------------------------------------

    cmd_LOAD_FILAMENT_help = (
        "Load filament using load-cell compression feedback and a temperature ramp"
    )

    def cmd_LOAD_FILAMENT(self, gcmd):
        name, extr = self._resolve_extruder(gcmd)
        start_temp = gcmd.get_float("START_TEMP", self.start_temp, above=0.0)
        max_temp = gcmd.get_float("MAX_TEMP", self.max_temp, above=start_temp)
        target_force = gcmd.get_float(
            "FORCE", self.load_target_force, above=0.0
        )
        overforce = gcmd.get_float(
            "OVERFORCE", self.load_overforce, above=target_force
        )
        total_length = gcmd.get_float(
            "LENGTH", self.load_total_length, above=0.0
        )
        max_flow = gcmd.get_float(
            "FLOW", self.max_volumetric_speed, above=0.0
        )
        restore_temp = bool(gcmd.get_int(
            "RESTORE_TEMP", 1 if self.restore_load_temperature else 0,
            minval=0, maxval=1
        ))

        previous_extruder = self._begin_operation(
            gcmd, name, "LOAD_FILAMENT")
        deadline = self.reactor.monotonic() + self.operation_timeout
        previous_target = self._target_temperature(extr)

        try:
            # Klipper already calculates filament_area from filament_diameter.
            max_e_speed = min(max_flow / extr.filament_area,
                              extr.max_e_velocity)
            if max_e_speed <= 0.0:
                raise gcmd.error("Calculated load speed is invalid")

            gcmd.respond_info(
                "LOAD_FILAMENT %s: start %.1fC, max %.1fC, force %.0f, "
                "overforce %.0f, length %.1fmm, flow <= %.2fmm^3/s "
                "(E-speed <= %.3fmm/s)"
                % (name, start_temp, max_temp, target_force, overforce,
                   total_length, max_flow, max_e_speed)
            )

            # Phase 1: Heat to 170C and zero the cell.
            self._set_temperature(extr, start_temp, wait=True)
            self._wait_for_extrusion_ready(gcmd, extr)
            self._zero_force(gcmd)
            seek_origin_e = self.tool.get_position()[3]

            # Phase 2: Feed until the filament builds the initial +force.
            stable = 0
            while stable < 2:
                self._check_timeout(gcmd, deadline)
                force = self._read_force()
                self._check_force(gcmd, force)
                advanced = self.tool.get_position()[3] - seek_origin_e
                if advanced > self.load_seek_max:
                    raise gcmd.error(
                        "No load-cell contact before %.1fmm feed"
                        % (self.load_seek_max,)
                    )

                if force < target_force - self.load_force_tolerance:
                    self._move_e(
                        gcmd, extr, self.load_seek_step, max_e_speed
                    )
                    stable = 0
                elif force > target_force + self.load_force_tolerance:
                    self._move_e(
                        gcmd, extr, -self.load_control_step * 0.5,
                        max_e_speed * self.load_min_feed_factor
                    )
                    stable = 0
                else:
                    stable += 1

            # This is the requested logical E=0 point.  It is kept local to
            # this module so a paused print's G-code extrusion coordinate is
            # not destroyed by a G92 command.
            load_zero_e = self.tool.get_position()[3]
            gcmd.respond_info(
                "Load force reached; local extrusion length zeroed at force %.0f"
                % (self._read_force(),)
            )

            # Phase 3: Continue heating while force control advances filament.
            # The normal temperature ramp stops once 3mm net feed is achieved.
            set_temp = start_temp
            next_temp_step = self.reactor.monotonic() + self.temp_step_time
            next_status = self.reactor.monotonic()
            max_temp_since = None

            while (self.tool.get_position()[3] - load_zero_e
                   < self.load_heat_hold_length):
                self._check_timeout(gcmd, deadline)
                now = self.reactor.monotonic()
                force = self._read_force()
                self._check_force(gcmd, force)
                progress = self.tool.get_position()[3] - load_zero_e
                actual_temp = self._temperature(extr)

                set_temp, next_temp_step = self._maybe_ramp_temperature(
                    extr, set_temp, max_temp, next_temp_step, now
                )

                if self._status_due(now, next_status):
                    gcmd.respond_info(
                        "LOAD heat: T=%.1f/%.1fC F=%.0f E=%.2fmm"
                        % (actual_temp, set_temp, force, progress)
                    )
                    next_status = now + self.status_interval

                if force > overforce:
                    # Relieve excessive compression.  Temperature may continue
                    # to rise, but only through the normal timed ramp above.
                    self._move_e(
                        gcmd, extr, -self.load_control_step,
                        max_e_speed * self.load_min_feed_factor
                    )
                elif force < target_force - self.load_force_tolerance:
                    self._move_e(
                        gcmd, extr, self.load_control_step, max_e_speed
                    )
                elif force > target_force + self.load_force_tolerance:
                    self._move_e(
                        gcmd, extr, -self.load_control_step * 0.5,
                        max_e_speed * self.load_min_feed_factor
                    )
                else:
                    # Deliberate creep in the force dead band.  A rigid plug
                    # pushes the force back up; softened filament permits net
                    # forward motion and the 3mm progress criterion can complete.
                    self._move_e(
                        gcmd, extr, self.load_probe_step,
                        max_e_speed * self.load_min_feed_factor
                    )

                if set_temp >= max_temp - 1.0e-6:
                    if actual_temp >= max_temp - 2.0:
                        if max_temp_since is None:
                            max_temp_since = now
                        elif now - max_temp_since > self.max_temp_stuck_time:
                            raise gcmd.error(
                                "Unable to advance 3mm at maximum temperature"
                            )
                else:
                    max_temp_since = None

            gcmd.respond_info(
                "Load: %.2fmm advanced; normal temperature ramp held at %.1fC"
                % (self.tool.get_position()[3] - load_zero_e, set_temp)
            )

            # Phase 4: Feed to 50mm total.  Temperature remains at the reached
            # target unless force exceeds OVERFORCE.  Feed speed is continuously
            # reduced as force approaches OVERFORCE and never exceeds FLOW.
            overforce_since = None
            next_status = self.reactor.monotonic()
            # Normal heating stopped at 3mm.  Keep the old ramp deadline so
            # overforce heating can still increase at no more than the configured
            # temperature_step / temperature_step_time.
            next_temp_step = max(
                next_temp_step, self.reactor.monotonic() + self.temp_step_time
            )

            while self.tool.get_position()[3] - load_zero_e < total_length:
                self._check_timeout(gcmd, deadline)
                now = self.reactor.monotonic()
                force = self._read_force()
                self._check_force(gcmd, force)
                progress = self.tool.get_position()[3] - load_zero_e
                remaining = total_length - progress
                actual_temp = self._temperature(extr)

                if self._status_due(now, next_status):
                    gcmd.respond_info(
                        "LOAD feed: T=%.1f/%.1fC F=%.0f E=%.2f/%.2fmm"
                        % (actual_temp, set_temp, force, progress, total_length)
                    )
                    next_status = now + self.status_interval

                if force > overforce:
                    # Raise temperature only at the configured ramp rate, and
                    # keep creeping forward at the minimum rate.  This avoids
                    # the previous deadlock where a static high force could stop
                    # all extrusion indefinitely.
                    set_temp, next_temp_step = self._maybe_ramp_temperature(
                        extr, set_temp, max_temp, next_temp_step, now
                    )
                    if overforce_since is None:
                        overforce_since = now

                    step = min(
                        self.load_feed_step * self.load_min_feed_factor,
                        remaining
                    )
                    self._move_e(
                        gcmd, extr, step,
                        max_e_speed * self.load_min_feed_factor
                    )

                    if (set_temp >= max_temp - 1.0e-6
                            and actual_temp >= max_temp - 2.0
                            and now - overforce_since
                            > self.max_temp_stuck_time):
                        raise gcmd.error(
                            "Persistent load overforce at maximum temperature"
                        )
                else:
                    overforce_since = None
                    if force <= target_force + self.load_force_tolerance:
                        speed_factor = 1.0
                    else:
                        span = max(
                            1.0,
                            overforce -
                            (target_force + self.load_force_tolerance)
                        )
                        ratio = (
                            force -
                            (target_force + self.load_force_tolerance)
                        ) / span
                        speed_factor = 1.0 - ratio * (
                            1.0 - self.load_min_feed_factor
                        )
                        speed_factor = max(
                            self.load_min_feed_factor,
                            min(1.0, speed_factor)
                        )

                    step = min(self.load_feed_step, remaining)
                    self._move_e(
                        gcmd, extr, step, max_e_speed * speed_factor
                    )

            gcmd.respond_info(
                "LOAD_FILAMENT completed: %.1fmm extruded after force contact; "
                "final target %.1fC"
                % (self.tool.get_position()[3] - load_zero_e, set_temp)
            )

            if restore_temp:
                self._set_temperature(extr, previous_target, wait=False)
                gcmd.respond_info(
                    "Restored previous heater target: %.1fC" % (previous_target,)
                )

        except Exception:
            # Restore the previous target on a load error.  This is preferable
            # to silently leaving an unexpectedly high ramp temperature active.
            try:
                self._set_temperature(extr, previous_target, wait=False)
            except Exception:
                logging.exception("Unable to restore heater after load error")
            raise
        finally:
            self._end_operation(previous_extruder)


def load_config(config):
    return LoadCellFilament(config)
