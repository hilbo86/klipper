# Extruder motor-current calibration using extrusion force
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import statistics


def select_run_current(curve, required_force, reserve=1.2,
                       grind_force_limit=None, grind_safety_factor=0.9):
    target = required_force * reserve
    for point in sorted(curve, key=lambda item: item["current"]):
        drive_force = point["stable_force"]
        if drive_force < target:
            continue
        if (grind_force_limit is not None
                and drive_force >= grind_force_limit * grind_safety_factor):
            continue
        return point["current"]
    return None


class ExtruderDriverAdapter:
    def __init__(self, printer, gcode, extruder_name, driver_name=None):
        self.printer = printer
        self.gcode = gcode
        self.extruder_name = extruder_name
        self.driver_name, self.driver = self._find_driver(driver_name)
        self.original_current = None

    def _find_driver(self, driver_name):
        if driver_name is not None:
            driver = self.printer.lookup_object(driver_name, None)
            if driver is None:
                raise self.printer.command_error(
                    "Unknown TMC driver '%s'" % (driver_name,))
            return driver_name, driver
        matches = []
        for name, obj in self.printer.lookup_objects():
            if (name.startswith("tmc")
                    and name.split()[-1] == self.extruder_name
                    and hasattr(obj, "get_status")):
                matches.append((name, obj))
        if len(matches) != 1:
            raise self.printer.command_error(
                "Expected one TMC driver for '%s', found %d"
                % (self.extruder_name, len(matches)))
        return matches[0]

    def get_run_current(self):
        status = self.driver.get_status(self.printer.get_reactor().monotonic())
        current = status.get("run_current")
        if current is None:
            raise self.printer.command_error(
                "TMC driver '%s' does not report run_current"
                % (self.driver_name,))
        return float(current)

    def set_run_current(self, current):
        if self.original_current is None:
            self.original_current = self.get_run_current()
        self.gcode.run_script_from_command(
            "SET_TMC_CURRENT STEPPER=%s CURRENT=%.6f"
            % (self.extruder_name, current))

    def restore_run_current(self):
        if self.original_current is not None:
            self.gcode.run_script_from_command(
                "SET_TMC_CURRENT STEPPER=%s CURRENT=%.6f"
                % (self.extruder_name, self.original_current))
            self.original_current = None

    def get_stallguard(self):
        status = self.driver.get_status(self.printer.get_reactor().monotonic())
        driver_status = status.get("drv_status") or {}
        for key in ("sg_result", "SG_RESULT", "stallguard"):
            if key in driver_status:
                return driver_status[key]
        return None


class ExtruderForceCurrentCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.monitor_name = config.get(
            "monitor", "extrusion_force_monitor")
        self.driver_name = config.get("driver", None)
        self.force_reserve = config.getfloat(
            "force_reserve", 1.2, above=1.0)
        self.grind_force_limit = config.getfloat(
            "grind_force_limit", None, above=0.0)
        self.grind_safety_factor = config.getfloat(
            "grind_safety_factor", 0.9, above=0.0, below=1.0)
        self.settle_time = config.getfloat(
            "settle_time", 0.5, minval=0.0)
        self.measure_time = config.getfloat(
            "measure_time", 1.0, above=0.0)
        self.last_curve = []
        self.recommended_current = None
        self.samples = []
        self.collection_window = None
        self.force_ceiling = float("inf")
        self.safety_error = None
        self.gcode.register_command(
            "EXTRUDER_CURRENT_CALIBRATE",
            self.cmd_EXTRUDER_CURRENT_CALIBRATE,
            desc="Calibrate extruder run current from measured force")

    def _state_callback(self, state):
        if abs(state["force_fast_g"]) >= self.force_ceiling:
            self.safety_error = "FORCE_CEILING exceeded"
        if self.collection_window is None:
            return
        start, end = self.collection_window
        if start <= state["print_time"] <= end:
            self.samples.append(state)

    def _extrude(self, toolhead, extruder, flow, duration):
        velocity = flow / extruder.filament_area
        start = toolhead.get_last_move_time()
        position = toolhead.get_position()
        position[3] += velocity * duration
        self.samples = []
        toolhead.manual_move(position, velocity)
        end = toolhead.get_last_move_time()
        self.collection_window = (start + self.settle_time, end)
        toolhead.wait_moves()
        self.collection_window = None
        if self.safety_error is not None:
            raise self.printer.command_error(self.safety_error)
        values = [state["force_control_g"] for state in self.samples]
        if len(values) < 2:
            raise self.printer.command_error(
                "Insufficient samples during current calibration")
        return max(values), statistics.mean(values)

    def _required_force(self, profile):
        usable = []
        for point in profile.points:
            limit = profile.get_recommended_max_flow(point["temperature"])
            if limit is None or point["flow"] <= limit:
                usable.append(point["mean_force"])
        return max(usable) if usable else None

    def cmd_EXTRUDER_CURRENT_CALIBRATE(self, gcmd):
        monitor = self.printer.lookup_object(self.monitor_name)
        manager = self.printer.lookup_object(
            "extrusion_force_profile_manager", None)
        profile_name = gcmd.get("PROFILE")
        profile = manager.get_profile(profile_name) if manager is not None else None
        if profile is None:
            raise gcmd.error(
                "Unknown extrusion force profile '%s'" % (profile_name,))
        extruder_name = gcmd.get("EXTRUDER", profile.extruder)
        if extruder_name != profile.extruder:
            raise gcmd.error(
                "Profile '%s' belongs to '%s'"
                % (profile.name, profile.extruder))
        extruder = self.printer.lookup_object(extruder_name)
        adapter = ExtruderDriverAdapter(
            self.printer, self.gcode, extruder_name, self.driver_name)
        current_min = gcmd.get_float("CURRENT_MIN", above=0.0)
        current_max = gcmd.get_float("CURRENT_MAX", minval=current_min)
        current_step = gcmd.get_float("CURRENT_STEP", above=0.0)
        temperature = gcmd.get_float(
            "TEMPERATURE", minval=extruder.heater.min_extrude_temp,
            below=extruder.heater.max_temp)
        self.force_ceiling = gcmd.get_float("FORCE_CEILING", above=0.0)
        repeats = gcmd.get_int("REPEATS", 2, minval=1)
        flow_start = gcmd.get_float("FLOW_START", above=0.0)
        flow_max = gcmd.get_float("FLOW_MAX", minval=flow_start)
        flow_step = gcmd.get_float("FLOW_STEP", above=0.0)
        required_force = gcmd.get_float(
            "REQUIRED_FORCE", self._required_force(profile), above=0.0)
        if required_force is None:
            raise gcmd.error(
                "REQUIRED_FORCE is needed when the profile has no data")
        save = bool(gcmd.get_int("SAVE", 0, minval=0, maxval=1))
        owner = "EXTRUDER_CURRENT_CALIBRATE"
        monitor.claim_operation(owner)
        original_target = extruder.get_status(
            self.reactor.monotonic())["target"]
        monitor.add_client(self._state_callback)
        try:
            toolhead = self.printer.lookup_object("toolhead")
            kin_status = toolhead.get_kinematics().get_status(
                self.reactor.monotonic())
            if "z" not in kin_status.get("homed_axes", ""):
                raise gcmd.error("Z must be homed before current calibration")
            self.gcode.run_script_from_command(
                "ACTIVATE_EXTRUDER EXTRUDER=%s" % (extruder_name,))
            heaters = self.printer.lookup_object("heaters")
            heaters.set_temperature(extruder.get_heater(), temperature, True)
            currents = []
            current = current_min
            while current <= current_max + 1e-9:
                currents.append(current)
                current += current_step
            flows = []
            flow = flow_start
            while flow <= flow_max + 1e-9:
                flows.append(flow)
                flow += flow_step
            curve = []
            for current in currents:
                adapter.set_run_current(current)
                self.reactor.pause(self.reactor.monotonic() + 0.2)
                repeat_forces = []
                repeat_peaks = []
                stallguard = []
                for repeat in range(repeats):
                    stable = peak = 0.0
                    for flow in flows:
                        self.safety_error = None
                        measured_peak, measured_stable = self._extrude(
                            toolhead, extruder, flow,
                            self.settle_time + self.measure_time)
                        peak = max(peak, measured_peak)
                        stable = max(stable, measured_stable)
                    repeat_forces.append(stable)
                    repeat_peaks.append(peak)
                    sg_result = adapter.get_stallguard()
                    if sg_result is not None:
                        stallguard.append(sg_result)
                point = {
                    "current": current,
                    "stable_force": statistics.median(repeat_forces),
                    "peak_force": statistics.median(repeat_peaks),
                    "stallguard": (statistics.median(stallguard)
                                   if stallguard else None),
                }
                curve.append(point)
                gcmd.respond_info(
                    "I=%.3fA stable=%.1fg peak=%.1fg"
                    % (current, point["stable_force"], point["peak_force"]))
            result = select_run_current(
                curve, required_force, self.force_reserve,
                self.grind_force_limit, self.grind_safety_factor)
            if result is None:
                raise gcmd.error(
                    "No tested current provides the required force reserve")
            self.last_curve = curve
            self.recommended_current = result
            gcmd.respond_info(
                "Recommended %s run_current: %.3fA (required force %.1fg, "
                "reserve %.2f)" % (
                    extruder_name, result, required_force, self.force_reserve))
            if save:
                configfile = self.printer.lookup_object("configfile")
                configfile.set(adapter.driver_name, "run_current", "%.3f" % result)
                gcmd.respond_info(
                    "run_current staged; run SAVE_CONFIG after review")
        finally:
            self.collection_window = None
            monitor.remove_client(self._state_callback)
            monitor.release_operation(owner)
            adapter.restore_run_current()
            self.printer.lookup_object("heaters").set_temperature(
                extruder.get_heater(), original_target, False)

    def get_status(self, eventtime):
        return {
            "recommended_current": self.recommended_current,
            "curve": list(self.last_curve),
        }


def load_config(config):
    return ExtruderForceCurrentCalibration(config)
