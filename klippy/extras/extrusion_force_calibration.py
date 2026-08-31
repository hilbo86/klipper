# Automated extrusion force/flow and response calibration
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import statistics

from .extrusion_force_profile import detect_knee


def estimate_response_tau(samples, initial_force, final_force, rising=True):
    """Estimate a first-order response tau from timestamped force samples."""
    if not samples or final_force == initial_force:
        return None
    target = initial_force + 0.6321205588 * (final_force - initial_force)
    start_time = samples[0][0]
    previous = samples[0]
    for current in samples[1:]:
        crossed = (current[1] >= target if rising else current[1] <= target)
        if not crossed:
            previous = current
            continue
        if current[1] == previous[1]:
            return max(0.0, current[0] - start_time)
        crossing_time = previous[0] + (
            (target - previous[1]) / (current[1] - previous[1])
            * (current[0] - previous[0]))
        return max(0.0, crossing_time - start_time)
    return None


class ExtrusionForceCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.monitor_name = config.get(
            "monitor", "extrusion_force_monitor")
        self.minimum_z_height = config.getfloat(
            "minimum_z_height", 5.0, minval=0.0)
        self.baseline_time = config.getfloat(
            "baseline_time", 1.0, above=0.0)
        self.purge_length = config.getfloat(
            "purge_length", 5.0, minval=0.0)
        self.purge_flow = config.getfloat(
            "purge_flow", 2.0, above=0.0)
        self.abort_force = config.getfloat(
            "abort_force", None, above=0.0)
        self.abort_force_rate = config.getfloat(
            "abort_force_rate", None, above=0.0)
        self.minimum_knee_slope_ratio = config.getfloat(
            "minimum_knee_slope_ratio", 2.0, above=1.0)
        self.minimum_knee_fit_improvement = config.getfloat(
            "minimum_knee_fit_improvement", 0.25,
            minval=0.0, maxval=1.0)
        self.measurements = []
        self.collection_window = None
        self.safety_error = None
        self._active_abort_force = self.abort_force or float("inf")
        self.gcode.register_command(
            "FORCE_FLOW_CALIBRATE", self.cmd_FORCE_FLOW_CALIBRATE,
            desc="Calibrate extrusion force over flow and temperature")
        self.gcode.register_command(
            "FORCE_RESPONSE_CALIBRATE", self.cmd_FORCE_RESPONSE_CALIBRATE,
            desc="Calibrate extrusion-force rise and fall response")

    def _objects(self):
        monitor = self.printer.lookup_object(self.monitor_name)
        manager = self.printer.lookup_object(
            "extrusion_force_profile_manager", None)
        if manager is None:
            raise self.printer.command_error(
                "No extrusion_force_profile is configured")
        return monitor, manager

    def _resolve(self, gcmd):
        monitor, manager = self._objects()
        profile_name = gcmd.get("PROFILE")
        profile = manager.get_profile(profile_name)
        if profile is None:
            raise gcmd.error(
                "Unknown extrusion force profile '%s'" % (profile_name,))
        extruder_name = gcmd.get("EXTRUDER", profile.extruder)
        if extruder_name != profile.extruder:
            raise gcmd.error(
                "Profile '%s' belongs to extruder '%s'"
                % (profile.name, profile.extruder))
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None:
            raise gcmd.error("Unknown extruder '%s'" % (extruder_name,))
        return monitor, profile, extruder_name, extruder

    def _check_prerequisites(self, gcmd, monitor, extruder_name):
        load_cell = monitor.load_cell
        if load_cell is None:
            raise gcmd.error("Extrusion force monitor is not ready")
        status = load_cell.get_status(self.reactor.monotonic())
        if not status.get("is_calibrated", False):
            raise gcmd.error("Load cell must be calibrated in grams")
        toolhead = self.printer.lookup_object("toolhead")
        kin_status = toolhead.get_kinematics().get_status(
            self.reactor.monotonic())
        if "z" not in kin_status.get("homed_axes", ""):
            raise gcmd.error("Z must be homed before force calibration")
        if toolhead.get_position()[2] < self.minimum_z_height:
            raise gcmd.error(
                "Move nozzle to at least %.3fmm Z before force calibration"
                % (self.minimum_z_height,))
        self.gcode.run_script_from_command(
            "ACTIVATE_EXTRUDER EXTRUDER=%s" % (extruder_name,))
        return toolhead

    def _parse_temperatures(self, gcmd, profile):
        value = gcmd.get("TEMPERATURES", None)
        if value is None:
            raise gcmd.error("TEMPERATURES must be specified")
        try:
            temperatures = [float(item.strip()) for item in value.split(",")]
        except ValueError:
            raise gcmd.error("Invalid TEMPERATURES list")
        if not temperatures:
            raise gcmd.error("TEMPERATURES must not be empty")
        heater = self.printer.lookup_object(profile.extruder).get_heater()
        maximum = heater.max_temp
        if profile.max_material_temperature is not None:
            maximum = min(maximum, profile.max_material_temperature)
        for temperature in temperatures:
            if temperature < heater.min_extrude_temp or temperature >= maximum:
                raise gcmd.error(
                    "Calibration temperature %.1f is outside safe limits"
                    % (temperature,))
        return temperatures

    def _handle_state(self, state):
        abort_force = self._active_abort_force
        if abs(state["force_control_g"]) >= abort_force:
            self.safety_error = "ABORT_FORCE exceeded"
        if (self.abort_force_rate is not None
                and abs(state["dforce_dt"]) >= self.abort_force_rate):
            self.safety_error = "abort_force_rate exceeded"
        if self.collection_window is None:
            return
        start, end = self.collection_window
        if start <= state["print_time"] <= end:
            self.measurements.append(state)

    def _set_temperature(self, extruder, temperature, wait=True):
        heaters = self.printer.lookup_object("heaters")
        heaters.set_temperature(extruder.get_heater(), temperature, wait)

    def _wait_baseline(self):
        self.reactor.pause(self.reactor.monotonic() + self.baseline_time)

    def _extrude(self, toolhead, extruder, flow, duration,
                 settle_time=0.0, collect=False):
        e_velocity = flow / extruder.filament_area
        start_time = toolhead.get_last_move_time()
        position = toolhead.get_position()
        position[3] += e_velocity * duration
        self.measurements = []
        self.collection_window = None
        toolhead.manual_move(position, e_velocity)
        end_time = toolhead.get_last_move_time()
        if collect:
            self.collection_window = (start_time + settle_time, end_time)
        toolhead.wait_moves()
        self.collection_window = None
        if self.safety_error is not None:
            raise self.printer.command_error(self.safety_error)
        return list(self.measurements)

    def _measure_flow(self, toolhead, extruder, flow, settle_time,
                      measure_time):
        samples = self._extrude(
            toolhead, extruder, flow, settle_time + measure_time,
            settle_time=settle_time, collect=True)
        steady = [state["force_control_g"] for state in samples
                  if state["motion_state"] == "EXTRUSION_STEADY"]
        if len(steady) < 2:
            # At very short calibration moves the callback rate and configured
            # transient time may leave no steady label; the explicit settle
            # window still makes the remaining samples suitable.
            steady = [state["force_control_g"] for state in samples]
        if len(steady) < 2:
            raise self.printer.command_error(
                "Insufficient force samples at flow %.3f" % (flow,))
        return {
            "mean_force": statistics.mean(steady),
            "force_sigma": statistics.pstdev(steady),
            "sample_count": len(steady),
        }

    def _flow_limits(self, points, profile, abort_force):
        limits = {}
        physical_limits = {}
        lower_bounds = set()
        for temperature in sorted(set(
                point["temperature"] for point in points)):
            curve = sorted((point for point in points
                            if point["temperature"] == temperature),
                           key=lambda point: point["flow"])
            knee = detect_knee(
                [(point["flow"], point["mean_force"]) for point in curve],
                self.minimum_knee_slope_ratio,
                self.minimum_knee_fit_improvement)
            candidates = []
            if knee is not None:
                candidates.append(knee["flow"] * profile.flow_safety_factor)
                physical_limits[temperature] = knee["flow"]
            else:
                physical_limits[temperature] = curve[-1]["flow"]
                lower_bounds.add(temperature)
            force_limit = abort_force * profile.flow_safety_factor
            for point in curve:
                if point["mean_force"] >= force_limit:
                    candidates.append(point["flow"])
                    break
            if candidates:
                limits[temperature] = min(candidates)
            else:
                limits[temperature] = (
                    curve[-1]["flow"] * profile.flow_safety_factor)
        return limits, physical_limits, lower_bounds

    def cmd_FORCE_FLOW_CALIBRATE(self, gcmd):
        monitor, profile, extruder_name, extruder = self._resolve(gcmd)
        owner = "FORCE_FLOW_CALIBRATE"
        monitor.claim_operation(owner)
        original_target = extruder.get_status(
            self.reactor.monotonic())["target"]
        monitor.add_client(self._handle_state)
        try:
            toolhead = self._check_prerequisites(
                gcmd, monitor, extruder_name)
            temperatures = self._parse_temperatures(gcmd, profile)
            flow_start = gcmd.get_float("FLOW_START", above=0.0)
            flow_step = gcmd.get_float("FLOW_STEP", above=0.0)
            flow_max = gcmd.get_float("FLOW_MAX", minval=flow_start)
            settle_time = gcmd.get_float("SETTLE_TIME", 1.0, above=0.0)
            measure_time = gcmd.get_float("MEASURE_TIME", 2.0, above=0.0)
            abort_force = gcmd.get_float("ABORT_FORCE", self.abort_force,
                                         above=0.0)
            if abort_force is None:
                raise gcmd.error(
                    "ABORT_FORCE or calibration abort_force must be specified")
            self._active_abort_force = abort_force
            self.safety_error = None
            points = []
            flow_count = int(math.floor(
                (flow_max - flow_start) / flow_step + 1e-9)) + 1
            flows = [flow_start + index * flow_step
                     for index in range(flow_count)]
            if flows[-1] < flow_max - 1e-9:
                flows.append(flow_max)
            for temperature in temperatures:
                gcmd.respond_info(
                    "Stabilizing %s at %.1fC"
                    % (extruder_name, temperature))
                self._set_temperature(extruder, temperature, wait=True)
                self._wait_baseline()
                if self.purge_length > 0.0:
                    purge_duration = self.purge_length / (
                        self.purge_flow / extruder.filament_area)
                    self._extrude(toolhead, extruder, self.purge_flow,
                                  purge_duration)
                    self._wait_baseline()
                for flow in flows:
                    result = self._measure_flow(
                        toolhead, extruder, flow, settle_time, measure_time)
                    point = {
                        "temperature": temperature,
                        "flow": flow,
                        "mean_force": result["mean_force"],
                        "force_sigma": result["force_sigma"],
                        "sample_count": result["sample_count"],
                    }
                    points.append(point)
                    gcmd.respond_info(
                        "T=%.1fC Q=%.3fmm^3/s F=%.1f+/-%.1fg (%d samples)"
                        % (temperature, flow, point["mean_force"],
                           point["force_sigma"], point["sample_count"]))
            limits, physical_limits, lower_bounds = self._flow_limits(
                points, profile, abort_force)
            profile.replace_calibration(
                points, limits,
                physical_flow_limits=physical_limits,
                physical_flow_lower_bounds=lower_bounds)
            gcmd.respond_info(
                "Force-flow profile '%s' updated; run SAVE_CONFIG after "
                "reviewing the results" % (profile.name,))
        finally:
            self.collection_window = None
            monitor.remove_client(self._handle_state)
            monitor.release_operation(owner)
            self._set_temperature(extruder, original_target, wait=False)

    def _response_segment(self, toolhead, extruder, flow, duration):
        samples = self._extrude(
            toolhead, extruder, flow, duration, collect=True)
        return [(state["print_time"], state["force_fast_g"])
                for state in samples]

    def cmd_FORCE_RESPONSE_CALIBRATE(self, gcmd):
        monitor, profile, extruder_name, extruder = self._resolve(gcmd)
        owner = "FORCE_RESPONSE_CALIBRATE"
        monitor.claim_operation(owner)
        original_target = extruder.get_status(
            self.reactor.monotonic())["target"]
        monitor.add_client(self._handle_state)
        try:
            toolhead = self._check_prerequisites(
                gcmd, monitor, extruder_name)
            temperature = gcmd.get_float("TEMPERATURE", above=0.0)
            flow_low = gcmd.get_float("FLOW_LOW", above=0.0)
            flow_high = gcmd.get_float("FLOW_HIGH", above=flow_low)
            duration = gcmd.get_float("DURATION", 3.0, above=0.0)
            abort_force = gcmd.get_float("ABORT_FORCE", self.abort_force,
                                         above=0.0)
            if abort_force is None:
                raise gcmd.error(
                    "ABORT_FORCE or calibration abort_force must be specified")
            self._active_abort_force = abort_force
            self.safety_error = None
            maximum = extruder.get_heater().max_temp
            if profile.max_material_temperature is not None:
                maximum = min(maximum, profile.max_material_temperature)
            if temperature >= maximum:
                raise gcmd.error("TEMPERATURE exceeds profile/heater limit")
            self._set_temperature(extruder, temperature, wait=True)
            self._wait_baseline()
            low_before = self._response_segment(
                toolhead, extruder, flow_low, duration)
            rise = self._response_segment(
                toolhead, extruder, flow_high, duration)
            fall = self._response_segment(
                toolhead, extruder, flow_low, duration)
            if not low_before or not rise or not fall:
                raise gcmd.error(
                    "Insufficient samples for response calibration")
            initial = statistics.mean(
                force for _, force in low_before[len(low_before) // 2:])
            high = statistics.mean(force for _, force in rise[len(rise) // 2:])
            final = statistics.mean(force for _, force in fall[len(fall) // 2:])
            tau_rise = estimate_response_tau(rise, initial, high, rising=True)
            tau_fall = estimate_response_tau(fall, high, final, rising=False)
            if tau_rise is None or tau_fall is None:
                raise gcmd.error("Unable to determine both response constants")
            profile.replace_calibration(
                profile.points, profile.flow_limits, tau_rise, tau_fall)
            gcmd.respond_info(
                "Profile '%s' response: rise_tau=%.4fs fall_tau=%.4fs; "
                "run SAVE_CONFIG after reviewing the results"
                % (profile.name, tau_rise, tau_fall))
        finally:
            self.collection_window = None
            monitor.remove_client(self._handle_state)
            monitor.release_operation(owner)
            self._set_temperature(extruder, original_target, wait=False)


def load_config(config):
    return ExtrusionForceCalibration(config)
