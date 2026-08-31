# Reference diagnostics and experimental extrusion-force analyses
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import statistics

from .extrusion_force_calibration import estimate_response_tau


def force_response_metrics(samples, initial_force, target_force,
                           tolerance_ratio=0.05):
    if len(samples) < 3 or target_force == initial_force:
        return None
    rising = target_force > initial_force
    tau = estimate_response_tau(
        samples, initial_force, target_force, rising=rising)
    forces = [force for _, force in samples]
    overshoot = ((max(forces) - target_force) if rising
                 else (target_force - min(forces)))
    tolerance = abs(target_force - initial_force) * tolerance_ratio
    settling_time = None
    for index, (sample_time, force) in enumerate(samples):
        if abs(force - target_force) > tolerance:
            continue
        if all(abs(later_force - target_force) <= tolerance
               for _, later_force in samples[index:]):
            settling_time = sample_time - samples[0][0]
            break
    mean_deviation = statistics.mean(
        abs(force - target_force) for _, force in samples)
    return {
        "tau": tau,
        "overshoot_g": max(0.0, overshoot),
        "settling_time": settling_time,
        "mean_deviation_g": mean_deviation,
    }


class CollisionDetector:
    def __init__(self, impulse_threshold, derivative_threshold,
                 minimum_xy_velocity, maximum_flow, cooldown):
        self.impulse_threshold = impulse_threshold
        self.derivative_threshold = derivative_threshold
        self.minimum_xy_velocity = minimum_xy_velocity
        self.maximum_flow = maximum_flow
        self.cooldown = cooldown
        self.last_detection = None

    def update(self, state):
        if state["motion_state"] != "TRAVEL":
            return False
        if state["flow_mm3_s"] > self.maximum_flow:
            return False
        if state.get("xy_velocity", 0.0) < self.minimum_xy_velocity:
            return False
        impulse = abs(state["force_fast_g"] - state["force_trend_g"])
        if (impulse < self.impulse_threshold
                or abs(state["dforce_dt"]) < self.derivative_threshold):
            return False
        if (self.last_detection is not None
                and state["print_time"] - self.last_detection < self.cooldown):
            return False
        self.last_detection = state["print_time"]
        return True


class ExtrusionForceDiagnostics:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.monitor_name = config.get(
            "monitor", "extrusion_force_monitor")
        self.minimum_z_height = config.getfloat(
            "minimum_z_height", 5.0, minval=0.0)
        self.settle_time = config.getfloat(
            "settle_time", 1.0, minval=0.0)
        self.measure_time = config.getfloat(
            "measure_time", 2.0, above=0.0)
        self.collision_enabled = config.getboolean(
            "collision_detection", False)
        impulse = config.getfloat(
            "collision_impulse_threshold", None, above=0.0)
        derivative = config.getfloat(
            "collision_derivative_threshold", None, above=0.0)
        if self.collision_enabled and (impulse is None or derivative is None):
            raise config.error(
                "Collision thresholds are required when collision_detection "
                "is enabled")
        self.collision_detector = (
            CollisionDetector(
                impulse, derivative,
                config.getfloat("collision_min_xy_velocity", 1.0, minval=0.0),
                config.getfloat("collision_max_flow", 0.05, minval=0.0),
                config.getfloat("collision_cooldown", 0.5, minval=0.0))
            if self.collision_enabled else None)
        self.monitor = None
        self.collection_window = None
        self.collected_states = []
        self.abort_force = float("inf")
        self.collection_error = None
        self.last_diagnostic = None
        self.last_pa_analysis = None
        self.collision_count = 0
        self.last_collision = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_command(
            "EXTRUSION_FORCE_DIAGNOSTIC", self.cmd_DIAGNOSTIC,
            desc="Run a reference extrusion-force health test")
        self.gcode.register_command(
            "EXTRUSION_FORCE_PA_ANALYZE", self.cmd_PA_ANALYZE,
            desc="Experimentally compare pressure-advance force responses")

    def _handle_ready(self):
        self.monitor = self.printer.lookup_object(self.monitor_name)
        self.monitor.add_client(self._handle_state)

    def _handle_state(self, state):
        if (self.collision_detector is not None
                and self.collision_detector.update(state)):
            self.collision_count += 1
            self.last_collision = state["print_time"]
            logging.warning(
                "Possible nozzle collision at print_time %.6f: force impulse "
                "%.1fg (log-only experimental detector)",
                state["print_time"],
                abs(state["force_fast_g"] - state["force_trend_g"]))
            self.printer.send_event(
                "extrusion_force:possible_collision", state)
        if self.collection_window is None:
            return
        start, end = self.collection_window
        if start <= state["print_time"] <= end:
            self.collected_states.append(state)
            if abs(state["force_fast_g"]) >= self.abort_force:
                self.collection_error = "ABORT_FORCE exceeded"

    def _resolve(self, gcmd):
        profile_name = gcmd.get("PROFILE", None)
        manager = self.printer.lookup_object(
            "extrusion_force_profile_manager", None)
        if manager is None:
            raise gcmd.error("No extrusion force profiles are configured")
        if profile_name is None:
            extruder_name = gcmd.get("EXTRUDER", "extruder")
            profile = manager.get_active(extruder_name)
        else:
            profile = manager.get_profile(profile_name)
            extruder_name = gcmd.get(
                "EXTRUDER", profile.extruder if profile is not None else "")
        if profile is None:
            raise gcmd.error("No matching extrusion force profile")
        if profile.extruder != extruder_name:
            raise gcmd.error(
                "Profile '%s' belongs to '%s'"
                % (profile.name, profile.extruder))
        extruder = self.printer.lookup_object(extruder_name)
        return profile, extruder_name, extruder

    def _check_motion(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        kin_status = toolhead.get_kinematics().get_status(
            self.reactor.monotonic())
        if "z" not in kin_status.get("homed_axes", ""):
            raise gcmd.error("Z must be homed before force diagnostics")
        if toolhead.get_position()[2] < self.minimum_z_height:
            raise gcmd.error(
                "Move nozzle to at least %.3fmm Z" % self.minimum_z_height)
        return toolhead

    def _set_temperature(self, extruder, temperature, wait):
        self.printer.lookup_object("heaters").set_temperature(
            extruder.get_heater(), temperature, wait)

    def _collect_extrusion(self, toolhead, extruder, flow, duration,
                           discard_time=0.0):
        velocity = flow / extruder.filament_area
        start = toolhead.get_last_move_time()
        position = toolhead.get_position()
        position[3] += velocity * duration
        self.collected_states = []
        self.collection_error = None
        toolhead.manual_move(position, velocity)
        end = toolhead.get_last_move_time()
        self.collection_window = (start + discard_time, end)
        toolhead.wait_moves()
        self.collection_window = None
        if self.collection_error is not None:
            raise self.printer.command_error(self.collection_error)
        return list(self.collected_states)

    def cmd_DIAGNOSTIC(self, gcmd):
        profile, extruder_name, extruder = self._resolve(gcmd)
        toolhead = self._check_motion(gcmd)
        temperature = gcmd.get_float(
            "TEMPERATURE", minval=extruder.heater.min_extrude_temp,
            below=extruder.heater.max_temp)
        if (profile.max_material_temperature is not None
                and temperature > profile.max_material_temperature):
            raise gcmd.error("TEMPERATURE exceeds profile material limit")
        flow = gcmd.get_float("FLOW", above=0.0)
        length = gcmd.get_float("LENGTH", above=0.0)
        self.abort_force = gcmd.get_float("ABORT_FORCE", above=0.0)
        expected = profile.expected_force(flow, temperature)
        if expected is None:
            raise gcmd.error("Diagnostic point is outside calibrated profile")
        velocity = flow / extruder.filament_area
        duration = length / velocity
        owner = "EXTRUSION_FORCE_DIAGNOSTIC"
        self.monitor.claim_operation(owner)
        original_target = extruder.get_status(
            self.reactor.monotonic())["target"]
        try:
            self.gcode.run_script_from_command(
                "ACTIVATE_EXTRUDER EXTRUDER=%s" % extruder_name)
            self._set_temperature(extruder, temperature, True)
            self.reactor.pause(self.reactor.monotonic() + 1.0)
            states = self._collect_extrusion(
                toolhead, extruder, flow, duration, self.settle_time)
            values = [state["force_control_g"] for state in states
                      if state["motion_state"] == "EXTRUSION_STEADY"]
            if len(values) < 2:
                values = [state["force_control_g"] for state in states]
            if len(values) < 2:
                raise gcmd.error("Insufficient diagnostic force samples")
            measured = statistics.mean(values)
            deviation = (measured - expected) / expected * 100.0
            self.last_diagnostic = {
                "profile": profile.name,
                "temperature": temperature,
                "flow_mm3_s": flow,
                "reference_force_g": expected,
                "measured_force_g": measured,
                "deviation_percent": deviation,
            }
            gcmd.respond_info(
                "Reference: %.1fg\nMeasured: %.1fg\nDeviation: %+.1f%%"
                % (expected, measured, deviation))
        finally:
            self.collection_window = None
            self.monitor.release_operation(owner)
            self._set_temperature(extruder, original_target, False)

    def _parse_pa_values(self, gcmd):
        try:
            values = [float(item.strip())
                      for item in gcmd.get("PA_VALUES").split(",")]
        except ValueError:
            raise gcmd.error("Invalid PA_VALUES")
        if len(values) < 2 or any(value < 0.0 for value in values):
            raise gcmd.error(
                "PA_VALUES requires at least two non-negative values")
        return sorted(set(values))

    def cmd_PA_ANALYZE(self, gcmd):
        profile, extruder_name, extruder = self._resolve(gcmd)
        toolhead = self._check_motion(gcmd)
        pa_values = self._parse_pa_values(gcmd)
        temperature = gcmd.get_float(
            "TEMPERATURE", minval=extruder.heater.min_extrude_temp,
            below=extruder.heater.max_temp)
        flow_low = gcmd.get_float("FLOW_LOW", above=0.0)
        flow_high = gcmd.get_float("FLOW_HIGH", above=flow_low)
        duration = gcmd.get_float("SEGMENT_TIME", 2.0, above=0.0)
        self.abort_force = gcmd.get_float("ABORT_FORCE", above=0.0)
        expected_low = profile.expected_force(flow_low, temperature)
        expected_high = profile.expected_force(flow_high, temperature)
        if expected_low is None or expected_high is None:
            raise gcmd.error("PA test flows are outside the calibrated profile")
        status = extruder.get_status(self.reactor.monotonic())
        original_pa = status.get("pressure_advance", 0.0)
        original_target = status["target"]
        owner = "EXTRUSION_FORCE_PA_ANALYZE"
        self.monitor.claim_operation(owner)
        try:
            self.gcode.run_script_from_command(
                "ACTIVATE_EXTRUDER EXTRUDER=%s" % extruder_name)
            self._set_temperature(extruder, temperature, True)
            results = []
            for pa in pa_values:
                self.gcode.run_script_from_command(
                    "SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.6f"
                    % (extruder_name, pa))
                self._collect_extrusion(
                    toolhead, extruder, flow_low, duration)
                rise_states = self._collect_extrusion(
                    toolhead, extruder, flow_high, duration)
                fall_states = self._collect_extrusion(
                    toolhead, extruder, flow_low, duration)
                rise = [(state["print_time"], state["force_fast_g"])
                        for state in rise_states]
                fall = [(state["print_time"], state["force_fast_g"])
                        for state in fall_states]
                rise_metrics = force_response_metrics(
                    rise, expected_low, expected_high)
                fall_metrics = force_response_metrics(
                    fall, expected_high, expected_low)
                if rise_metrics is None or fall_metrics is None:
                    raise gcmd.error("Insufficient samples for PA analysis")
                score = (
                    rise_metrics["mean_deviation_g"]
                    + fall_metrics["mean_deviation_g"]
                    + rise_metrics["overshoot_g"]
                    + fall_metrics["overshoot_g"])
                result = {"pressure_advance": pa, "score": score,
                          "rise": rise_metrics, "fall": fall_metrics}
                results.append(result)
                gcmd.respond_info(
                    "PA %.5f: rise_tau=%s fall_tau=%s overshoot=%.1fg "
                    "score=%.1f"
                    % (pa,
                       ("%.3fs" % rise_metrics["tau"]
                        if rise_metrics["tau"] is not None else "n/a"),
                       ("%.3fs" % fall_metrics["tau"]
                        if fall_metrics["tau"] is not None else "n/a"),
                       rise_metrics["overshoot_g"]
                       + fall_metrics["overshoot_g"], score))
            best = min(
                range(len(results)),
                key=lambda index: results[index]["score"])
            lower = results[max(0, best - 1)]["pressure_advance"]
            upper = results[min(len(results) - 1, best + 1)]["pressure_advance"]
            self.last_pa_analysis = {
                "profile": profile.name,
                "results": results,
                "recommended_test_range": [lower, upper],
            }
            gcmd.respond_info(
                "Recommended visual PA test range: %.5f..%.5f (force analysis "
                "does not replace print-quality validation)" % (lower, upper))
        finally:
            self.collection_window = None
            self.monitor.release_operation(owner)
            try:
                self.gcode.run_script_from_command(
                    "SET_PRESSURE_ADVANCE EXTRUDER=%s ADVANCE=%.6f"
                    % (extruder_name, original_pa))
            finally:
                self._set_temperature(extruder, original_target, False)

    def get_status(self, eventtime):
        return {
            "last_diagnostic": self.last_diagnostic,
            "collision_detection": self.collision_enabled,
            "collision_count": self.collision_count,
            "last_collision_time": self.last_collision,
            "last_pa_analysis": self.last_pa_analysis,
        }


def load_config(config):
    return ExtrusionForceDiagnostics(config)
