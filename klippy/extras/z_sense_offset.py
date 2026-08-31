# Correct first-layer Z offset from model-adjusted extrusion force
#
# Copyright (C) 2020 Martin Hierholzer <martin@hierholzer.info>
# Copyright (C) 2026 Timo Hilbig <gh@t-hilbig.de>
#
# Based on the implementation by Nibbles/Wessix on:
# https://github.com/RF1000community/Repetier-Firmware
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import statistics


class SensingZOffset:
    def __init__(self, config):
        self.name = config.get_name()
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        self.monitor_name = config.get(
            "monitor", "extrusion_force_monitor")
        self.force_threshold = config.getfloat(
            "force_threshold", None, minval=0.0)
        self.force_threshold_default = self.force_threshold
        self.minimum_force_margin = config.getfloat(
            "minimum_force_margin", 100.0, minval=0.0)
        self.z_force_slope = config.getfloat(
            "z_force_slope_g_per_mm", None, above=0.0)
        self.max_geometric_error = config.getfloat(
            "max_geometric_error", None, above=0.0)
        self.noise_factor = config.getfloat(
            "noise_factor", 6.0, above=0.0)
        self.relative_margin = config.getfloat(
            "relative_margin", 0.15, minval=0.0)
        self.minimum_confidence = config.getfloat(
            "minimum_confidence", 0.7, minval=0.0, maxval=1.0)
        self.max_z_offset = config.getfloat("max_z_offset", above=0.0)
        self.max_z_offset_default = self.max_z_offset
        self.max_z_height = config.getfloat(
            "max_z_height", 0.33, above=0.0)
        self.n_average_force = config.getint(
            "n_average_force", 10, minval=1)
        self.smoothing = config.getfloat(
            "smoothing", 0.5, minval=0.0, maxval=0.99)
        self.step_size = config.getfloat("step_size", 0.001, above=0.0)
        self.relative_tolerance = config.getfloat(
            "relative_tolerance", 0.1, above=0.0)
        self.acuteness = config.getfloat("acuteness", 24.0, above=0.0)
        self.max_steps_at_once = config.getfloat(
            "max_steps_at_once", 48.0, above=0.0)

        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "Z_SENSE_OFFSET", self.cmd_Z_SENSE_OFFSET,
            desc=self.cmd_Z_SENSE_OFFSET_help)
        self.gcode.register_command(
            "Z_FORCE_CALIBRATE", self.cmd_Z_FORCE_CALIBRATE,
            desc="Calibrate excess extrusion force over Z compression")

        self.z_offset = 0.0
        self.last_force = 0.0
        self.averaged_force = 0.0
        self.i_average = 0.0
        self.enable = False
        self.monitor = None
        self.load_cell = None
        self.dynamic_threshold = self.force_threshold
        self.mode = "legacy"
        self.calibration_window = None
        self.calibration_samples = []
        self.calibration_error = None

        # Each transform keeps the previous transform and forwards into it.
        # This makes load order with extrusion_force_control interchangeable.
        gcode_move = self.printer.load_object(config, "gcode_move")
        self.normal_transform = gcode_move.set_move_transform(self, force=True)
        self.printer.register_event_handler(
            "homing:home_rails_end", self._handle_home_rails_end)

    cmd_Z_SENSE_OFFSET_help = (
        "Increase Z offset from model-adjusted extrusion force")

    def cmd_Z_SENSE_OFFSET(self, gcmd):
        self.max_z_offset = gcmd.get_float(
            "MAX_Z_OFFSET", self.max_z_offset_default, minval=0.0)
        threshold = gcmd.get_float(
            "FORCE_THRESHOLD", self.force_threshold_default, minval=0.0)
        self.force_threshold = threshold
        self.z_offset = 0.0
        self.last_force = 0.0
        self.averaged_force = 0.0
        self.i_average = 0.0
        self.enable = True

    def _handle_ready(self):
        self.tool = self.printer.lookup_object("toolhead")
        self.monitor = self.printer.lookup_object(self.monitor_name, None)
        if self.monitor is not None:
            self.monitor.add_client(self._monitor_callback)
            self.mode = "model"
        else:
            self.load_cell = self.printer.lookup_object("load_cell", None)
            if self.load_cell is None or self.force_threshold is None:
                raise self.printer.config_error(
                    "z_sense_offset requires extrusion_force_monitor or "
                    "legacy force_threshold with a load_cell")
            self.load_cell.subscribe_force(self._legacy_callback)
            self.mode = "legacy"
        self.enable = False

    def _handle_home_rails_end(self, homing_state, rails):
        self.enable = False
        self.z_offset = 0.0

    def get_position(self):
        position = list(self.normal_transform.get_position())
        position[2] -= self.z_offset
        return position

    def move(self, newpos, speed):
        position = list(newpos)
        position[2] += self.z_offset
        self.normal_transform.move(position, speed)

    def _legacy_callback(self, force):
        if self.force_threshold is not None:
            self._process_force(abs(force), self.force_threshold)

    def _monitor_callback(self, state):
        if self.calibration_window is not None:
            start, end, abort_force = self.calibration_window
            if start <= state["print_time"] <= end:
                self.calibration_samples.append(state)
                if abs(state["force_fast_g"]) >= abort_force:
                    self.calibration_error = "ABORT_FORCE exceeded"
        if not self.enable:
            return
        if state["motion_state"] != "EXTRUSION_STEADY":
            return
        expected = state["expected_dynamic_force_g"]
        excess = state["excess_force_g"]
        if expected is None or excess is None:
            if self.force_threshold is None:
                return
            fallback_confidence = max(0.0, self.minimum_confidence - 0.4)
            if state["confidence"] < fallback_confidence:
                return
            self.mode = "legacy_fallback"
            self.dynamic_threshold = self.force_threshold
            self._process_force(abs(state["force_control_g"]),
                                self.force_threshold)
            return
        if state["confidence"] < self.minimum_confidence:
            return
        self.mode = "model"
        threshold = max(
            self.minimum_force_margin,
            state["noise_g"] * self.noise_factor,
            abs(expected) * self.relative_margin)
        self.dynamic_threshold = threshold
        self._process_force(max(0.0, excess), threshold)

    def _process_force(self, force, threshold):
        if not self.enable:
            return
        if self.tool.get_position()[2] > self.max_z_height:
            return
        if self.max_z_offset <= 0.0:
            return
        self.averaged_force += force
        self.i_average += 1.0
        if self.i_average < self.n_average_force:
            return
        smoothed_force = self.averaged_force / self.i_average
        self.averaged_force *= self.smoothing
        self.i_average *= self.smoothing
        if smoothed_force < threshold:
            self.last_force = smoothed_force
            return
        exceed_tolerance = (
            smoothed_force > threshold * (1.0 + self.relative_tolerance))
        steps = 1.0
        if exceed_tolerance and smoothed_force >= self.last_force:
            ratio = smoothed_force / threshold - 1.0
            steps += min(ratio * self.acuteness, self.max_steps_at_once)
            offset_ratio = self.z_offset / self.max_z_offset
            steps -= (steps - 1.0) * offset_ratio
        if not exceed_tolerance and smoothed_force < self.last_force:
            steps = 0.0
        self.z_offset = min(
            self.z_offset + steps * self.step_size, self.max_z_offset)
        self.last_force = smoothed_force
        logging.info(
            "z_sense_offset mode=%s force=%.1fg threshold=%.1fg "
            "steps=%.2f z_offset=%.6f",
            self.mode, smoothed_force, threshold, steps, self.z_offset)

    def _calibration_line(self, extruder, line_length, line_speed, flow,
                          y_step, abort_force):
        duration = line_length / line_speed
        position = self.tool.get_position()
        tool_status = self.tool.get_status(
            self.printer.get_reactor().monotonic())
        x_min = tool_status["axis_minimum"][0]
        x_max = tool_status["axis_maximum"][0]
        direction = 1.0
        if position[0] + line_length > x_max:
            direction = -1.0
        if position[0] - line_length < x_min and direction < 0.0:
            raise self.printer.command_error(
                "Not enough X travel for Z force calibration line")
        if y_step:
            position[1] += y_step
            y_min = tool_status["axis_minimum"][1]
            y_max = tool_status["axis_maximum"][1]
            if not y_min <= position[1] <= y_max:
                raise self.printer.command_error(
                    "Not enough Y travel for Z force calibration lines")
            self.tool.manual_move(position, line_speed)
            self.tool.wait_moves()
            position = self.tool.get_position()
        position[0] += direction * line_length
        y_min = tool_status["axis_minimum"][1]
        y_max = tool_status["axis_maximum"][1]
        if not y_min <= position[1] <= y_max:
            raise self.printer.command_error(
                "Not enough Y travel for Z force calibration lines")
        position[3] += flow * duration / extruder.filament_area
        start_time = self.tool.get_last_move_time()
        self.calibration_samples = []
        self.calibration_error = None
        self.tool.manual_move(position, line_speed)
        end_time = self.tool.get_last_move_time()
        self.calibration_window = (start_time, end_time, abort_force)
        self.tool.wait_moves()
        self.calibration_window = None
        if self.calibration_error is not None:
            raise self.printer.command_error(self.calibration_error)
        values = [state["excess_force_g"] for state in self.calibration_samples
                  if state["motion_state"] == "EXTRUSION_STEADY"
                  and state["excess_force_g"] is not None]
        if len(values) < 2:
            values = [state["excess_force_g"]
                      for state in self.calibration_samples
                      if state["excess_force_g"] is not None]
        if len(values) < 2:
            raise self.printer.command_error(
                "Insufficient model-valid samples on calibration line")
        return statistics.mean(values)

    def cmd_Z_FORCE_CALIBRATE(self, gcmd):
        if self.monitor is None:
            raise gcmd.error(
                "Z_FORCE_CALIBRATE requires extrusion_force_monitor")
        state = self.monitor.get_latest_state()
        extruder_name = gcmd.get(
            "EXTRUDER", state["extruder"] if state is not None else "extruder")
        profile = self.monitor.get_active_profile(extruder_name)
        if profile is None:
            raise gcmd.error(
                "Z_FORCE_CALIBRATE requires an active force profile")
        extruder = self.printer.lookup_object(extruder_name)
        line_length = gcmd.get_float("LINE_LENGTH", above=0.0)
        line_speed = gcmd.get_float("LINE_SPEED", above=0.0)
        line_spacing = gcmd.get_float("LINE_SPACING", 2.0, above=0.0)
        flow = gcmd.get_float("FLOW", above=0.0)
        z_step = gcmd.get_float("Z_STEP", above=0.0)
        steps = gcmd.get_int("STEPS", minval=3)
        abort_force = gcmd.get_float("ABORT_FORCE", above=0.0)
        reference_margin = gcmd.get_float(
            "REFERENCE_MARGIN", self.minimum_force_margin, minval=0.0)
        max_error = gcmd.get_float(
            "MAX_GEOMETRIC_ERROR", self.max_geometric_error, above=0.0)
        if max_error is None:
            raise gcmd.error(
                "MAX_GEOMETRIC_ERROR or max_geometric_error must be specified")
        kin_status = self.tool.get_kinematics().get_status(
            self.printer.get_reactor().monotonic())
        if not all(axis in kin_status.get("homed_axes", "") for axis in "xyz"):
            raise gcmd.error("XYZ must be homed before Z_FORCE_CALIBRATE")
        if not extruder.get_heater().can_extrude:
            raise gcmd.error("Extruder is not hot enough for calibration")
        owner = "Z_FORCE_CALIBRATE"
        self.monitor.claim_operation(owner)
        start_z = self.tool.get_position()[2]
        try:
            points = []
            for index in range(steps):
                excess = self._calibration_line(
                    extruder, line_length, line_speed, flow,
                    0.0 if index == 0 else line_spacing,
                    abort_force)
                compression = index * z_step
                points.append((compression, excess))
                gcmd.respond_info(
                    "Z compression %.4fmm: excess force %.1fg"
                    % (compression, excess))
                if index + 1 < steps:
                    position = self.tool.get_position()
                    position[2] -= z_step
                    self.tool.manual_move(position, 2.0)
                    self.tool.wait_moves()
            mean_x = statistics.mean(point[0] for point in points)
            mean_y = statistics.mean(point[1] for point in points)
            variance = sum((point[0] - mean_x) ** 2 for point in points)
            slope = sum((point[0] - mean_x) * (point[1] - mean_y)
                        for point in points) / variance
            if slope <= 0.0:
                raise gcmd.error(
                    "Z force curve did not produce a positive slope")
            self.z_force_slope = slope
            self.max_geometric_error = max_error
            self.minimum_force_margin = max(
                reference_margin, slope * max_error)
            configfile = self.printer.lookup_object("configfile")
            configfile.set(self.name, "z_force_slope_g_per_mm", "%.6f" % slope)
            configfile.set(self.name, "max_geometric_error", "%.6f" % max_error)
            configfile.set(self.name, "minimum_force_margin",
                           "%.3f" % self.minimum_force_margin)
            gcmd.respond_info(
                "Z force slope %.1fg/mm; minimum margin %.1fg. Run "
                "SAVE_CONFIG after validating the first layer."
                % (slope, self.minimum_force_margin))
        finally:
            self.calibration_window = None
            self.monitor.release_operation(owner)
            position = self.tool.get_position()
            if position[2] < start_z:
                position[2] = start_z
                self.tool.manual_move(position, 2.0)
                self.tool.wait_moves()

    def get_status(self, eventtime):
        return {
            "enabled": self.enable,
            "mode": self.mode,
            "z_offset": self.z_offset,
            "dynamic_threshold_g": self.dynamic_threshold,
            "last_force_g": self.last_force,
            "z_force_slope_g_per_mm": self.z_force_slope,
        }


def load_config(config):
    return SensingZOffset(config)
