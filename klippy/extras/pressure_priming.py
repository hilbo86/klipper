# Prime an extruder until a stable, configured force is reached
#
# Copyright (C) 2025-2026 Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import statistics
import math


class PressurePriming:
    def __init__(self, config):
        self.name = config.get_name()
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.force_threshold = config.getfloat(
            "force_threshold", above=0.0)
        self.force_threshold_default = self.force_threshold
        self.max_prime_length = config.getfloat(
            "max_prime_length", minval=1.0)
        self.max_prime_length_default = self.max_prime_length
        self.force_safety_limit = config.getfloat(
            "force_safety_limit", 8000.0, above=0.0)
        self.baseline_samples = config.getint(
            "baseline_samples", 10, minval=2)
        self.sample_timeout = config.getfloat(
            "sample_timeout", 2.0, above=0.0)
        self.monitor_name = config.get(
            "monitor", "extrusion_force_monitor")
        self.load_cell = self.printer.lookup_object("load_cell")
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "PRESSURE_PRIME", self.cmd_PRESSURE_PRIME,
            desc="Prime extruder until force rises above a stable threshold")
        self.tool = None
        self.monitor = None
        self.baseline_values = []
        self.baseline_force = None
        self.sample_window = None
        self.window_values = []
        self.overpressure = False

    def _handle_ready(self):
        self.tool = self.printer.lookup_object("toolhead")
        self.monitor = self.printer.lookup_object(self.monitor_name, None)

    def _sample_callback(self, sample):
        absolute_force = sample["absolute_force_g"]
        if self.baseline_force is None:
            self.baseline_values.append(absolute_force)
            return
        force = absolute_force - self.baseline_force
        if abs(force) >= self.force_safety_limit:
            self.overpressure = True
        if self.sample_window is not None:
            start_time, end_time = self.sample_window
            if start_time <= sample["print_time"] <= end_time:
                self.window_values.append(force)

    def _resolve_extruder(self, gcmd):
        value = gcmd.get("EXTRUDER", None)
        if value is None:
            extruder = self.tool.get_extruder()
            return extruder.get_name(), extruder
        value = str(value).strip().lower()
        if value in ("0", "e0", "t0"):
            value = "extruder"
        elif value in ("1", "e1", "t1"):
            value = "extruder1"
        extruder = self.printer.lookup_object(value, None)
        if extruder is None:
            raise gcmd.error("Unknown extruder '%s'" % (value,))
        return value, extruder

    def _capture_baseline(self, gcmd):
        self.baseline_values = []
        self.baseline_force = None
        deadline = self.reactor.monotonic() + self.sample_timeout
        while len(self.baseline_values) < self.baseline_samples:
            if self.reactor.monotonic() >= deadline:
                raise gcmd.error("Timeout collecting pressure-prime baseline")
            self.reactor.pause(self.reactor.monotonic() + 0.02)
        self.baseline_force = statistics.mean(
            self.baseline_values[-self.baseline_samples:])

    def _extrude_segment(self, gcmd, speed):
        start_time = self.tool.get_last_move_time()
        position = self.tool.get_position()
        position[3] += 1.0
        self.window_values = []
        self.sample_window = None
        self.tool.manual_move(position, speed)
        end_time = self.tool.get_last_move_time()
        self.sample_window = (start_time, end_time)
        self.tool.wait_moves()
        self.sample_window = None
        if self.overpressure:
            raise gcmd.error(
                "Pressure-prime force safety limit exceeded")
        if len(self.window_values) < 3:
            raise gcmd.error(
                "Insufficient timestamped load-cell samples during extrusion")
        cutoff = min(len(self.window_values) // 6,
                     (len(self.window_values) - 1) // 2)
        values = (self.window_values[cutoff:-cutoff]
                  if cutoff else self.window_values)
        return statistics.mean(values)

    def cmd_PRESSURE_PRIME(self, gcmd):
        extruder_name, extruder = self._resolve_extruder(gcmd)
        target_temp = gcmd.get_float(
            "TARGET_TEMP", 210.0,
            minval=extruder.heater.min_extrude_temp,
            below=extruder.heater.max_temp)
        threshold = gcmd.get_float(
            "THRESHOLD", self.force_threshold_default, above=0.0,
            maxval=self.force_safety_limit)
        self.force_safety_limit = gcmd.get_float(
            "LIMIT", self.force_safety_limit, above=threshold)
        if self.force_safety_limit <= threshold:
            raise gcmd.error("LIMIT must be greater than THRESHOLD")
        maximum_length = gcmd.get_float(
            "LENGTH", self.max_prime_length_default, minval=1.0,
            maxval=100.0)
        speed = gcmd.get_float(
            "SPEED", 120.0, minval=30.0, maxval=900.0) / 60.0
        maximum_duration = maximum_length / min(
            extruder.max_e_velocity, speed) * 2.0
        owner = "PRESSURE_PRIME"
        if self.monitor is not None:
            self.monitor.claim_operation(owner)
        original_target = extruder.get_status(
            self.reactor.monotonic())["target"]
        self.load_cell.add_client(self._sample_callback)
        self.overpressure = False
        try:
            status = self.load_cell.get_status(self.reactor.monotonic())
            if not status.get("is_calibrated", False):
                raise gcmd.error("Load cell must be calibrated in grams")
            if self.tool.get_extruder().get_name() != extruder_name:
                self.gcode.run_script_from_command(
                    "ACTIVATE_EXTRUDER EXTRUDER=%s" % (extruder_name,))
            heaters = self.printer.lookup_object("heaters")
            heaters.set_temperature(extruder.get_heater(), target_temp, True)
            self._capture_baseline(gcmd)
            deadline = self.reactor.monotonic() + maximum_duration
            forces = []
            stable_hits = 0
            for segment in range(int(math.ceil(maximum_length))):
                if self.reactor.monotonic() >= deadline:
                    raise gcmd.error("Pressure priming timed out")
                force = self._extrude_segment(gcmd, speed)
                forces.append(force)
                delta_ratio = (abs(force - forces[-2]) / max(abs(force), 1.0)
                               if len(forces) > 1 else float("inf"))
                if force >= threshold and delta_ratio < 0.15:
                    stable_hits += 1
                else:
                    stable_hits = 0
                gcmd.respond_info(
                    "Pressure prime %dmm: %.1fg" % (segment + 1, force))
                if stable_hits >= 1:
                    gcmd.respond_info(
                        "Pressure priming successful after %dmm at %.1fg"
                        % (segment + 1, force))
                    return
            raise gcmd.error("Maximum pressure-prime length reached")
        finally:
            self.sample_window = None
            self.load_cell.remove_client(self._sample_callback)
            if self.monitor is not None:
                self.monitor.release_operation(owner)
            self.printer.lookup_object("heaters").set_temperature(
                extruder.get_heater(), original_target, False)
            self.baseline_force = None
            self.baseline_values = []


def load_config(config):
    return PressurePriming(config)
