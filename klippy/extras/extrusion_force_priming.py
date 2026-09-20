# Prime an extruder until stable force or low-force material flow is detected
#
# Copyright (C) 2025-2026 Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import collections
import logging
import math
import statistics

from .extrusion_force_motion import MotionForceSampler


class PrimingThermalEvidence:
    def __init__(self, config):
        self.reactor = config.get_printer().get_reactor()
        self.baseline_time = config.getfloat(
            "thermal_baseline_time", 5.0, above=0.0)
        self.settle_timeout = config.getfloat(
            "thermal_settle_timeout", 30.0, above=self.baseline_time)
        self.confirm_time = config.getfloat(
            "thermal_confirm_time", 2.0, above=0.0)
        self.power_delta = config.getfloat(
            "thermal_power_delta", 0.02, above=0.0, maxval=1.0)
        self.temperature_tolerance = config.getfloat(
            "thermal_temperature_tolerance", 2.0, above=0.0)
        self.timer = None
        self.samples = collections.deque()
        self.baseline_power = None

    def _read(self, eventtime):
        status = self.heater.get_status(eventtime)
        return (eventtime, status["temperature"], status["power"],
                status["target"])

    def _temperature_stable(self, samples):
        return all(abs(temp - self.target) <= self.temperature_tolerance
                   and abs(target - self.target) < 1.0e-6
                   for _, temp, _, target in samples)

    def capture_baseline(self, gcmd, heater, target):
        self.heater, self.target = heater, target
        self.baseline_power = None
        self.samples.clear()
        deadline = self.reactor.monotonic() + self.settle_timeout
        while self.reactor.monotonic() < deadline:
            now = self.reactor.monotonic()
            self.samples.append(self._read(now))
            while (len(self.samples) > 1
                   and self.samples[1][0] <= now - self.baseline_time):
                self.samples.popleft()
            if (len(self.samples) >= 3
                    and now - self.samples[0][0] >= self.baseline_time
                    and self._temperature_stable(self.samples)):
                temperatures = [row[1] for row in self.samples]
                # M109 may return during the tail of a heating transient.
                # Require a quiet idle interval before comparing heater load.
                if (max(temperatures) - min(temperatures)
                        <= self.temperature_tolerance * 0.5
                        and abs(temperatures[-1] - temperatures[0])
                        <= self.temperature_tolerance * 0.25):
                    powers = [row[2] for row in self.samples]
                    midpoint = len(powers) // 2
                    power_drift = abs(statistics.mean(powers[:midpoint])
                                      - statistics.mean(powers[midpoint:]))
                    if power_drift <= self.power_delta:
                        self.baseline_power = statistics.mean(powers)
                        self.required_delta = max(
                            self.power_delta, 3.0 * statistics.pstdev(powers))
                        break
            self.reactor.pause(now + 0.1)
        self.samples.clear()
        if self.baseline_power is None:
            gcmd.respond_info(
                "No stable idle heater baseline; priming requires target force")
        else:
            gcmd.respond_info(
                "Prime idle heater power %.1f%%; required increase %.1f%%"
                % (100.0 * self.baseline_power, 100.0 * self.required_delta))

    def start(self):
        if self.baseline_power is not None:
            self.timer = self.reactor.register_timer(
                self._sample, self.reactor.monotonic())

    def _sample(self, eventtime):
        self.samples.append(self._read(eventtime))
        while (len(self.samples) > 1
               and self.samples[1][0] <= eventtime - self.confirm_time):
            self.samples.popleft()
        return eventtime + 0.1

    def confirmed(self):
        if self.baseline_power is None or len(self.samples) < 3:
            return False
        if (self.samples[-1][0] - self.samples[0][0] < self.confirm_time
                or not self._temperature_stable(self.samples)):
            return False
        # Include the short force-measurement pauses: the heater's response
        # lags filament feed. Compare a sustained mean with the long idle mean,
        # not one PWM peak or the residual power from heating to TARGET_TEMP.
        powers = [row[2] for row in self.samples]
        midpoint = len(powers) // 2
        required = self.baseline_power + self.required_delta
        return (statistics.mean(powers[:midpoint]) >= required
                and statistics.mean(powers[midpoint:]) >= required)

    def stop(self):
        if self.timer is not None:
            self.reactor.unregister_timer(self.timer)
            self.timer = None
        self.samples.clear()


def format_pressure_summary(forces, success, failure_reason=None, length=None):
    status = "SUCCESS" if success else "FAILURE"
    lines = [
        "Pressure priming summary: %s after %gmm"
        % (status, len(forces) if length is None else length)]
    previous = 0.0
    for index, force in enumerate(forces):
        delta = force - previous if index else force
        lines.append(
            "Nr %3d: F_mean=%7.1fg; F_delta=%+7.1fg"
            % (index + 1, force, delta))
        previous = force
    if failure_reason:
        lines.append("Reason: %s" % (failure_reason,))
    return "\n".join(lines)


class ExtrusionForcePriming:
    def __init__(self, config):
        self.name = config.get_name()
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.motion = MotionForceSampler(config)
        self.thermal = PrimingThermalEvidence(config)
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
            desc="Prime until stable force or material flow is detected")
        self.tool = None
        self.monitor = None
        self.baseline_values = []
        self.baseline_force = None
        self.moving_samples = []
        self.overpressure = False
        self.active_force_limit = None

    def _handle_ready(self):
        self.tool = self.printer.lookup_object("toolhead")
        self.monitor = self.printer.lookup_object(self.monitor_name, None)

    def _sample_callback(self, sample):
        self.motion.add_sample(sample)
        absolute_force = sample["absolute_force_g"]
        if self.baseline_force is None:
            self.baseline_values.append(absolute_force)
            return
        force = absolute_force - self.baseline_force
        force_limit = (self.active_force_limit
                       if self.active_force_limit is not None
                       else self.force_safety_limit)
        if abs(force) >= force_limit:
            self.overpressure = True

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

    def _extrude_segment(self, gcmd, speed, length=1.0):
        start_time = self.tool.get_last_move_time()
        position = self.tool.get_position()
        position[3] += length
        self.tool.manual_move(
            position, min(speed, length / self.motion.sample_time))
        end_time = self.tool.get_last_move_time()
        self.tool.wait_moves()
        self.moving_samples = self.motion.collect(gcmd, start_time, end_time)
        window_values = [value - self.baseline_force
                         for value in self.moving_samples]
        if self.overpressure:
            raise gcmd.error(
                "Pressure-prime force safety limit exceeded")
        if len(window_values) < 3:
            raise gcmd.error(
                "Insufficient timestamped load-cell samples during extrusion")
        cutoff = min(len(window_values) // 6, (len(window_values) - 1) // 2)
        values = window_values[cutoff:-cutoff] if cutoff else window_values
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
        force_limit = gcmd.get_float(
            "LIMIT", self.force_safety_limit, above=threshold,
            maxval=self.force_safety_limit)
        maximum_length = gcmd.get_float(
            "LENGTH", self.max_prime_length_default, minval=1.0,
            maxval=100.0)
        speed = gcmd.get_float(
            "SPEED", 120.0, minval=30.0, maxval=900.0) / 60.0
        speed = min(extruder.max_e_velocity, speed,
                    1.0 / self.motion.sample_time)
        maximum_duration = (maximum_length / speed * 2.0
                            + math.ceil(maximum_length) * 2.0
                            * (self.motion.settle_time + self.motion.sample_time
                               + self.motion.timeout))
        owner = "PRESSURE_PRIME"
        original_target = None
        operation_claimed = False
        client_added = False
        forces = []
        extruded = 0.0
        success = False
        failure_reason = None
        self.overpressure = False
        self.active_force_limit = force_limit
        self.motion.reset()
        try:
            if self.monitor is not None:
                self.monitor.claim_operation(owner)
                operation_claimed = True
            original_target = extruder.get_status(
                self.reactor.monotonic())["target"]
            self.load_cell.add_client(self._sample_callback)
            client_added = True
            status = self.load_cell.get_status(self.reactor.monotonic())
            if not status.get("is_calibrated", False):
                raise gcmd.error("Load cell must be calibrated in grams")
            if self.tool.get_extruder().get_name() != extruder_name:
                self.gcode.run_script_from_command(
                    "ACTIVATE_EXTRUDER EXTRUDER=%s" % (extruder_name,))
            heaters = self.printer.lookup_object("heaters")
            heaters.set_temperature(extruder.get_heater(), target_temp, True)
            self.tool.wait_moves()
            self.thermal.capture_baseline(
                gcmd, extruder.get_heater(), target_temp)
            self._capture_baseline(gcmd)
            self.thermal.start()
            deadline = self.reactor.monotonic() + maximum_duration
            previous_above_threshold = False
            for segment in range(int(math.ceil(maximum_length))):
                if self.reactor.monotonic() >= deadline:
                    raise gcmd.error("Pressure priming timed out")
                before = self.motion.idle(gcmd, self.tool)
                if self.overpressure:
                    raise gcmd.error(
                        "Pressure-prime force safety limit exceeded")
                length = min(1.0, maximum_length - extruded)
                force = self._extrude_segment(gcmd, speed, length)
                extruded += length
                forces.append(force)
                delta_ratio = (abs(force - forces[-2]) / max(abs(force), 1.0)
                               if len(forces) > 1 else float("inf"))
                above_threshold = force >= threshold
                stable_pair = (above_threshold
                               and previous_above_threshold
                               and delta_ratio < 0.15)
                previous_above_threshold = above_threshold
                low_force_flow = False
                if not above_threshold:
                    after = self.motion.idle(gcmd, self.tool)
                    low_force_flow = self.motion.check(
                        self.moving_samples, before, after,
                        self.baseline_force, 1.0, threshold)
                    low_force_flow = low_force_flow and self.thermal.confirmed()
                else:
                    self.motion.matches = 0
                if self.overpressure:
                    raise gcmd.error(
                        "Pressure-prime force safety limit exceeded")
                gcmd.respond_info(
                    "Pressure prime %gmm: %.1fg" % (extruded, force))
                if stable_pair or low_force_flow:
                    success = True
                    if low_force_flow:
                        gcmd.respond_info(
                            "Filament flow confirmed below target force: "
                            "motion/idle delta %.1fg and increased heater power"
                            % self.motion.delta)
                    gcmd.respond_info(
                        "Pressure priming successful after %gmm at %.1fg"
                        % (extruded, force))
                    return
            raise gcmd.error("Maximum pressure-prime length reached")
        except Exception as error:
            failure_reason = str(error)
            raise
        finally:
            self.moving_samples = []
            cleanup_error = None
            try:
                self.thermal.stop()
            except Exception as error:
                logging.exception("Unable to stop priming heater sampling")
                cleanup_error = error
            if client_added:
                try:
                    self.load_cell.remove_client(self._sample_callback)
                except Exception as error:
                    logging.exception(
                        "Unable to remove pressure-prime load-cell client")
                    cleanup_error = error
            if operation_claimed:
                try:
                    self.monitor.release_operation(owner)
                except Exception as error:
                    logging.exception(
                        "Unable to release pressure-prime operation lock")
                    cleanup_error = cleanup_error or error
            if original_target is not None:
                try:
                    self.printer.lookup_object("heaters").set_temperature(
                        extruder.get_heater(), original_target, False)
                except Exception as error:
                    logging.exception(
                        "Unable to restore pressure-prime heater target")
                    cleanup_error = cleanup_error or error
            self.baseline_force = None
            self.baseline_values = []
            self.active_force_limit = None
            try:
                gcmd.respond_info(format_pressure_summary(
                    forces, success, failure_reason, extruded))
            except Exception as error:
                logging.exception("Unable to report pressure-prime summary")
                cleanup_error = cleanup_error or error
            if cleanup_error is not None and failure_reason is None:
                raise cleanup_error


def load_config(config):
    return ExtrusionForcePriming(config)
