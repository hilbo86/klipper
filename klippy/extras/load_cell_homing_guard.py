# Load-cell collision detection during endstop homing moves
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import collections
import logging
import math
import statistics


class HomingCollisionDetector:
    """Fast, sign-independent force detector with a local frozen baseline."""
    def __init__(self, minimum_force, noise_factor, confirm_time,
                 rate_threshold=None, relative_baseline_factor=0.0):
        self.minimum_force = minimum_force
        self.noise_factor = noise_factor
        self.confirm_time = confirm_time
        self.rate_threshold = rate_threshold
        self.relative_baseline_factor = relative_baseline_factor
        self.reset()

    def reset(self):
        self.baseline = None
        self.noise = None
        self.threshold = None
        self.window = collections.deque(maxlen=3)
        self.last_time = None
        self.last_fast = None
        self.fast_time = None
        self.suspect_since = None
        self.suspect_count = 0
        self.peak_delta = 0.0
        self.peak_rate = 0.0

    def arm(self, baseline_samples):
        if len(baseline_samples) < 3:
            raise ValueError("at least three baseline samples are required")
        self.reset()
        self.baseline = statistics.median(baseline_samples)
        self.noise = statistics.pstdev(baseline_samples)
        self.threshold = max(
            self.minimum_force, self.noise * self.noise_factor,
            abs(self.baseline) * self.relative_baseline_factor)

    def update(self, print_time, force):
        if self.baseline is None:
            return None
        if self.last_time is not None and print_time <= self.last_time:
            return None
        self.window.append(force)
        self.last_time, previous_fast = print_time, self.last_fast
        # Three-point median rejects a single ADC outlier without the slow
        # extrusion monitor filter. Wait for a full window before detecting.
        if len(self.window) < 3:
            return None
        fast = statistics.median(self.window)
        self.last_fast = fast
        delta = fast - self.baseline
        self.peak_delta = max(self.peak_delta, abs(delta))
        rate = 0.0
        if previous_fast is not None:
            rate = (fast - previous_fast) / (print_time - self.fast_time)
        self.fast_time = print_time
        self.peak_rate = max(self.peak_rate, abs(rate))
        suspect = abs(delta) >= self.threshold
        if self.rate_threshold is not None:
            suspect |= (abs(rate) >= self.rate_threshold
                        and abs(delta) >= self.threshold * 0.5)
        if not suspect:
            self.suspect_since = None
            self.suspect_count = 0
            return None
        if self.suspect_since is None:
            self.suspect_since = print_time
            self.suspect_count = 1
            return None
        self.suspect_count += 1
        if (self.suspect_count >= 2
                and print_time - self.suspect_since >= self.confirm_time):
            return {"print_time": print_time, "force_g": fast,
                    "baseline_force_g": self.baseline,
                    "delta_g": delta, "threshold_g": self.threshold,
                    "force_rate_g_s": rate}
        return None


class LoadCellHomingGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.enabled = config.getboolean("enabled", False)
        self.mode = config.getchoice(
            "mode", {"diagnostic": "diagnostic", "abort": "abort"},
            default="diagnostic")
        self.load_cell_name = config.get("load_cell", "load_cell")
        self.monitor_name = config.get("monitor", "extrusion_force_monitor")
        self.settle_time = config.getfloat("settle_time", 0.2, minval=0.0)
        self.baseline_time = config.getfloat("baseline_time", 0.3, above=0.0)
        self.minimum_force = config.getfloat(
            "minimum_collision_force", None, above=0.0)
        self.axis_forces = {
            axis: config.getfloat("collision_force_" + axis, None, above=0.0)
            for axis in "xyz"}
        if any(value is not None and not math.isfinite(value)
               for value in [self.minimum_force] + list(self.axis_forces.values())):
            raise config.error("Homing collision thresholds must be finite")
        self.noise_factor = config.getfloat("noise_factor", 8.0, above=0.0)
        self.relative_baseline_factor = config.getfloat(
            "relative_baseline_factor", 0.0, minval=0.0)
        self.rate_threshold = config.getfloat(
            "force_rate_threshold", None, above=0.0)
        self.confirm_time = config.getfloat("confirm_time", 0.02, minval=0.0)
        self.enable_extruder_steppers = config.getboolean(
            "enable_extruder_steppers", True)
        self.guard_axes = {
            axis: config.getboolean("guard_" + axis, True)
            for axis in "xyz"}
        self.collision_retract = config.getboolean("collision_retract", False)
        self.retract_distance = config.getfloat(
            "collision_retract_distance", 2.0, above=0.0)
        self.retract_speed = config.getfloat(
            "collision_retract_speed", 5.0, above=0.0)
        if (self.enabled and self.mode == "abort"
                and self.minimum_force is None
                and any(self.guard_axes[a] and self.axis_forces[a] is None
                        for a in "xyz")):
            raise config.error("Set minimum_collision_force or every enabled "
                               "axis threshold before abort mode")
        self.load_cell = None
        self.monitor = None
        self.toolhead = None
        self.detector = None
        self.baseline_samples = []
        self.state = "DISABLED"
        self.axis = None
        self.motion_vector = [0.0, 0.0, 0.0]
        self.collision = None
        self.last_collision = None
        self.collision_count = 0
        self.axis_peaks = {}
        self.completion = None
        self.move_start_time = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        if not self.enabled:
            return
        self.load_cell = self.printer.lookup_object(self.load_cell_name)
        if not self.load_cell.get_status(self.reactor.monotonic()).get(
                "is_calibrated", False):
            raise self.printer.config_error(
                "load_cell_homing_guard requires calibrated force in grams")
        self.monitor = self.printer.lookup_object(self.monitor_name, None)
        self.toolhead = self.printer.lookup_object("toolhead")
        self.load_cell.add_client(self._handle_sample)

    def _handle_sample(self, sample):
        if self.state == "BASELINING":
            self.baseline_samples.append(sample["absolute_force_g"])
        elif (self.state in ("ARMED", "SUSPECT")
              and self.move_start_time is not None
              and sample["print_time"] >= self.move_start_time):
            result = self.detector.update(
                sample["print_time"], sample["absolute_force_g"])
            self.state = ("SUSPECT" if self.detector.suspect_since is not None
                          else "ARMED")
            if result is not None:
                if self.collision is not None:
                    return
                result.update({"axis": self.axis,
                               "motion_vector": list(self.motion_vector)})
                self.collision = result
                self.last_collision = dict(result)
                self.collision_count += 1
                self.state = ("COLLISION" if self.mode == "abort"
                              else "ARMED")
                logging.error(
                    "Load cell homing collision detected: axis=%s "
                    "force=%.1fg baseline=%.1fg delta=%.1fg "
                    "threshold=%.1fg motion=%s", self.axis,
                    result["force_g"], result["baseline_force_g"],
                    result["delta_g"], result["threshold_g"],
                    self.motion_vector)
                if self.mode == "abort" and self.completion is not None:
                    self.completion.complete(1)
                if self.mode == "diagnostic":
                    self.printer.send_event(
                        "load_cell_homing:collision", result)

    def prepare_move(self, movepos, probe_pos=False):
        if not self.enabled or probe_pos:
            return False
        if self.monitor is not None and self.monitor.get_active_operation():
            self.state = "SUSPENDED"
            return False
        startpos = self.toolhead.get_position()
        delta = [movepos[i] - startpos[i] for i in range(3)]
        axes = ["xyz"[i] for i in range(3)
                if abs(delta[i]) > 1e-9 and self.guard_axes["xyz"[i]]]
        if not axes:
            return False
        distance = math.sqrt(sum(d * d for d in delta))
        self.axis = "".join(axes)
        self.motion_vector = [d / distance for d in delta]
        threshold = min(
            self.axis_forces[a] or self.minimum_force or float("inf")
            for a in axes)
        self.detector = HomingCollisionDetector(
            threshold, self.noise_factor, self.confirm_time,
            self.rate_threshold, self.relative_baseline_factor)
        self.state = "PREPARING"
        self.collision = None
        self.toolhead.wait_moves()
        if self.enable_extruder_steppers:
            enable = self.printer.lookup_object("stepper_enable")
            steppers = [name for name in enable.get_steppers()
                        if name.startswith("extruder")]
            if steppers:
                enable.set_motors_enable(steppers, True)
                self.toolhead.wait_moves()
        self.reactor.pause(self.reactor.monotonic() + self.settle_time)
        self.baseline_samples = []
        self.state = "BASELINING"
        self.reactor.pause(self.reactor.monotonic() + self.baseline_time)
        try:
            self.detector.arm(self.baseline_samples)
        except ValueError as exc:
            self.state = "FAULT"
            raise self.printer.command_error(
                "Load cell homing baseline failed: %s" % (exc,))
        self.state = "ARMED"
        self.printer.send_event("load_cell_homing:armed", self.get_status(0.0))
        return True

    def start_move(self, print_time):
        self.move_start_time = print_time
        if self.mode == "abort":
            self.completion = self.reactor.completion()
        return self.completion

    def finish_move(self, was_collision=False):
        if self.detector is not None and self.axis is not None:
            peaks = self.axis_peaks.setdefault(
                self.axis, {"force_delta_g": 0.0, "force_rate_g_s": 0.0,
                            "noise_g": 0.0, "moves": 0})
            peaks["force_delta_g"] = max(
                peaks["force_delta_g"], self.detector.peak_delta)
            peaks["force_rate_g_s"] = max(
                peaks["force_rate_g_s"], self.detector.peak_rate)
            peaks["noise_g"] = max(peaks["noise_g"], self.detector.noise)
            peaks["moves"] += 1
        self.completion = None
        self.move_start_time = None
        self.state = "FAULT" if was_collision else "DISABLED"
        self.printer.send_event("load_cell_homing:finished", self.get_status(0.0))

    def get_status(self, eventtime):
        detector = self.detector
        threshold = detector.threshold if detector else None
        if threshold is not None and not math.isfinite(threshold):
            threshold = None
        return {
            "enabled": self.enabled, "mode": self.mode,
            "state": self.state, "axis": self.axis,
            "motion_vector": list(self.motion_vector),
            "baseline_force_g": (detector.baseline if detector else None),
            "noise_g": (detector.noise if detector else None),
            "collision_threshold_g": threshold,
            "force_g": (detector.last_fast if detector else None),
            "force_delta_g": (
                detector.last_fast - detector.baseline
                if detector is not None and detector.last_fast is not None
                and detector.baseline is not None else None),
            "peak_delta_g": (detector.peak_delta if detector else None),
            "peak_rate_g_s": (detector.peak_rate if detector else None),
            "axis_peaks": {axis: dict(peaks)
                           for axis, peaks in self.axis_peaks.items()},
            "last_collision": self.last_collision,
            "last_collision_axis": (self.last_collision["axis"]
                                    if self.last_collision else None),
            "last_collision_force_g": (self.last_collision["force_g"]
                                       if self.last_collision else None),
            "last_collision_delta_g": (self.last_collision["delta_g"]
                                       if self.last_collision else None),
            "collision_count": self.collision_count,
        }


def load_config(config):
    return LoadCellHomingGuard(config)
