# Adaptive extrusion-only speed and temperature control
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class SpeedController:
    def __init__(self, minimum_speed_factor, speed_down_rate, speed_up_rate,
                 soft_force_margin, hard_force_margin, recovery_delay,
                 hysteresis, overload_debounce, minimum_confidence,
                 control_interval):
        self.minimum_speed_factor = minimum_speed_factor
        self.speed_down_rate = speed_down_rate
        self.speed_up_rate = speed_up_rate
        self.soft_force_margin = soft_force_margin
        self.hard_force_margin = hard_force_margin
        self.recovery_delay = recovery_delay
        self.hysteresis = hysteresis
        self.overload_debounce = overload_debounce
        self.minimum_confidence = minimum_confidence
        self.control_interval = control_interval
        self.enabled = False
        self.state = "NORMAL"
        self.speed_factor = 1.0
        self.last_update = None
        self.overload_since = None
        self.recovery_since = None

    def set_enabled(self, enabled):
        self.enabled = enabled
        if not enabled:
            self.state = "NORMAL"
            self.speed_factor = 1.0
            self.overload_since = None
            self.recovery_since = None

    def update(self, monitor_state):
        event = None
        sample_time = monitor_state["print_time"]
        if not self.enabled:
            return event
        if (self.last_update is not None
                and sample_time - self.last_update < self.control_interval):
            return event
        dt = (sample_time - self.last_update
              if self.last_update is not None else self.control_interval)
        self.last_update = sample_time
        excess = monitor_state["excess_force_g"]
        valid = (
            monitor_state["motion_state"] == "EXTRUSION_STEADY"
            and monitor_state["confidence"] >= self.minimum_confidence
            and excess is not None)
        if not valid:
            self.overload_since = None
            self.recovery_since = None
            return event
        if excess >= self.hard_force_margin:
            if self.state != "HARD_FAULT":
                event = "hard_overload"
            self.state = "HARD_FAULT"
            self.speed_factor = self.minimum_speed_factor
            return event
        if self.state == "HARD_FAULT":
            self.state = "LIMITING"
        if excess >= self.soft_force_margin:
            self.recovery_since = None
            if self.overload_since is None:
                self.overload_since = sample_time
            if sample_time - self.overload_since >= self.overload_debounce:
                if self.state != "LIMITING":
                    event = "soft_overload"
                self.state = "LIMITING"
                self.speed_factor = max(
                    self.minimum_speed_factor,
                    self.speed_factor - self.speed_down_rate * dt)
            return event
        self.overload_since = None
        if excess <= self.soft_force_margin - self.hysteresis:
            if self.speed_factor >= 1.0:
                self.speed_factor = 1.0
                self.state = "NORMAL"
                self.recovery_since = None
                return event
            if self.recovery_since is None:
                self.recovery_since = sample_time
            if sample_time - self.recovery_since >= self.recovery_delay:
                self.state = "RECOVERY"
                self.speed_factor = min(
                    1.0, self.speed_factor + self.speed_up_rate * dt)
                if self.speed_factor >= 1.0:
                    self.state = "NORMAL"
                    self.recovery_since = None
        else:
            self.recovery_since = None
        return event


class ExtrusionForceControl:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.monitor_name = config.get(
            "monitor", "extrusion_force_monitor")
        self.start_speed_enabled = config.getboolean(
            "adaptive_speed", False)
        self.start_temperature_enabled = config.getboolean(
            "adaptive_temperature", False)
        self.soft_force_margin = config.getfloat(
            "soft_force_margin", None, above=0.0)
        self.hard_force_margin = config.getfloat(
            "hard_force_margin", None, above=0.0)
        if ((self.start_speed_enabled or self.start_temperature_enabled)
                and (self.soft_force_margin is None
                     or self.hard_force_margin is None)):
            raise config.error(
                "soft_force_margin and hard_force_margin are required when "
                "adaptive control is enabled")
        if (self.soft_force_margin is not None
                and self.hard_force_margin is not None
                and self.hard_force_margin <= self.soft_force_margin):
            raise config.error(
                "hard_force_margin must exceed soft_force_margin")
        self.controller = None
        self.controller_options = {
            "minimum_speed_factor": config.getfloat(
                "min_speed_factor", 0.6, above=0.0, maxval=1.0),
            "speed_down_rate": config.getfloat(
                "speed_down_rate", 0.5, above=0.0),
            "speed_up_rate": config.getfloat(
                "speed_up_rate", 0.1, above=0.0),
            "recovery_delay": config.getfloat(
                "recovery_delay", 2.0, minval=0.0),
            "hysteresis": config.getfloat(
                "hysteresis", 50.0, minval=0.0),
            "overload_debounce": config.getfloat(
                "overload_debounce", 0.25, minval=0.0),
            "minimum_confidence": config.getfloat(
                "minimum_confidence", 0.7, minval=0.0, maxval=1.0),
            "control_interval": config.getfloat(
                "control_interval", 0.25, above=0.0),
        }
        self.temperature_enabled = False
        self.temperature_state = "TEMP_IDLE"
        self.temperature_step = config.getfloat(
            "temperature_step", 5.0, above=0.0)
        self.temperature_step_interval = config.getfloat(
            "temperature_step_interval", 10.0, above=0.0)
        self.temperature_recovery_interval = config.getfloat(
            "temperature_recovery_interval", 30.0, above=0.0)
        self.max_temperature_increase = config.getfloat(
            "max_temperature_increase", 15.0, minval=0.0)
        self.heater_safety_margin = config.getfloat(
            "heater_safety_margin", 2.0, above=0.0)
        self.base_target = None
        self.adaptive_delta = 0.0
        self.last_adaptive_target = None
        self.last_temperature_step = None
        self.temperature_recovery_since = None
        self.monitor = None
        self.latest_extruder = None
        self.gcode.register_command(
            "SET_EXTRUSION_FORCE_CONTROL", self.cmd_SET_CONTROL,
            desc="Enable or disable adaptive extrusion force control")
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        # Chain with any transform already present (notably z_sense_offset).
        gcode_move = self.printer.load_object(config, "gcode_move")
        self.normal_transform = gcode_move.set_move_transform(self, force=True)

    def _build_controller(self):
        soft_margin = (self.soft_force_margin
                       if self.soft_force_margin is not None else float("inf"))
        hard_margin = (self.hard_force_margin
                       if self.hard_force_margin is not None else float("inf"))
        self.controller = SpeedController(
            soft_force_margin=soft_margin,
            hard_force_margin=hard_margin,
            **self.controller_options)

    def _handle_ready(self):
        self.monitor = self.printer.lookup_object(self.monitor_name)
        self.monitor.add_client(self._handle_state)
        self._build_controller()
        self.controller.set_enabled(self.start_speed_enabled)
        self.temperature_enabled = self.start_temperature_enabled
        if self.temperature_enabled and not self.controller.enabled:
            raise self.printer.config_error(
                "adaptive_temperature requires adaptive_speed")

    def get_position(self):
        return list(self.normal_transform.get_position())

    def move(self, newpos, speed):
        position = list(newpos)
        current = self.normal_transform.get_position()
        positive_extrusion = (
            len(position) > 3 and len(current) > 3
            and position[3] > current[3] + 1e-12)
        if (positive_extrusion and self.controller is not None
                and self.controller.enabled):
            speed *= self.controller.speed_factor
        self.normal_transform.move(position, speed)

    def _external_target_change(self, target):
        if self.base_target is None:
            self.base_target = target
            self.last_adaptive_target = target
            return
        if (self.last_adaptive_target is not None
                and abs(target - self.last_adaptive_target) > 0.1):
            self.base_target = target
            self.adaptive_delta = 0.0
            self.last_adaptive_target = target
            self.temperature_state = "TEMP_IDLE"

    def _set_adaptive_temperature(self, extruder, target):
        heaters = self.printer.lookup_object("heaters")
        heaters.set_temperature(extruder.get_heater(), target, False)
        self.last_adaptive_target = target

    def _update_temperature(self, state):
        extruder_name = state["extruder"]
        if extruder_name is None:
            return
        extruder = self.printer.lookup_object(extruder_name, None)
        if extruder is None:
            return
        self.latest_extruder = extruder_name
        self._external_target_change(state["target_temperature"])
        if not self.temperature_enabled:
            return
        sample_time = state["print_time"]
        limiting = (
            self.controller.state == "LIMITING"
            and self.controller.speed_factor
            <= self.controller.minimum_speed_factor + 1e-9
            and state["excess_force_g"] is not None
            and state["excess_force_g"] >= self.soft_force_margin)
        if limiting:
            self.temperature_recovery_since = None
            if self.last_temperature_step is None:
                self.last_temperature_step = sample_time
            if (sample_time - self.last_temperature_step
                    < self.temperature_step_interval):
                return
            profile = self.monitor.get_active_profile(extruder_name)
            maximum = extruder.get_heater().max_temp - self.heater_safety_margin
            if profile is not None and profile.max_material_temperature is not None:
                maximum = min(maximum, profile.max_material_temperature)
            maximum = min(
                maximum, self.base_target + self.max_temperature_increase)
            new_delta = min(
                self.adaptive_delta + self.temperature_step,
                max(0.0, maximum - self.base_target))
            if new_delta > self.adaptive_delta:
                self.adaptive_delta = new_delta
                self._set_adaptive_temperature(
                    extruder, self.base_target + self.adaptive_delta)
                self.temperature_state = "TEMP_ASSIST"
            self.last_temperature_step = sample_time
            return
        self.last_temperature_step = None
        if self.adaptive_delta <= 0.0:
            self.temperature_state = "TEMP_IDLE"
            return
        if self.controller.state == "NORMAL":
            if self.temperature_recovery_since is None:
                self.temperature_recovery_since = sample_time
            if (sample_time - self.temperature_recovery_since
                    >= self.temperature_recovery_interval):
                self.adaptive_delta = max(
                    0.0, self.adaptive_delta - self.temperature_step)
                self._set_adaptive_temperature(
                    extruder, self.base_target + self.adaptive_delta)
                self.temperature_recovery_since = sample_time
                self.temperature_state = (
                    "TEMP_RECOVERY" if self.adaptive_delta > 0.0
                    else "TEMP_IDLE")
        else:
            self.temperature_recovery_since = None

    def _handle_state(self, state):
        event = self.controller.update(state)
        if event is not None:
            self.printer.send_event("extrusion_force:%s" % (event,), state)
        self._update_temperature(state)

    def _disable_temperature(self):
        if (self.adaptive_delta > 0.0 and self.latest_extruder is not None
                and self.base_target is not None):
            extruder = self.printer.lookup_object(self.latest_extruder, None)
            if extruder is not None:
                self._set_adaptive_temperature(extruder, self.base_target)
        self.temperature_enabled = False
        self.temperature_state = "TEMP_IDLE"
        self.adaptive_delta = 0.0

    def cmd_SET_CONTROL(self, gcmd):
        speed_enabled = bool(gcmd.get_int(
            "SPEED", int(self.controller.enabled), minval=0, maxval=1))
        temperature_enabled = bool(gcmd.get_int(
            "TEMP", int(self.temperature_enabled), minval=0, maxval=1))
        if (speed_enabled and (self.soft_force_margin is None
                              or self.hard_force_margin is None)):
            raise gcmd.error(
                "Configure soft_force_margin and hard_force_margin before "
                "enabling adaptive control")
        if temperature_enabled and not speed_enabled:
            raise gcmd.error("Adaptive temperature requires adaptive speed")
        self.controller.set_enabled(speed_enabled)
        if not temperature_enabled:
            self._disable_temperature()
        else:
            self.temperature_enabled = True
        gcmd.respond_info(
            "Extrusion force control: speed=%d temperature=%d"
            % (speed_enabled, temperature_enabled))

    def get_status(self, eventtime):
        return {
            "adaptive_speed": (self.controller.enabled
                               if self.controller is not None else False),
            "state": (self.controller.state
                      if self.controller is not None else "NORMAL"),
            "speed_factor": (self.controller.speed_factor
                             if self.controller is not None else 1.0),
            "adaptive_temperature": self.temperature_enabled,
            "temperature_state": self.temperature_state,
            "base_target": self.base_target,
            "adaptive_delta": self.adaptive_delta,
        }


def load_config(config):
    return ExtrusionForceControl(config)
