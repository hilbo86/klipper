# Extrusion delivery-failure, overload, jam, and clog detection
#
# Copyright (C) 2026  Timo Hilbig <gh@t-hilbig.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
import math


class ExtrusionForceGuardLogic:
    def __init__(self, minimum_monitor_flow, minimum_expected_force,
                 minimum_confidence, underload_ratio, underload_time,
                 underload_filament_length, soft_force_margin,
                 hard_force_margin, hard_overload_time, health_tau=120.0,
                 clog_warning_score=None):
        self.minimum_monitor_flow = minimum_monitor_flow
        self.minimum_expected_force = minimum_expected_force
        self.minimum_confidence = minimum_confidence
        self.underload_ratio = underload_ratio
        self.underload_time = underload_time
        self.underload_filament_length = underload_filament_length
        self.soft_force_margin = soft_force_margin
        self.hard_force_margin = hard_force_margin
        self.hard_overload_time = hard_overload_time
        self.health_tau = health_tau
        self.clog_warning_score = clog_warning_score
        self.enabled = False
        self.state = "DISABLED"
        self.underload_start = None
        self.underload_e_start = None
        self.underload_distance = 0.0
        self.hard_start = None
        self.last_time = None
        self.last_e_position = None
        self.health = None
        self.clog_warning = False

    def set_enabled(self, enabled):
        self.enabled = enabled
        self.state = "IDLE" if enabled else "DISABLED"
        self._reset_suspects()

    def _reset_suspects(self):
        self.underload_start = None
        self.underload_e_start = None
        self.underload_distance = 0.0
        self.hard_start = None

    def _note_distance(self, state, dt):
        e_position = state.get("e_position")
        if e_position is not None and self.last_e_position is not None:
            distance = max(0.0, e_position - self.last_e_position)
        else:
            distance = max(0.0, state["e_velocity"] * dt)
        self.last_e_position = e_position
        return distance

    def _update_health(self, state, dt, events):
        expected = state["expected_dynamic_force_g"]
        measured = state["force_trend_g"]
        if expected is None or expected <= 0.0 or measured <= 0.0:
            return
        instantaneous = min(1.0, expected / measured)
        if self.health is None:
            self.health = instantaneous
        elif dt > 0.0:
            alpha = 1.0 - math.exp(-dt / self.health_tau)
            self.health += alpha * (instantaneous - self.health)
        clog_score = 1.0 - self.health
        warning = (self.clog_warning_score is not None
                   and clog_score >= self.clog_warning_score)
        if warning and not self.clog_warning:
            events.append("clog_warning")
        self.clog_warning = warning

    def update(self, state):
        events = []
        fault = None
        sample_time = state["print_time"]
        dt = (max(0.0, sample_time - self.last_time)
              if self.last_time is not None else 0.0)
        self.last_time = sample_time
        distance = self._note_distance(state, dt)
        if not self.enabled:
            return events, fault
        expected = state["expected_dynamic_force_g"]
        eligible = (
            state["motion_state"] == "EXTRUSION_STEADY"
            and state["flow_mm3_s"] >= self.minimum_monitor_flow
            and expected is not None
            and expected >= self.minimum_expected_force
            and state["confidence"] >= self.minimum_confidence)
        if not eligible:
            self.state = (
                "ARMING" if state["motion_state"] == "EXTRUSION_TRANSIENT"
                else "IDLE")
            self._reset_suspects()
            return events, fault

        self._update_health(state, dt, events)
        measured = state["force_control_g"]
        excess = state["excess_force_g"]
        if measured < expected * self.underload_ratio:
            if self.underload_start is None:
                self.underload_start = sample_time
                self.underload_e_start = state.get("e_position")
                self.underload_distance = 0.0
            self.underload_distance += distance
            self.state = "UNDERLOAD_SUSPECT"
            if (sample_time - self.underload_start >= self.underload_time
                    and self.underload_distance
                    >= self.underload_filament_length):
                self.state = "DELIVERY_FAILURE"
                fault = "DELIVERY_FAILURE"
                events.append("delivery_failure")
                return events, fault
        else:
            self.underload_start = None
            self.underload_e_start = None
            self.underload_distance = 0.0

        if excess is not None and excess >= self.hard_force_margin:
            if self.hard_start is None:
                self.hard_start = sample_time
                events.append("hard_overload")
            self.state = "OVERLOAD_SUSPECT"
            if sample_time - self.hard_start >= self.hard_overload_time:
                self.state = "JAM"
                fault = "JAM"
                events.append("jam")
                return events, fault
        else:
            self.hard_start = None
            if (excess is not None
                    and excess >= self.soft_force_margin):
                if self.state != "OVERLOAD_SUSPECT":
                    events.append("soft_overload")
                self.state = "OVERLOAD_SUSPECT"
            elif self.underload_start is None:
                self.state = "MONITORING"
        return events, fault

    def get_status(self):
        return {
            "enabled": self.enabled,
            "state": self.state,
            "nozzle_health": self.health,
            "clog_score": (1.0 - self.health
                           if self.health is not None else None),
        }


class ExtrusionForceGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.monitor_name = config.get(
            "monitor", "extrusion_force_monitor")
        self.start_enabled = config.getboolean("guard", False)
        required = {
            "minimum_monitor_flow": config.getfloat(
                "minimum_monitor_flow", None, minval=0.0),
            "minimum_expected_force": config.getfloat(
                "minimum_expected_force", None, minval=0.0),
            "underload_ratio": config.getfloat(
                "underload_ratio", None, above=0.0, below=1.0),
            "underload_time": config.getfloat(
                "underload_time", None, above=0.0),
            "underload_filament_length": config.getfloat(
                "underload_filament_length", None, above=0.0),
            "soft_force_margin": config.getfloat(
                "soft_force_margin", None, above=0.0),
            "hard_force_margin": config.getfloat(
                "hard_force_margin", None, above=0.0),
            "hard_overload_time": config.getfloat(
                "hard_overload_time", None, above=0.0),
        }
        if self.start_enabled and any(value is None for value in required.values()):
            missing = [name for name, value in required.items()
                       if value is None]
            raise config.error(
                "Guard thresholds must be configured when guard is enabled: %s"
                % (", ".join(missing),))
        self.required = required
        self.minimum_confidence = config.getfloat(
            "minimum_confidence", 0.8, minval=0.0, maxval=1.0)
        self.health_tau = config.getfloat(
            "health_tau", 120.0, above=0.0)
        self.clog_warning_score = config.getfloat(
            "clog_warning_score", None, above=0.0, below=1.0)
        self.pause_on_delivery_failure = config.getboolean(
            "pause_on_delivery_failure", True)
        self.pause_on_jam = config.getboolean("pause_on_jam", True)
        if self.pause_on_delivery_failure or self.pause_on_jam:
            self.printer.load_object(config, "pause_resume")
        gcode_macro = self.printer.load_object(config, "gcode_macro")
        self.delivery_template = gcode_macro.load_template(
            config, "delivery_failure_gcode", "")
        self.jam_template = gcode_macro.load_template(config, "jam_gcode", "")
        self.logic = None
        self.monitor = None
        self.last_fault = None
        self.last_fault_time = None
        self.delivery_failures = 0
        self.jams = 0
        self.fault_pending = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode.register_command(
            "SET_EXTRUSION_FORCE_GUARD", self.cmd_SET_GUARD,
            desc="Enable or disable extrusion force fault detection")

    def _build_logic(self):
        if any(value is None for value in self.required.values()):
            raise self.printer.command_error(
                "Configure all extrusion_force_guard detection thresholds "
                "before enabling it")
        self.logic = ExtrusionForceGuardLogic(
            minimum_confidence=self.minimum_confidence,
            health_tau=self.health_tau,
            clog_warning_score=self.clog_warning_score,
            **self.required)

    def _handle_ready(self):
        self.monitor = self.printer.lookup_object(self.monitor_name)
        self.monitor.add_client(self._handle_state)
        if self.start_enabled:
            self._build_logic()
            self.logic.set_enabled(True)

    def _handle_state(self, state):
        if self.logic is None:
            return
        events, fault = self.logic.update(state)
        for event in events:
            self.printer.send_event("extrusion_force:%s" % (event,), state)
        if fault is not None and not self.fault_pending:
            self.fault_pending = True
            self.last_fault = fault
            self.last_fault_time = state["print_time"]
            if fault == "DELIVERY_FAILURE":
                self.delivery_failures += 1
            else:
                self.jams += 1
            self.reactor.register_callback(
                lambda eventtime: self._handle_fault(eventtime, fault))

    def _handle_fault(self, eventtime, fault):
        try:
            pause = ((fault == "DELIVERY_FAILURE"
                      and self.pause_on_delivery_failure)
                     or (fault == "JAM" and self.pause_on_jam))
            script = "PAUSE\n" if pause else ""
            template = (self.delivery_template
                        if fault == "DELIVERY_FAILURE" else self.jam_template)
            script += template.render()
            if script.strip():
                self.gcode.run_script(script + "\nM400")
        except Exception:
            logging.exception("Extrusion force guard fault script failed")

    def cmd_SET_GUARD(self, gcmd):
        enabled = bool(gcmd.get_int("ENABLE", 1, minval=0, maxval=1))
        if self.logic is None:
            self._build_logic()
        self.logic.set_enabled(enabled)
        self.fault_pending = False
        gcmd.respond_info(
            "Extrusion force guard %s" % ("enabled" if enabled else "disabled"))

    def get_status(self, eventtime):
        status = (self.logic.get_status() if self.logic is not None
                  else {"enabled": False, "state": "DISABLED",
                        "nozzle_health": None, "clog_score": None})
        status.update({
            "last_fault": self.last_fault,
            "last_fault_time": self.last_fault_time,
            "delivery_failures": self.delivery_failures,
            "jams": self.jams,
        })
        return status


def load_config(config):
    return ExtrusionForceGuard(config)
