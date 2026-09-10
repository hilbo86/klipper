# Continuous toolhead jogging from physical buttons
#
# Copyright (C) 2026  Timo H.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mcu


HOMING_START_DELAY = 0.001
MODE_GUARD_INTERVAL = 0.100
VIRTUAL_CLICK_DELAY = 0.001


class JogInput:
    def __init__(self, name, axis, direction, speed):
        self.name = name
        self.axis = axis
        self.direction = direction
        self.speed = speed
        self.pressed = False
        self.forwarded = False


class VirtualButtonRegistration:
    def __init__(self, names, callback):
        self.names = names
        self.callback = callback
        self.state = 0

    def update(self, eventtime, name, state):
        bit = 1 << self.names.index(name)
        new_state = self.state | bit if state else self.state & ~bit
        if new_state == self.state:
            return
        self.state = new_state
        self.callback(eventtime, self.state)


class JogButtons:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_mutex = self.gcode.get_mutex()
        self.toolhead = self.kin = self.dispatch = None
        self.kin_steppers = self.trigger_steppers = []
        self.is_ready = self.enabled = self.dispatch_active = False
        self.active_input = None
        self.last_reject = ""
        self.enable_hold_time = config.getfloat(
            'enable_hold_time', 2., minval=2.)
        self.extrude_distance = config.getfloat(
            'extrude_distance', 25., above=0.)
        xy_speed = config.getfloat('xy_speed', 40., above=0.)
        z_speed = config.getfloat('z_speed', 5., above=0.)
        extrude_speed = config.getfloat('extrude_speed', 5., above=0.)
        input_defs = [
            ('x_minus', 0, -1, xy_speed),
            ('x_plus', 0, 1, xy_speed),
            ('y_minus', 1, -1, xy_speed),
            ('y_plus', 1, 1, xy_speed),
            ('z_minus', 2, -1, z_speed),
            ('z_plus', 2, 1, z_speed),
            ('extrude_minus', 3, -1, extrude_speed),
            ('extrude_plus', 3, 1, extrude_speed),
        ]
        self.inputs = []
        self.inputs_by_name = {}
        for name, axis, direction, speed in input_defs:
            pin = config.get(name + '_pin', None)
            if pin is None:
                continue
            jog_input = JogInput(name, axis, direction, speed)
            self.inputs.append(jog_input)
            self.inputs_by_name[name] = jog_input
        if not self.inputs:
            raise config.error("[jog_buttons] must define a motion pin")
        self.ok_input = None
        ok_pin = config.get('ok_pin', None)
        if ok_pin is not None:
            self.ok_input = JogInput('ok', None, 0, 0.)
            self.inputs_by_name['ok'] = self.ok_input
        emergency_pin = config.get('emergency_stop_pin', None)
        self.virtual_callbacks = {
            name: [] for name in self.inputs_by_name
        }
        if emergency_pin is not None:
            self.virtual_callbacks['emergency_stop'] = []
        ppins = self.printer.lookup_object('pins')
        self.pin_error = ppins.error
        ppins.register_chip('jog_buttons', self)
        buttons = self.printer.load_object(config, 'buttons')
        for jog_input in self.inputs:
            pin = config.get(jog_input.name + '_pin')
            callback = lambda e, s, ji=jog_input: (
                self._motion_button_event(e, ji, s))
            buttons.register_debounce_button(pin, callback, config)
        if self.ok_input is not None:
            buttons.register_debounce_button(
                ok_pin, self._ok_button_event, config)
        if emergency_pin is not None:
            buttons.register_debounce_button(
                emergency_pin, self._emergency_stop_event, config)
        self.ok_hold_target = None
        self.ok_hold_handled = False
        self.ok_hold_timer = self.reactor.register_timer(
            self._ok_hold_event)
        self.mode_guard_timer = self.reactor.register_timer(
            self._mode_guard_event)
        self.gcode.register_command(
            'SET_JOG_MODE', self.cmd_SET_JOG_MODE,
            desc=self.cmd_SET_JOG_MODE_help)
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint(
            'jog_buttons/set_mode', self._handle_set_mode_request)
        self.printer.register_event_handler(
            'klippy:mcu_identify', self._handle_mcu_identify)
        self.printer.register_event_handler(
            'klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'klippy:shutdown', self._handle_shutdown)
        self.printer.register_event_handler(
            'toolhead:sync_print_time', self._handle_motion_start)
        self.printer.register_event_handler(
            'stepper_enable:motor_off', self._handle_motor_off)

    # Logical button chip interface used by extras/buttons.py.
    def register_button_callback(self, pin_params_list, callback):
        names = []
        for pin_params in pin_params_list:
            name = pin_params['pin']
            if name not in self.virtual_callbacks:
                raise self.pin_error(
                    "Unknown jog_buttons virtual pin '%s'" % (name,))
            if pin_params['invert'] or pin_params['pullup']:
                raise self.pin_error(
                    "jog_buttons virtual pins cannot be inverted or pulled up")
            names.append(name)
        registration = VirtualButtonRegistration(names, callback)
        for name in names:
            self.virtual_callbacks[name].append(registration)

    def _emit_virtual(self, eventtime, name, state):
        for registration in self.virtual_callbacks[name]:
            registration.update(eventtime, name, bool(state))

    def _handle_mcu_identify(self):
        # TriggerDispatch must allocate its MCU objects before MCU config.
        self.toolhead = self.printer.lookup_object('toolhead')
        self.kin = self.toolhead.get_kinematics()
        self.kin_steppers = self.kin.get_steppers()
        if not self.kin_steppers:
            raise self.printer.config_error(
                "[jog_buttons] requires kinematic steppers")
        trigger_steppers = list(self.kin_steppers)
        extruders = [
            (name, obj) for name, obj in self.printer.lookup_objects()
            if (name == 'extruder'
                or (name.startswith('extruder') and name[8:].isdigit()))
        ]
        for name, extruder in extruders:
            extruder_stepper = getattr(extruder, 'extruder_stepper', None)
            if extruder_stepper is not None:
                trigger_steppers.append(extruder_stepper.stepper)
        for name, wrapper in self.printer.lookup_objects('extruder_stepper'):
            trigger_steppers.append(wrapper.extruder_stepper.stepper)
        self.trigger_steppers = []
        for stepper in trigger_steppers:
            if stepper not in self.trigger_steppers:
                self.trigger_steppers.append(stepper)
        self.dispatch = mcu.TriggerDispatch(
            self.trigger_steppers[0].get_mcu())
        for stepper in self.trigger_steppers:
            self.dispatch.add_stepper(stepper)

    def _handle_ready(self):
        self.idle_timeout = self.printer.lookup_object('idle_timeout')
        self.pause_resume = self.printer.lookup_object('pause_resume', None)
        self.print_stats = self.printer.lookup_object('print_stats', None)
        self.virtual_sd = self.printer.lookup_object('virtual_sdcard', None)
        self.is_ready = True

    def _handle_shutdown(self):
        self.is_ready = self.enabled = self.dispatch_active = False
        self.active_input = None

    def _handle_motion_start(self, curtime, print_time, est_print_time):
        if self.enabled and self.active_input is None:
            self._set_enabled(False, "another toolhead operation started")

    def _handle_motor_off(self):
        if self.enabled:
            self._set_enabled(False, "the steppers were disabled")

    def _emergency_stop_event(self, eventtime, state):
        if state and not self.printer.is_shutdown():
            self.printer.invoke_shutdown(
                "Shutdown due to jog emergency-stop button")
        self._emit_virtual(eventtime, 'emergency_stop', state)

    def get_status(self, eventtime):
        active = self.active_input
        return {
            'enabled': self.enabled,
            'active': active is not None,
            'axis': "" if active is None else "xyze"[active.axis],
            'direction': 0 if active is None else active.direction,
            'last_reject': self.last_reject,
        }

    def _job_reject_reason(self, eventtime, check_gcode=True,
                           check_idle_state=True):
        if not self.is_ready or self.printer.is_shutdown():
            return "printer is not ready"
        if check_gcode and self.gcode_mutex.test():
            return "G-Code is busy"
        if self.virtual_sd is not None and self.virtual_sd.is_active():
            return "a virtual SD print is active"
        if self.pause_resume is not None and self.pause_resume.is_paused:
            return "the printer is paused"
        if self.print_stats is not None:
            state = self.print_stats.get_status(eventtime)['state']
            if state in ('printing', 'paused'):
                return "print state is %s" % (state,)
        idle_state = self.idle_timeout.get_status(eventtime)['state']
        if check_idle_state and idle_state == 'Printing':
            return "the printer is not idle"
        print_time, est_print_time, lookahead_empty = (
            self.toolhead.check_busy(eventtime))
        if not lookahead_empty or print_time > est_print_time + 0.001:
            return "the toolhead is busy"
        homed_axes = self.toolhead.get_status(eventtime)['homed_axes']
        if any(axis not in homed_axes for axis in 'xyz'):
            return "all XYZ axes must be homed"
        return None

    def _activation_reject_reason(self, eventtime, check_gcode=True):
        reason = self._job_reject_reason(
            eventtime, check_gcode=check_gcode, check_idle_state=True)
        if reason is not None:
            return reason
        if any(jog_input.pressed for jog_input in self.inputs):
            return "a motion button is pressed"
        return None

    def _set_enabled(self, enabled, reason=None):
        enabled = bool(enabled)
        if self.enabled == enabled:
            return
        self.enabled = enabled
        if not enabled and self.dispatch_active:
            self.dispatch.trigger()
        guard_time = self.reactor.NEVER
        if enabled:
            guard_time = self.reactor.monotonic() + MODE_GUARD_INTERVAL
        self.reactor.update_timer(self.mode_guard_timer, guard_time)
        if enabled:
            menu = self.printer.lookup_object('menu', None)
            if menu is not None:
                menu.exit(force=True)
        display = self.printer.lookup_object('display', None)
        if display is not None:
            display.request_redraw()
        if reason is not None:
            logging.info("Manual jog mode disabled: %s", reason)
        self.printer.send_event('jog_buttons:mode_changed', enabled)

    def _mode_guard_event(self, eventtime):
        if not self.enabled:
            return self.reactor.NEVER
        if self.active_input is not None:
            return eventtime + MODE_GUARD_INTERVAL
        reason = self._job_reject_reason(
            eventtime, check_gcode=True, check_idle_state=False)
        if reason is not None:
            self.last_reject = reason
            self._set_enabled(False, reason)
            return self.reactor.NEVER
        return eventtime + MODE_GUARD_INTERVAL

    def _try_enable(self, eventtime, check_gcode=True):
        reason = self._activation_reject_reason(
            eventtime, check_gcode=check_gcode)
        if reason is not None:
            self.last_reject = reason
            return reason
        self.last_reject = ""
        self._set_enabled(True)
        return None

    def _motion_button_event(self, eventtime, jog_input, state):
        jog_input.pressed = bool(state)
        if jog_input.forwarded:
            self._emit_virtual(eventtime, jog_input.name, state)
            if not state:
                jog_input.forwarded = False
            return
        if not state:
            if self.active_input is jog_input and self.dispatch_active:
                self.dispatch.trigger()
            return
        if not self.enabled:
            jog_input.forwarded = True
            self._emit_virtual(eventtime, jog_input.name, True)
            return
        if self.active_input is not None:
            return
        reason = self._job_reject_reason(
            eventtime, check_gcode=True, check_idle_state=False)
        if reason is not None:
            self.last_reject = reason
            self._set_enabled(False, reason)
            jog_input.forwarded = True
            self._emit_virtual(eventtime, jog_input.name, True)
            return
        if jog_input.axis == 3:
            extruder = self.toolhead.get_extruder()
            if (getattr(extruder, 'extruder_stepper', None) is None
                or not extruder.get_heater().can_extrude):
                self.last_reject = "active extruder is below minimum temp"
                return
        with self.gcode_mutex:
            self._run_jog(jog_input)

    def _ok_button_event(self, eventtime, state):
        ok_input = self.ok_input
        ok_input.pressed = bool(state)
        if state:
            ok_input.forwarded = False
            self.ok_hold_handled = False
            self.ok_hold_target = not self.enabled
            if self.ok_hold_target:
                reason = self._activation_reject_reason(eventtime)
                if reason is not None:
                    self.ok_hold_target = None
                    ok_input.forwarded = True
                    self._emit_virtual(eventtime, 'ok', True)
                    return
            self.reactor.update_timer(
                self.ok_hold_timer, eventtime + self.enable_hold_time)
            return
        self.reactor.update_timer(
            self.ok_hold_timer, self.reactor.NEVER)
        if ok_input.forwarded:
            self._emit_virtual(eventtime, 'ok', False)
            ok_input.forwarded = False
        elif not self.ok_hold_handled and self.ok_hold_target:
            self._emit_virtual(eventtime, 'ok', True)
            self.reactor.register_callback(
                lambda e: self._emit_virtual(e, 'ok', False),
                eventtime + VIRTUAL_CLICK_DELAY)
        self.ok_hold_target = None
        self.ok_hold_handled = False

    def _ok_hold_event(self, eventtime):
        if not self.ok_input.pressed or self.ok_hold_target is None:
            return self.reactor.NEVER
        if self.ok_hold_target:
            reason = self._try_enable(eventtime)
            if reason is not None:
                self.ok_hold_target = None
                self.ok_input.forwarded = True
                self._emit_virtual(eventtime, 'ok', True)
                return self.reactor.NEVER
            self.gcode.respond_info("Manual jog mode enabled")
        else:
            self._set_enabled(False)
            self.gcode.respond_info("Manual jog mode disabled")
        self.ok_hold_handled = True
        return self.reactor.NEVER

    cmd_SET_JOG_MODE_help = "Enable or disable guarded manual jogging"
    def cmd_SET_JOG_MODE(self, gcmd):
        enable = bool(gcmd.get_int('ENABLE', minval=0, maxval=1))
        if enable:
            reason = self._try_enable(
                self.reactor.monotonic(), check_gcode=False)
            if reason is not None:
                raise gcmd.error("Unable to enable manual jog mode: %s"
                                  % (reason,))
        else:
            self._set_enabled(False)
        state = "enabled" if self.enabled else "disabled"
        gcmd.respond_info("Manual jog mode %s" % (state,))

    def _handle_set_mode_request(self, web_request):
        enable = web_request.get('enable', types=(bool, int))
        if enable not in (False, True, 0, 1):
            raise web_request.error("enable must be true or false")
        if enable:
            reason = self._try_enable(self.reactor.monotonic())
            if reason is not None:
                raise web_request.error(
                    "Unable to enable manual jog mode: %s" % (reason,))
        else:
            self._set_enabled(False)
        web_request.send(self.get_status(self.reactor.monotonic()))

    def _calc_halt_position(self, start_kin_pos, start_mcu_pos):
        halt_kin_pos = dict(start_kin_pos)
        for stepper in self.kin_steppers:
            step_delta = (stepper.get_mcu_position()
                          - start_mcu_pos[stepper])
            halt_kin_pos[stepper.get_name()] += (
                step_delta * stepper.get_step_dist())
        current_pos = self.toolhead.get_position()
        halt_xyz = self.kin.calc_position(halt_kin_pos)
        return [p if p is not None else current_pos[i]
                for i, p in enumerate(halt_xyz)] + current_pos[3:]

    def _sync_extruder_position(self, extruder, start_pos,
                                start_mcu_pos):
        extruder_stepper = extruder.extruder_stepper.stepper
        step_delta = (extruder_stepper.get_mcu_position()
                      - start_mcu_pos[extruder_stepper])
        halt_e = start_pos[3] + step_delta * extruder_stepper.get_step_dist()
        for stepper in self.trigger_steppers:
            if stepper.get_trapq() is extruder.get_trapq():
                stepper.set_position([halt_e, 0., 0.])
        extruder.last_position = halt_e
        self.toolhead.set_extruder(extruder, halt_e)

    def _run_extruder_jog(self, jog_input, completion):
        while jog_input.pressed and self.enabled:
            target_pos = self.toolhead.get_position()
            target_pos[3] += (
                jog_input.direction * self.extrude_distance)
            self.toolhead.drip_move(
                target_pos, jog_input.speed, completion,
                allow_extra_axes=True)
            if completion.test():
                break

    def _run_jog(self, jog_input):
        axis = jog_input.axis
        current_pos = self.toolhead.get_position()
        if axis < 3:
            status = self.toolhead.get_status(self.reactor.monotonic())
            limit = (status['axis_maximum'][axis]
                     if jog_input.direction > 0
                     else status['axis_minimum'][axis])
            if abs(limit - current_pos[axis]) < 0.000000001:
                self.last_reject = "axis is at its software limit"
                return
            target_pos = list(current_pos)
            target_pos[axis] = limit
        active_extruder = None
        if axis == 3:
            active_extruder = self.toolhead.get_extruder()
        self.toolhead.flush_step_generation()
        start_kin_pos = {
            s.get_name(): s.get_commanded_position()
            for s in self.kin_steppers
        }
        start_mcu_pos = {
            s: s.get_mcu_position() for s in self.trigger_steppers
        }
        self.active_input = jog_input
        self.last_reject = ""
        error = None
        reason = None
        try:
            print_time = self.toolhead.get_last_move_time()
            completion = self.dispatch.start(print_time)
            self.dispatch_active = True
            if not jog_input.pressed or not self.enabled:
                self.dispatch.trigger()
            self.toolhead.dwell(HOMING_START_DELAY)
            if axis == 3:
                self._run_extruder_jog(jog_input, completion)
            else:
                self.toolhead.drip_move(
                    target_pos, jog_input.speed, completion)
        except self.printer.command_error as e:
            error = str(e)
            if self.dispatch_active:
                self.dispatch.trigger()
        except Exception:
            logging.exception("Continuous button jog failed")
            error = "internal jog error"
            if self.dispatch_active:
                self.dispatch.trigger()
        finally:
            if self.dispatch_active:
                try:
                    move_end_time = self.toolhead.get_last_move_time()
                    try:
                        self.dispatch.wait_end(move_end_time)
                    finally:
                        reason = self.dispatch.stop()
                except Exception:
                    logging.exception("Error stopping continuous button jog")
                    if error is None:
                        error = "unable to stop jog cleanly"
                self.dispatch_active = False
            if not self.printer.is_shutdown():
                try:
                    self.toolhead.flush_step_generation()
                    halt_pos = self._calc_halt_position(
                        start_kin_pos, start_mcu_pos)
                    self.toolhead.set_position(halt_pos)
                    if active_extruder is not None:
                        self._sync_extruder_position(
                            active_extruder, current_pos, start_mcu_pos)
                except Exception:
                    logging.exception(
                        "Unable to synchronize position after jog")
                    if error is None:
                        error = "unable to synchronize position after jog"
            self.active_input = None
        if (reason is not None
            and reason >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT):
            error = "communication timeout while stopping jog"
        if error is not None and not self.printer.is_shutdown():
            self.last_reject = error
            self.gcode.respond_info("Jog stopped: %s" % (error,))


def load_config(config):
    return JogButtons(config)
