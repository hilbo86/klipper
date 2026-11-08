# Central tool interlocks for the laser. Distributed under GNU GPLv3.
import math
import re
from collections import deque


TOOLS = ('BLUE_LASER', 'IR_LASER', 'DREMEL')
ID_WINDOWS = (
    # Main ID measured at 80%; preserve the original +/-3 point tolerance.
    (77.00, 83.00, 'BLUE_LASER'), (56.09, 62.09, 'IR_LASER'),
    (14.58, 20.58, 'DREMEL'), (65.79, 71.79, 'BLUE_LASER+IR_LASER'),
    (47.00, 53.00, 'BLUE_LASER+DREMEL'),
    (38.82, 44.82, 'IR_LASER+DREMEL'), (28.21, 34.21, 'NONE'))
OUTPUTS = {'BLUE_LASER': 'laser', 'IR_LASER': 'aux_tool',
           'DREMEL': 'aux_tool'}
TEMPERATURES = {'BLUE_LASER': 'temperature_sensor Laser',
                'IR_LASER': 'temperature_sensor aux_tool',
                'DREMEL': 'temperature_sensor aux_tool'}
BOUNDARY_AXES = {'X': (0, 1, 'X_MAX'), 'Y': (1, 1, 'Y_MAX'),
                 'Z': (2, -1, 'Z_MIN')}


def classify_id(value, previous=None, hysteresis=0.):
    for low, high, name in ID_WINDOWS:
        margin = hysteresis if name == previous else 0.
        if low - margin < value < high + margin:
            return name
    return 'UNKNOWN'


def file_tool(stream):
    """Require a bounded, explicit first command, before even a modal move."""
    position = stream.tell()
    try:
        stream.seek(0)
        header = stream.read(65536)
        if isinstance(header, bytes):
            header = header.decode('utf-8-sig')
        for line in header.lstrip('\ufeff').splitlines():
            line = re.sub(r'\([^)]*\)', '', line).split(';', 1)[0].strip()
            if not line:
                continue
            match = re.fullmatch(r'JOB_START\s+TOOL=(BLUE_LASER|IR_LASER|DREMEL)',
                                 line, re.IGNORECASE)
            if match:
                return match.group(1).upper()
            break
    finally:
        stream.seek(position)
    raise ValueError('Erster Jobbefehl muss JOB_START TOOL=<Werkzeug> sein.')


class LaserToolManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.id_name = config.get('id_input', 'analog_input ID')
        self.stable_samples = config.getint('stable_samples', 3, minval=2)
        self.id_filter_size = config.getint(
            'id_filter_samples', 5, minval=1, maxval=9)
        if self.id_filter_size % 2 != 1:
            raise config.error('id_filter_samples muss ungerade sein.')
        self.id_hysteresis = config.getfloat(
            'id_hysteresis', .5, minval=0., maxval=1.)
        if not math.isfinite(self.id_hysteresis):
            raise config.error('id_hysteresis muss endlich sein.')
        self.max_age = config.getfloat('max_sample_age', .6, above=0.)
        self.interval = config.getfloat('poll_interval', .1, above=0.)
        self.temp_max_age = config.getfloat('temperature_max_age', 1., above=0.)
        self.positioning_speed = config.getfloat('positioning_speed', 20., above=0.)
        self.dremel_sensor = config.getboolean('dremel_has_sensor', False)
        self.calibration_max_power = config.getfloat(
            'calibration_max_power', .1, above=0., maxval=1.)
        self.calibration_max_duration = config.getfloat(
            'calibration_max_duration', 10., above=0., maxval=30.)
        self.calibration_deadline = None
        self.calibration_idle = self.printer.load_object(
            config, 'gcode_macro').load_template(
                config, 'calibration_idle_gcode', '_TOOLS_OFF')
        self.profiles = {}
        for tool in TOOLS:
            prefix = tool.lower() + '_'
            calibrated = config.getboolean(prefix + 'calibrated', False)
            bounds_calibrated = config.getboolean(
                prefix + 'bounds_calibrated', calibrated)
            # No unmeasured Aux bounds or offsets are inferred.
            values = {key: config.getfloat(prefix + key, None)
                      for key in ('x_min', 'x_max', 'y_min', 'y_max',
                                  'offset_x', 'offset_y')}
            bounds = [values[key] for key in
                      ('x_min', 'x_max', 'y_min', 'y_max')]
            if bounds_calibrated and (None in bounds
                                      or not all(math.isfinite(v) for v in bounds)
                                      or values['x_min'] >= values['x_max']
                                      or values['y_min'] >= values['y_max']):
                raise config.error('Unvollstaendige Fahrgrenzen fuer ' + tool)
            if calibrated and (not bounds_calibrated or None in values.values()
                               or not all(math.isfinite(v) for v in values.values())):
                raise config.error('Unvollstaendige Geometrie fuer ' + tool)
            values['calibrated'] = calibrated
            values['bounds_calibrated'] = bounds_calibrated
            values['production_ready'] = config.getboolean(
                prefix + 'production_ready', tool != 'DREMEL')
            self.profiles[tool] = values
        self.detected = self.candidate = 'UNKNOWN'
        self.motion_tool = None
        self.samples = 0
        self.last_sample = None
        self.id_value = self.id_age = None
        self.id_filtered_value = None
        self.id_samples = deque(maxlen=self.id_filter_size)
        self.job_tool = None
        self.job_file = None
        self.preflight_tool = None
        self.granted = False
        self.preparing = False
        self.stopped = False
        self.fault = None
        self.calibrating = False
        self.calibration_valid = False
        self.calibration_xy = None
        self.saved_idle = None
        self.boundary = None
        self.boundary_moving = False
        self.boundary_limits = {
            axis: config.getfloat('boundary_' + axis.lower()
                                  + ('_min' if direction < 0 else '_max'), None)
            for axis, (index, direction, button) in BOUNDARY_AXES.items()}
        self.homing = False
        self.applied_offset = (0., 0.)
        self.resume_checked = False
        self.cooling_requested = {'water_pump': False, 'heat_exchanger': False}
        for command in ('JOB_START', '_TOOL_ARM', '_TOOL_DISARM',
                        '_TOOL_END', '_TOOL_RESUME_CHECK', 'TOOL_STATUS',
                        'TOOL_CALIBRATION_BEGIN', 'TOOL_CALIBRATION_HOLD',
                        'TOOL_CALIBRATION_ARM', 'TOOL_CALIBRATION_END',
                        'TOOL_BOUNDS_BEGIN', 'TOOL_BOUNDS_MOVE', 'TOOL_BOUNDS_END'):
            self.gcode.register_command(command, getattr(self, 'cmd_' + command),
                                        desc='Laser Werkzeugverwaltung')
        self.printer.register_event_handler('klippy:ready', self._ready)
        self.printer.register_event_handler('klippy:shutdown', self._shutdown)
        self.printer.register_event_handler('stepper_enable:motor_off',
                                            self._motor_off)
        self.printer.register_event_handler('homing:home_rails_begin',
                                            self._home_begin)
        self.printer.register_event_handler('homing:home_rails_end',
                                            self._home_end)
        self.printer.register_event_handler('gcode:command_error',
                                            self._command_error)

    def _ready(self):
        self.id_sensor = self.printer.lookup_object(self.id_name)
        self.toolhead = self.printer.lookup_object('toolhead')
        self.sd = self.printer.lookup_object('virtual_sdcard')
        self.pause = self.printer.lookup_object('pause_resume')
        state = self.toolhead.get_status(self.reactor.monotonic())
        for axis, (index, direction, button) in BOUNDARY_AXES.items():
            native = state['axis_maximum' if direction > 0 else 'axis_minimum'][index]
            limit = self.boundary_limits[axis]
            if limit is None:
                self.boundary_limits[axis] = native
            elif not math.isfinite(limit) or direction * (limit - native) < 0:
                raise self.printer.config_error('Ungueltige Mess-Suchgrenze fuer ' + axis)
        for tool, profile in self.profiles.items():
            if profile['bounds_calibrated']:
                for axis, index in (('x', 0), ('y', 1)):
                    if not (state['axis_minimum'][index]
                            <= profile[axis + '_min']
                            < profile[axis + '_max']
                            <= state['axis_maximum'][index]):
                        raise self.printer.config_error(
                            'Profil ausserhalb Maschinenfahrbereich: ' + tool)
        kin = self.toolhead.get_kinematics()
        self.kin = kin
        self.base_check_move = kin.check_move
        kin.check_move = self.check_move
        self.base_sd_resume = self.sd.do_resume
        self.sd.do_resume = self._sd_resume
        self.base_sd_pause = self.sd.do_pause
        self.sd.do_pause = self._sd_pause
        self.base_sd_cancel = self.sd.do_cancel
        self.sd.do_cancel = self._sd_cancel
        self.base_sd_reset = self.sd._reset_file
        self.sd._reset_file = self._sd_reset
        self.gcode.register_command('M25', None)
        self.gcode.register_command('M25', self._pause_command,
                                    desc='Pause with tool power disabled')
        # Guard the actual mux handlers, including the renamed _SET_PIN path.
        # Public macros, the display and GPIO controls all reach these handlers.
        handlers = self.gcode.mux_commands['SET_PIN'][1]
        for pin in ('laser', 'aux_tool', 'tool_enable', 'Steckdose_4',
                    'air_assist'):
            original = handlers[pin]
            handlers[pin] = self._pin_handler(pin, original)
        fan_handlers = self.gcode.mux_commands['SET_FAN_SPEED'][1]
        for name in self.cooling_requested:
            fan_handlers[name] = self._cooling_handler(name, fan_handlers[name])
        self.reactor.register_timer(self._poll, self.reactor.NOW)

    def _cooling_handler(self, name, original):
        def guarded(gcmd):
            if gcmd.get('TEMPLATE', None) is not None:
                raise gcmd.error('Main-Kuehlkreis erlaubt keine ungesicherten Templates.')
            speed = gcmd.get_float('SPEED', minval=0., maxval=1.)
            if not math.isfinite(speed):
                raise gcmd.error('Ungueltiger Kuehlkreis-Sollwert.')
            if self.granted and (self.job_tool or self.detected) == 'BLUE_LASER' and not speed:
                raise gcmd.error('Main-Werkzeug vor Abschalten der Kuehlung ausschalten.')
            result = original(gcmd)
            self.cooling_requested[name] = bool(speed)
            return result
        return guarded

    def _pin_handler(self, pin, original):
        def guarded(gcmd):
            if gcmd.get('TEMPLATE', None) is not None:
                raise gcmd.error('PWM-Templates am Werkzeug gesperrt.')
            value = gcmd.get_float('VALUE', minval=0., maxval=1.)
            self.check_output(pin, value)
            return original(gcmd)
        return guarded

    def _read_adc(self, adc, eventtime):
        read_time, raw = adc.get_last_value()
        age = adc.get_mcu().estimated_print_time(eventtime) - read_time
        return read_time, raw, age

    def _update_id(self, eventtime):
        timestamp, raw, age = self._read_adc(self.id_sensor.mcu_adc, eventtime)
        value = raw * self.id_sensor.scale + self.id_sensor.offset
        self.id_value = value if math.isfinite(value) else None
        self.id_age = max(age, 0.) if math.isfinite(age) else None
        fresh = timestamp > 0 and -.1 <= age <= self.max_age
        if not fresh or (self.last_sample is not None
                         and timestamp < self.last_sample):
            self.id_samples.clear()
            self.id_filtered_value = None
            self.candidate = self.detected = 'STALE'
            self.samples = 0
            return
        if timestamp == self.last_sample:
            return
        if (self.last_sample is not None
                and timestamp - self.last_sample > self.max_age):
            self.id_samples.clear()
            self.candidate = 'UNKNOWN'
            self.detected = 'UNSTABLE'
            self.samples = 0
        self.last_sample = timestamp
        if self.id_value is None or not 0. < raw < 1.:
            self.id_samples.clear()
            self.id_filtered_value = None
            self.candidate = self.detected = 'UNKNOWN'
            self.samples = 0
            return

        # Retain an established class near its boundaries, but acquire new
        # classes only inside the acquisition windows. Never average a real tool
        # change into a third resistor ID or delay revocation behind the filter.
        held = self.detected
        anchor = self.candidate
        if anchor in ('UNKNOWN', 'STALE'):
            anchor = classify_id(value)
        if anchor != 'UNKNOWN' and (
                classify_id(value, anchor, self.id_hysteresis) != anchor
                or any(classify_id(sample, anchor, self.id_hysteresis) != anchor
                       for sample in self.id_samples)):
            self.id_samples.clear()
        self.id_samples.append(value)
        ordered = sorted(self.id_samples)
        self.id_filtered_value = ordered[len(ordered) // 2]
        candidate = classify_id(self.id_filtered_value, held, self.id_hysteresis)
        if candidate != self.candidate:
            self.candidate, self.samples = candidate, 0
        if len(self.id_samples) < self.id_filter_size:
            self.samples = 0
        else:
            self.samples += 1
        self.detected = 'UNSTABLE'
        if self.samples >= self.stable_samples:
            self.detected = candidate
            if candidate in TOOLS:
                self.motion_tool = candidate

    def _temperature_valid(self, tool, eventtime):
        if tool == 'DREMEL' and not self.dremel_sensor:
            return True
        sensor = self.printer.lookup_object(TEMPERATURES[tool])
        timestamp, raw, age = self._read_adc(sensor.sensor.mcu_adc, eventtime)
        temperature = sensor.last_temp
        return (timestamp > 0 and -.1 <= age <= self.temp_max_age
                and .001 < raw < .999 and math.isfinite(temperature)
                and sensor.min_temp <= temperature <= sensor.max_temp)

    def _compressor(self):
        return self.printer.lookup_object('gcode_macro COMPRESSOR_VARS').variables

    def _reasons(self, tool=None, reference=False, calibration=False):
        eventtime = self.reactor.monotonic()
        self._update_id(eventtime)
        tool = tool or self.detected
        reasons = []
        if self.detected not in TOOLS:
            reasons.append('Werkzeug-ID: ' + self.detected)
        elif tool != self.detected:
            reasons.append('Jobwerkzeug passt nicht zur Werkzeug-ID')
        if tool not in TOOLS:
            return reasons
        profile = self.profiles[tool]
        if not calibration:
            if not profile['calibrated']:
                reasons.append('Werkzeuggeometrie nicht kalibriert')
            if not profile['production_ready']:
                reasons.append('Produktivfreigabe/CAM fehlt')
        if not self._temperature_valid(tool, eventtime):
            reasons.append('Aktive Werkzeugtemperatur ungueltig/veraltet')
        lid = self.printer.lookup_object('gcode_button deckelklappe')
        if lid.get_status(eventtime)['state'] != 'RELEASED':
            reasons.append('Deckel offen')
        if tool == 'BLUE_LASER':
            v = self._compressor()
            if (v['emergency_lock'] or v['is_cooldown']
                    or v['is_preparing_stop'] or v['stop_requested']
                    or v['needs_maintenance']):
                reasons.append('Kompressor gesperrt/Wartung erforderlich')
        if reference:
            state = self.toolhead.get_status(eventtime)
            if state['homed_axes'] != 'xyz':
                reasons.append('Achsen nicht referenziert')
            if tool == 'IR_LASER' and not self._z_at_top(state):
                reasons.append('IR: Z muss oben referenziert und gehalten sein')
            if not self._inside_geometry(tool, self.toolhead.get_position()):
                reasons.append('Aktuelle Position ausserhalb Werkzeugfahrbereich')
        if self.fault:
            reasons.append(self.fault)
        return reasons

    def _require(self, tool, reference=False, calibration=False):
        reasons = self._reasons(tool, reference, calibration)
        if reasons:
            raise self.gcode.error('Werkzeug gesperrt: ' + '; '.join(reasons))

    def _require_periphery(self, tool):
        if tool == 'BLUE_LASER' and (not all(self.cooling_requested.values())
                                    or not self._compressor()['air_requested']):
            raise self.gcode.error('Main-Kuehlung oder Air Assist nicht vorbereitet.')

    def _z_at_top(self, state):
        return ('z' in state['homed_axes'] and abs(
            self.toolhead.get_position()[2] - state['axis_maximum'][2]) < .01)

    def _inside_geometry(self, tool, position):
        p = self.profiles[tool]
        return not p['bounds_calibrated'] or all(
            p[axis + '_min'] - 1.e-6 <= position[index] <= p[axis + '_max'] + 1.e-6
            for axis, index in (('x', 0), ('y', 1)))

    def _poll(self, eventtime):
        if self.stopped:
            return self.reactor.NEVER
        self._update_id(eventtime)
        if self.boundary:
            reason = self._boundary_reason()
            if reason:
                self._trip('Grenzvermessung: ' + reason)
                return self.reactor.NEVER
        if (self.granted and self.calibration_deadline is not None
                and eventtime >= self.calibration_deadline):
            self._trip('Kalibrierfreigabe abgelaufen; Bezug neu herstellen')
        if self.job_tool or self.granted:
            tool = self.job_tool or self.detected
            if self.detected != tool:
                self._trip('Werkzeugverlust, Konflikt oder geaenderte ID')
            elif not self._temperature_valid(tool, eventtime):
                self._trip('Aktive Werkzeugtemperatur ungueltig/veraltet')
            elif not self.preparing:
                state = self.toolhead.get_status(eventtime)
                if state['homed_axes'] != 'xyz':
                    self._trip('Achsenreferenz verloren')
                elif tool == 'IR_LASER' and not self._z_at_top(state):
                    self._trip('IR: Z nicht mehr oben gehalten')
                elif not self._inside_geometry(tool, self.toolhead.get_position()):
                    self._trip('Position ausserhalb Werkzeugfahrbereich')
                elif self.granted and tool == 'BLUE_LASER' and (
                        not all(self.cooling_requested.values())
                        or not self._compressor()['air_requested']):
                    self._trip('Main-Kuehlung oder Air Assist ausgefallen')
        if self.stopped:
            return self.reactor.NEVER
        if self.job_tool and self.sd.current_file is None and not self.sd.is_active():
            # A missing JOB_FINISH may never leave a power grant behind at EOF.
            self.gcode.run_script('job_finish')
        return eventtime + self.interval

    def _trip(self, message):
        # MCU shutdown discards already queued power/motion as well. A normal
        # G-code zero behind the lookahead queue cannot guarantee this.
        self.fault = message
        self.granted = False
        self.printer.invoke_shutdown('Werkzeug: ' + message)

    def _shutdown(self):
        self.stopped = True
        self.granted = False
        self.calibration_valid = False
        self._clear_boundary()

    def _motor_off(self):
        self.calibration_valid = False
        if self.job_tool or self.granted or self.boundary:
            self._trip('Referenz durch Motorabschaltung verloren')

    def _home_begin(self, homing_state, rails):
        if self.boundary or self.granted or (self.job_tool and not self.preparing) or (self.calibrating
                            and homing_state.get_axes() != [2]):
            raise self.gcode.error('Homing waehrend Freigabe/Kalibrierung gesperrt.')
        self.homing = True

    def _home_end(self, homing_state, rails):
        self.homing = False

    def _command_error(self):
        self.homing = False
        self.resume_checked = False

    def _sd_resume(self):
        if self.calibrating or self.boundary:
            raise self.gcode.error('Dateijob waehrend Kalibrierung gesperrt.')
        if self.sd.current_file is None:
            raise self.gcode.error('Keine Jobdatei geladen.')
        if self.sd.file_position == 0 and not self.job_tool:
            try:
                tool = file_tool(self.sd.current_file)
            except (ValueError, UnicodeError) as exc:
                raise self.gcode.error(str(exc))
            if tool == 'BLUE_LASER':
                self.gcode.run_script_from_command('_COMPRESSOR_INIT')
            self._require(tool)
            self.preflight_tool = tool
        elif (not self.resume_checked or not self.job_tool
              or self.sd.file_path() != self.job_file):
            raise self.gcode.error('Dateifortsetzung nur ueber geprueftes RESUME.')
        else:
            self._require(self.job_tool, reference=True)
        self.resume_checked = False
        return self.base_sd_resume()

    def _sd_cancel(self):
        self.cmd__TOOL_END(None)
        return self.base_sd_cancel()

    def _sd_pause(self):
        result = self.base_sd_pause()
        if self.granted:
            self.gcode.run_script_from_command('_TOOLS_OFF')
        return result

    def _sd_reset(self):
        if self.job_tool or self.pause.is_paused:
            raise self.gcode.error('Aktiven/pausierten Job zuerst mit CANCEL_PRINT beenden.')
        self.preflight_tool = None
        return self.base_sd_reset()

    def _pause_command(self, gcmd):
        self.gcode.run_script_from_command('PAUSE')

    def check_output(self, pin, value):
        if not math.isfinite(value):
            raise self.gcode.error('Ungueltiger Leistungswert.')
        if not value:
            return
        if pin in ('Steckdose_4', 'air_assist'):
            self._update_id(self.reactor.monotonic())
            if self.detected != 'BLUE_LASER':
                raise self.gcode.error('Air Assist nur fuer BLUE_LASER verfuegbar.')
            return
        tool = self.job_tool or (self.detected if self.calibrating else None)
        if not self.granted or not tool or self.pause.is_paused:
            raise self.gcode.error('Keine Werkzeugfreigabe fuer diesen Job.')
        self._require(tool, reference=True, calibration=self.calibrating)
        self._require_periphery(tool)
        if pin != 'tool_enable' and pin != OUTPUTS[tool]:
            raise self.gcode.error('Falscher Leistungsausgang fuer ' + tool)
        if self.calibrating and pin != 'tool_enable' and value > self.calibration_max_power:
            raise self.gcode.error('Leistung oberhalb Kalibrierlimit.')

    def check_move(self, move):
        if self.boundary:
            if not self.boundary_moving:
                raise move.move_error('Grenzvermessung: nur TOOL_BOUNDS_MOVE erlaubt')
            b = self.boundary
            if (any(abs(move.axes_d[i]) > 1.e-6 for i in range(3) if i != b['index'])
                    or abs(move.axes_d[b['index']]) > 1.000001
                    or not b['minimum'] - 1.e-6 <= move.end_pos[b['index']] <= b['maximum'] + 1.e-6):
                raise move.move_error('Grenzvermessung: unzulaessige Bewegung')
            return self.base_check_move(move)
        if not self.homing:
            tool = self.job_tool or (self.candidate if self.candidate in TOOLS
                                     else self.motion_tool)
            if tool == 'IR_LASER' and move.axes_d[2] < -1.e-8:
                raise move.move_error('IR-Profil: Z-Abwaertsbewegung gesperrt')
            if self.calibrating and self.calibration_xy is not None:
                if any(abs(move.end_pos[i] - self.calibration_xy[i]) > 1.e-6
                       for i in (0, 1)):
                    raise move.move_error('Kalibrierung: XY-Bezug wird gehalten')
            if tool in self.profiles:
                p = self.profiles[tool]
                if p['bounds_calibrated']:
                    if not self._inside_geometry(tool, move.end_pos):
                        raise move.move_error('Werkzeugfahrbereich ' + tool)
                elif not self.calibrating and any(move.axes_d[:2]):
                    raise move.move_error('Aux-Geometrie fehlt: Kalibriermodus erforderlich')
        return self.base_check_move(move)

    def _apply_offset(self, tool):
        p = self.profiles[tool]
        # Klipper machine = gcode + base_position; correction is -geometry.
        correction = (-p['offset_x'], -p['offset_y'])
        self.gcode.run_script_from_command(
            'SET_GCODE_OFFSET X_ADJUST=%.6f Y_ADJUST=%.6f MOVE=0'
            % (correction[0] - self.applied_offset[0],
               correction[1] - self.applied_offset[1]))
        self.applied_offset = correction

    def cmd_JOB_START(self, gcmd):
        tool = gcmd.get('TOOL').upper()
        if self.calibrating or self.boundary or self.job_tool or self.pause.is_paused:
            raise gcmd.error('Werkzeugwechsel im laufenden/pausierten Job gesperrt.')
        if not self.sd.is_cmd_from_sd() or tool != self.preflight_tool:
            raise gcmd.error('JOB_START nur als erster gepruefter Dateibefehl.')
        self._require(tool)
        self.job_tool, self.job_file = tool, self.sd.file_path()
        self.preparing = True
        self.preflight_tool = None
        self._apply_offset(tool)
        self.gcode.run_script_from_command('job_init\nG28')
        # A measured lower bound may lie beyond the physical homing origin.
        # Enter the permitted rectangle with power off before granting power.
        p = self.profiles[tool]
        current = self.toolhead.get_position()
        target = [max(p[axis + '_min'], min(p[axis + '_max'], current[index]))
                  for axis, index in (('x', 0), ('y', 1))]
        if target != list(current[:2]):
            self.toolhead.manual_move(target + [None], self.positioning_speed)
        self.gcode.run_script_from_command('job_preparation')

    def cmd__TOOL_ARM(self, gcmd):
        if not self.job_tool:
            raise self.gcode.error('JOB_START TOOL=<Werkzeug> fehlt.')
        self._require(self.job_tool, reference=True)
        self._require_periphery(self.job_tool)
        self.granted = True
        self.preparing = False

    def cmd__TOOL_DISARM(self, gcmd):
        if self.calibrating and self.granted and self.detected == 'BLUE_LASER':
            self.gcode.run_script_from_command(
                'UPDATE_DELAYED_GCODE ID=main_cooling_stop DURATION=90')
        self.granted = False
        self.resume_checked = False
        self.calibration_deadline = None

    def cmd__TOOL_END(self, gcmd):
        self.gcode.run_script_from_command('_TOOLS_OFF')
        self.toolhead.wait_moves()
        self.job_tool = self.job_file = self.preflight_tool = None
        self.preparing = False

    def cmd__TOOL_RESUME_CHECK(self, gcmd):
        if not self.job_tool:
            raise self.gcode.error('Kein passender Werkzeugjob fuer RESUME.')
        self._require(self.job_tool, reference=True)
        self._require_periphery(self.job_tool)
        self.resume_checked = True

    def cmd_TOOL_STATUS(self, gcmd):
        gcmd.respond_info(str(self.get_status(self.reactor.monotonic())))

    def cmd_TOOL_CALIBRATION_BEGIN(self, gcmd):
        if self.boundary:
            raise gcmd.error('Grenzvermessung zuerst beenden.')
        if self.job_tool or self.sd.is_active() or self.pause.is_paused:
            raise gcmd.error('Kalibrierung nur ohne laufenden/pausierten Job.')
        if self.calibrating:
            raise gcmd.error('Kalibrierung bereits aktiv.')
        if self.toolhead.get_status(self.reactor.monotonic())['homed_axes'] != 'xyz':
            raise gcmd.error('Vor der Kalibrierung alle Achsen referenzieren.')
        self.toolhead.wait_moves()
        self.gcode.run_script_from_command('_TOOLS_OFF')
        if any(self.applied_offset):
            self.gcode.run_script_from_command(
                'SET_GCODE_OFFSET X_ADJUST=%.6f Y_ADJUST=%.6f MOVE=0'
                % (-self.applied_offset[0], -self.applied_offset[1]))
            self.applied_offset = (0., 0.)
        idle = self.printer.lookup_object('idle_timeout')
        self.saved_idle = idle.idle_timeout
        # Replace the timeout action, rather than merely extending its duration.
        # This keeps XY energised even during a long mechanical change.
        self.saved_idle_gcode = idle.idle_gcode
        idle.idle_gcode = self.calibration_idle
        self.calibrating = self.calibration_valid = True

    def cmd_TOOL_CALIBRATION_HOLD(self, gcmd):
        if not self.calibrating or not self.calibration_valid:
            raise gcmd.error('Keine gueltige Kalibrierung.')
        self.gcode.run_script_from_command('_TOOLS_OFF')
        self.toolhead.wait_moves()
        self.calibration_xy = list(self.toolhead.get_position()[:2])

    def cmd_TOOL_CALIBRATION_ARM(self, gcmd):
        tool = gcmd.get('TOOL').upper()
        if not self.calibrating or not self.calibration_valid:
            raise gcmd.error('Kalibrierbezug ungueltig; neu beginnen.')
        self._require(tool, reference=True, calibration=True)
        if tool == 'BLUE_LASER':
            self.gcode.run_script_from_command(
                '_COMPRESSOR_INIT\n_COMPRESSOR_CHECK_READY MAINTENANCE=1\n'
                'START_CLEANING\nAIR_ASSIST_ON')
        self._require_periphery(tool)
        self.granted = True
        self.calibration_deadline = (self.reactor.monotonic()
                                     + self.calibration_max_duration)
        self.gcode.run_script_from_command('SET_PIN PIN=tool_enable VALUE=1')

    def cmd_TOOL_CALIBRATION_END(self, gcmd):
        self.gcode.run_script_from_command('_TOOLS_OFF')
        if self.calibrating:
            self.printer.lookup_object('idle_timeout').idle_gcode = self.saved_idle_gcode
            self.gcode.run_script_from_command('SET_IDLE_TIMEOUT TIMEOUT=%.6f'
                                               % self.saved_idle)
        self.calibrating = self.calibration_valid = False
        self.calibration_xy = None

    def _boundary_reason(self):
        b = self.boundary
        if self.detected != b['tool']:
            return 'Werkzeug-ID geaendert/ungueltig'
        state = self.toolhead.get_status(self.reactor.monotonic())
        if state['homed_axes'] != 'xyz':
            return 'Achsenreferenz verloren'
        if b['tool'] == 'IR_LASER' and not self._z_at_top(state):
            return 'IR: Z muss oben gehalten sein'
        if self.reactor.monotonic() > b['deadline']:
            return 'Messsitzung abgelaufen'
        for name in ('pwm_tool laser', 'pwm_tool aux_tool', 'output_pin tool_enable'):
            if self.printer.lookup_object(name).get_status(
                    self.reactor.monotonic())['value']:
                return 'Werkzeugleistung muss ausgeschaltet bleiben'
        position = self.toolhead.get_position()
        if not b['minimum'] - 1.e-6 <= position[b['index']] <= b['maximum'] + 1.e-6:
            return 'Messachse ausserhalb Suchbereich'
        if any(abs(position[i] - b['origin'][i]) > 1.e-6
               for i in range(3) if i != b['index']):
            return 'Andere Achse wurde bewegt'

    def cmd_TOOL_BOUNDS_BEGIN(self, gcmd):
        if (self.boundary or self.calibrating or self.job_tool
                or self.sd.is_active() or self.pause.is_paused):
            raise gcmd.error('Grenzvermessung nur ohne Job oder andere Kalibrierung.')
        tool, axis = gcmd.get('TOOL').upper(), gcmd.get('AXIS').upper()
        if tool not in TOOLS or axis not in BOUNDARY_AXES:
            raise gcmd.error('TOOL und AXIS=X/Y/Z erforderlich.')
        if tool == 'IR_LASER' and axis == 'Z':
            raise gcmd.error('IR-Laser: keine Z-Grenzvermessung.')
        self._update_id(self.reactor.monotonic())
        state = self.toolhead.get_status(self.reactor.monotonic())
        if self.detected != tool or state['homed_axes'] != 'xyz':
            raise gcmd.error('Passende stabile ID und XYZ-Referenz erforderlich.')
        if tool == 'IR_LASER' and not self._z_at_top(state):
            raise gcmd.error('IR: Z muss oben gehalten sein.')
        if (not hasattr(self.kin, 'update_limits')
                or getattr(self.kin, 'dual_carriage_axis', None) is not None):
            raise gcmd.error('Grenzvermessung erfordert kartesische Kinematik.')
        index, direction, button = BOUNDARY_AXES[axis]
        limit = gcmd.get_float('LIMIT')
        self.toolhead.wait_moves()
        position = list(self.toolhead.get_position())
        if not math.isfinite(limit) or direction * (limit - position[index]) <= 0:
            raise gcmd.error('Endliche Suchgrenze in Suchrichtung erforderlich.')
        if direction * (limit - self.boundary_limits[axis]) > 1.e-6:
            raise gcmd.error('Suchgrenze ausserhalb konfiguriertem Messbereich.')
        self.gcode.run_script_from_command('_TOOLS_OFF')
        self.toolhead.wait_moves()
        idle = self.printer.lookup_object('idle_timeout')
        self.boundary = {
            'tool': tool, 'axis': axis, 'index': index, 'direction': direction,
            'button': button, 'limit': limit, 'origin': position,
            'minimum': min(position[index], limit),
            'maximum': max(position[index], limit),
            'original_limits': tuple(self.kin.limits[index]),
            'idle_gcode': idle.idle_gcode,
            'deadline': self.reactor.monotonic() + 60.}
        idle.idle_gcode = self.calibration_idle

    def cmd_TOOL_BOUNDS_MOVE(self, gcmd):
        if not self.boundary:
            raise gcmd.error('Keine aktive Grenzvermessung.')
        self._update_id(self.reactor.monotonic())
        reason = self._boundary_reason()
        if reason:
            raise gcmd.error('Grenzvermessung: ' + reason)
        b = self.boundary
        target = gcmd.get_float('POSITION')
        speed = gcmd.get_float('SPEED', 5., minval=.1, maxval=10.)
        position = list(self.toolhead.get_position())
        delta = target - position[b['index']]
        if (not math.isfinite(target) or not math.isfinite(speed)
                or not b['minimum'] - 1.e-6 <= target <= b['maximum'] + 1.e-6
                or abs(delta) > 1.000001):
            raise gcmd.error('Grenzvermessung: maximal 1 mm je Schritt innerhalb Suchbereich.')
        pressed = self.printer.lookup_object('gcode_button ' + b['button']).get_status(
            self.reactor.monotonic())['state'] == 'PRESSED'
        if pressed and delta * b['direction'] > 1.e-6:
            raise gcmd.error('Grenztaster ausgeloest: nur Rueckzug erlaubt.')
        limits = self.kin.limits[b['index']]
        extended = (min(limits[0], b['minimum']), max(limits[1], b['maximum']))
        target_position = list(position)
        target_position[b['index']] = target
        self.boundary_moving = True
        try:
            self.kin.update_limits(b['index'], extended)
            self.toolhead.manual_move(target_position, speed)
            self.toolhead.wait_moves()
            b['deadline'] = self.reactor.monotonic() + 60.
        finally:
            self.kin.update_limits(b['index'], limits)
            self.boundary_moving = False

    def _clear_boundary(self):
        if self.boundary:
            self.printer.lookup_object('idle_timeout').idle_gcode = self.boundary['idle_gcode']
            self.boundary = None

    def cmd_TOOL_BOUNDS_END(self, gcmd):
        self.gcode.run_script_from_command('_TOOLS_OFF')
        self.toolhead.wait_moves()
        if self.boundary:
            b = self.boundary
            position = self.toolhead.get_position()[b['index']]
            if not b['original_limits'][0] <= position <= b['original_limits'][1]:
                self.kin.clear_homing_state('xyz')
            self._clear_boundary()

    def get_status(self, eventtime):
        self._update_id(eventtime)
        tool = self.detected if self.detected in TOOLS else None
        profile = self.profiles.get(tool)
        reasons = self._reasons(self.job_tool or tool)
        temperature = ([TEMPERATURES[tool]] if tool and
                       (tool != 'DREMEL' or self.dremel_sensor) else [])
        return {'detected_tool': self.detected, 'id_value': self.id_value,
                'id_filtered_value': self.id_filtered_value,
                'id_filter_samples': len(self.id_samples),
                'id_filter_size': self.id_filter_size,
                'id_sample_age': self.id_age, 'stable_samples': self.samples,
                'requested_tool': self.job_tool, 'power_granted': self.granted,
                'profile_calibrated': bool(profile and profile['calibrated']),
                'profile_bounds_calibrated': bool(profile and profile['bounds_calibrated']),
                'geometry': profile, 'blocked_reasons': reasons,
                'temperature_sensors': temperature,
                'valid_temperature_sensors': temperature if tool and
                    self._temperature_valid(tool, eventtime) else [],
                'applied_offset': self.applied_offset,
                'ir_z_locked': (self.job_tool or self.motion_tool) == 'IR_LASER',
                'z_at_top': self._z_at_top(self.toolhead.get_status(eventtime)),
                'calibrating': self.calibrating,
                'calibration_valid': self.calibration_valid,
                'boundary_measurement': ({key: self.boundary[key] for key in
                    ('tool', 'axis', 'limit', 'minimum', 'maximum')}
                    if self.boundary else None),
                'boundary_search_limits': dict(self.boundary_limits),
                'available_controls': ([OUTPUTS[tool], 'crosshair', 'Y_Laser']
                    + (['air_assist', 'Steckdose_4', 'water_pump',
                        'heat_exchanger', 'tank_licht', 'uv_disinfector']
                       if tool == 'BLUE_LASER' else [])) if tool else []}


def load_config(config):
    return LaserToolManager(config)
