"""Standalone Klipper doubles, also used by configuration macro integration tests."""
import importlib.util
import io
import shlex
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'laser_tool_manager', ROOT / 'klippy/extras/laser_tool_manager.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MacroError(Exception):
    pass


class Status:
    def __init__(self, data):
        self.data = data

    def get_status(self, eventtime):
        return self.data


class GCommand:
    def __init__(self, host, params):
        self.host, self.params = host, params

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_float(self, key, default=None, minval=None, maxval=None):
        value = self.get(key, default)
        if value is None:
            return None
        value = float(value)
        if ((minval is not None and value < minval)
                or (maxval is not None and value > maxval)):
            self.host.error('Value outside range')
        return value

    def error(self, message):
        self.host.error(message)

    def respond_info(self, message):
        self.host.info(message)


class Config:
    def __init__(self, host, values=None):
        self.host = host
        # Synthetic test geometry; no dependency on a printer configuration.
        self.values = {
            'blue_laser_calibrated': True,
            'blue_laser_x_min': 0., 'blue_laser_x_max': 266.,
            'blue_laser_y_min': 0., 'blue_laser_y_max': 292.,
            'blue_laser_offset_x': 0., 'blue_laser_offset_y': 0.,
        }
        self.values.update(values or {})

    def get_printer(self):
        return self.host

    def get(self, key, default=None):
        return self.values.get(key, default)

    def getfloat(self, key, default=None, **kwargs):
        value = self.get(key, default)
        return float(value) if value is not None else None

    def getint(self, key, default=None, **kwargs):
        return int(self.get(key, default))

    def getboolean(self, key, default=False):
        return str(self.get(key, default)).lower() == 'true'

    def error(self, message):
        self.host.error(message)


class ADC:
    def __init__(self, host, value):
        self.host, self.value = host, value
        self.timestamp = None

    def get_last_value(self):
        timestamp = (self.estimated_print_time(self.host.now)
                     if self.timestamp is None else self.timestamp)
        return timestamp, self.value()

    def get_mcu(self):
        return self

    def estimated_print_time(self, eventtime):
        return eventtime + 10


class Toolhead:
    def __init__(self, host):
        self.host = host
        self.data = {'estimated_print_time': 0., 'homed_axes': 'xyz',
                     'axis_minimum': (0., 0., 0.),
                     'axis_maximum': (266., 292., 38.)}
        self.position = [0., 0., 38., 0.]
        self.kin = SimpleNamespace(check_move=lambda move: None)

    def get_status(self, eventtime):
        return self.data

    def get_position(self):
        return self.position

    def get_kinematics(self):
        return self.kin

    def wait_moves(self):
        self.host.events.append(('wait_moves', None))

    def manual_move(self, position, speed):
        end = list(self.position)
        for index, value in enumerate(position):
            if value is not None:
                end[index] = value
        delta = [b-a for a, b in zip(self.position, end)]
        self.kin.check_move(SimpleNamespace(
            end_pos=end, axes_d=delta, move_error=self.host.error))
        assert not self.host.tool_manager.granted
        self.position = end


class SD:
    def __init__(self, host):
        self.host = host
        self.current_file = None
        self.file_position = 0
        self.active = self.from_sd = False

    def file_path(self):
        return self.host.print_stats['filename'] if self.current_file else None

    def is_active(self):
        return self.active

    def is_cmd_from_sd(self):
        return self.from_sd

    def do_resume(self):
        self.active = True

    def do_cancel(self):
        self.active = False
        self.current_file = None

    def do_pause(self):
        self.active = False

    def _reset_file(self):
        self.current_file = None
        self.file_position = 0


class GCode:
    def __init__(self, host):
        self.host = host
        self.commands = {}
        self.mux_commands = {'SET_PIN': ('PIN', {}), 'SET_FAN_SPEED': ('FAN', {})}
        for name in ('laser', 'aux_tool', 'tool_enable', 'air_assist', 'Steckdose_4'):
            self.mux_commands['SET_PIN'][1][name] = self.set_pin
        for name in ('water_pump', 'heat_exchanger'):
            self.mux_commands['SET_FAN_SPEED'][1][name] = self.set_fan

    def set_pin(self, gcmd):
        self.host.pins[gcmd.get('PIN')]['value'] = gcmd.get_float('VALUE')

    def set_fan(self, gcmd):
        self.host.fans[gcmd.get('FAN')] = gcmd.get_float('SPEED')

    def register_command(self, command, callback, **kwargs):
        if callback is None:
            return self.commands.pop(command, None)
        self.commands[command] = callback

    def run_script_from_command(self, script):
        for line in script.splitlines():
            self.host.run(line)

    run_script = run_script_from_command

    def error(self, message):
        self.host.error(message)


def attach(host, tool_options=None):
    gcode = GCode(host)
    host.backend_gcode = gcode
    host.handlers = {}
    host.shutdown_message = None
    host.objects['gcode'] = gcode
    host.objects['gcode_macro'] = SimpleNamespace(
        load_template=lambda *args: args[-1])
    host.objects['toolhead'] = Toolhead(host)
    host.objects['idle_timeout'].idle_timeout = 600.
    host.objects['idle_timeout'].idle_gcode = 'M84'
    host.objects['pause_resume'].is_paused = False
    sd = SD(host)
    host.objects['virtual_sdcard'] = sd
    host.sd = sd
    host.objects['analog_input ID'].mcu_adc = ADC(
        host, lambda: host.objects['analog_input ID'].data['value'] / 100)
    host.objects['analog_input ID'].scale = 100.
    host.objects['analog_input ID'].offset = 0.
    for name in ('Laser', 'aux_tool'):
        host.objects['temperature_sensor ' + name] = SimpleNamespace(
            sensor=SimpleNamespace(mcu_adc=ADC(host, lambda: .2)),
            last_temp=25., min_temp=-273.15, max_temp=99999999.9)
    host.register_event_handler = lambda name, cb: host.handlers.setdefault(name, []).append(cb)
    host.register_timer = lambda *args: None
    host.NOW = 0.
    host.NEVER = float('inf')
    host.load_object = lambda cfg, name: host.objects[name]
    host.invoke_shutdown = lambda message: shutdown(host, message)
    host.objects['gcode_macro COMPRESSOR_VARS'].variables = host.v
    host.tool_manager = MODULE.LaserToolManager(Config(host, tool_options))
    host.objects['laser_tool_manager'] = host.tool_manager
    host.tool_manager._ready()
    # Warm up real reports before the timing tests' zero point.
    manager = host.tool_manager
    start = host.now
    for remaining in reversed(range(manager.id_filter_size
                                    + manager.stable_samples - 1)):
        host.now = start - remaining * .1
        manager._update_id(host.now)
    host.now = start


def shutdown(host, message):
    host.shutdown_message = message
    # Model the MCU shutdown guarantee: outstanding power requests are discarded.
    for pin in host.pins.values():
        pin['value'] = 0.
    for callback in host.handlers.get('klippy:shutdown', []):
        callback()


def start_job(host, tool='BLUE_LASER'):
    host.sd.current_file = io.StringIO('JOB_START TOOL=' + tool + '\nM3 S128\n')
    host.sd.file_position = 0
    host.sd.from_sd = True
    host.sd.do_resume()
    host.run('JOB_START TOOL=' + tool)


def select(machine, value):
    machine.objects['analog_input ID'].data['value'] = value
    manager = machine.tool_manager
    for _ in range(manager.id_filter_size + manager.stable_samples - 1):
        machine.now += .1
        manager._update_id(machine.now)


def measured_profile(machine, tool):
    machine.tool_manager.profiles[tool].update(
        calibrated=True, x_min=5., x_max=250., y_min=2., y_max=240.,
        offset_x=3., offset_y=-7.)


def move(start, end):
    def error(message):
        return MacroError(message)
    return SimpleNamespace(end_pos=end, axes_d=[b-a for a, b in zip(start, end)],
                           move_error=error)


class Machine:
    """Module-only host: macro callbacks model the external Klipper interface.

    This host does not reproduce the printer's compressor or job macro logic.
    The configuration repository executes those real Jinja macros separately.
    """
    def __init__(self, tool_options=None, id_value=80.):
        self.now = 0.
        self.events = []
        self.variables = {'COMPRESSOR_VARS': {
            'emergency_lock': 0, 'is_cooldown': 0, 'is_preparing_stop': 0,
            'stop_requested': 0, 'needs_maintenance': 0, 'air_requested': 0}}
        self.objects = {
            'gcode_macro COMPRESSOR_VARS': Status(self.v),
            'print_stats': Status({'state': 'printing',
                                   'filename': 'laser.gcode'}),
            'idle_timeout': Status({'state': 'Printing'}),
            'pause_resume': Status({'is_paused': False}),
            'analog_input ID': Status({'value': id_value}),
            'gcode_button deckelklappe': Status({'state': 'RELEASED'}),
        }
        self.pins = {name: {'value': 0.} for name in (
            'laser', 'aux_tool', 'tool_enable', 'Steckdose_4', 'air_assist')}
        self.fans = {}
        attach(self, tool_options)
        # Periphery readiness is fixture input to module tests. Its actual
        # preparation is checked against the real configuration macros.
        if self.tool_manager.detected == 'BLUE_LASER':
            self.run('START_CLEANING')
            self.run('AIR_ASSIST_ON')

    @property
    def v(self):
        return self.variables['COMPRESSOR_VARS']

    @property
    def print_stats(self):
        return self.objects['print_stats'].data

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def get_reactor(self):
        return self

    def monotonic(self):
        return self.now

    def info(self, message):
        self.events.append(('info', message))
        return ''

    def error(self, message):
        raise MacroError(message)

    def power(self, value):
        tool = self.tool_manager.job_tool or self.tool_manager.detected
        self.run('_SET_PIN PIN=%s VALUE=%.12f' % (MODULE.OUTPUTS[tool], value))

    def job(self):
        start_job(self)
        self.power(128/255)

    def run(self, command):
        words = shlex.split(command)
        if not words:
            return
        name = words[0].upper()
        params = dict(word.split('=', 1) for word in words[1:])
        self.events.append((name, params))
        gcmd = GCommand(self, params)
        if name in self.backend_gcode.commands:
            return self.backend_gcode.commands[name](gcmd)
        if name in ('SET_PIN', '_SET_PIN'):
            return self.backend_gcode.mux_commands['SET_PIN'][1][params['PIN']](gcmd)
        if name == 'SET_FAN_SPEED':
            return self.backend_gcode.mux_commands[name][1][params['FAN']](gcmd)
        if name == '_TOOLS_OFF':
            self.run('_TOOL_DISARM')
            for pin in ('tool_enable', 'laser', 'aux_tool'):
                self.run('_SET_PIN PIN=%s VALUE=0' % pin)
        elif name == 'JOB_PREPARATION':
            self.run('_TOOL_ARM')
            self.run('_SET_PIN PIN=tool_enable VALUE=1')
        elif name == 'JOB_FINISH':
            self.run('_TOOL_END')
        elif name == 'START_CLEANING':
            self.run('SET_FAN_SPEED FAN=water_pump SPEED=1')
            self.run('SET_FAN_SPEED FAN=heat_exchanger SPEED=.8')
        elif name == 'STOP_CLEANING':
            self.run('SET_FAN_SPEED FAN=water_pump SPEED=0')
            self.run('SET_FAN_SPEED FAN=heat_exchanger SPEED=0')
        elif name == 'AIR_ASSIST_ON':
            self.run('_SET_PIN PIN=Steckdose_4 VALUE=1')
            self.v['air_requested'] = 1
        elif name == 'PAUSE':
            self.run('_TOOLS_OFF')
            self.sd.do_pause()
            self.objects['pause_resume'].is_paused = True
            self.print_stats['state'] = 'paused'
            self.v['air_requested'] = 0
        elif name == 'RESUME':
            if self.tool_manager.job_tool == 'BLUE_LASER':
                self.run('START_CLEANING')
                self.run('AIR_ASSIST_ON')
            self.run('_TOOL_RESUME_CHECK')
            self.run('_TOOL_ARM')
            self.sd.do_resume()
            self.objects['pause_resume'].is_paused = False
            self.print_stats['state'] = 'printing'
        elif name not in ('JOB_INIT', 'G28', '_COMPRESSOR_INIT',
                          '_COMPRESSOR_CHECK_READY', 'SET_GCODE_OFFSET',
                          'SET_IDLE_TIMEOUT', 'UPDATE_DELAYED_GCODE'):
            raise AssertionError('Unmodelled host callback: ' + command)
