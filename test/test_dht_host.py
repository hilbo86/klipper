"""DHT host adapter against the real pin allocator and message codec."""
import contextlib
import json
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'klippy'))
from extras import dht_sensor
import gcode
import msgproto
import pins


class Config:
    error = ValueError

    def __init__(self, printer, name='temperature_sensor chamber', **options):
        self.printer, self.name = printer, name
        self.options = dict(sensor_type='DHT22', sensor_pin='nano_b:PD2',
                            dht_auxiliary_mcu=True)
        self.options.update(options)

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.name

    def get(self, key, default=None):
        return self.options.get(key, default)

    def getboolean(self, key, default=None):
        return self.get(key, default)

    def getfloat(self, key, default, **limits):
        value = float(self.get(key, default))
        if (value <= limits.get('above', -float('inf'))
            or value < limits.get('minval', -float('inf'))
            or value > limits.get('maxval', float('inf'))):
            raise ValueError(key)
        return value


class MCU:
    def __init__(self):
        self.oid = 0
        self.commands, self.callbacks, self.responses = [], [], {}
        self.diagnostic_response = None
        self.diagnostic_requests = []
        self.constants = dict(DHT_VERSION=1, MCU='atmega328p',
                              CLOCK_FREQ=16000000)

    def create_oid(self):
        self.oid += 1
        return self.oid

    def register_config_callback(self, callback):
        self.callbacks.append(callback)

    def register_serial_response(self, callback, fmt, oid):
        self.responses[oid] = callback

    def get_constants(self):
        return self.constants

    def add_config_cmd(self, command, **flags):
        self.commands.append((command, flags))

    def get_query_slot(self, oid):
        return self.seconds_to_clock(1.) + oid

    def seconds_to_clock(self, seconds):
        return int(seconds * 16000000)

    def clock32_to_clock64(self, clock):
        return clock

    def clock_to_print_time(self, clock):
        return clock / 16000000. + 10.

    def estimated_print_time(self, eventtime):
        return eventtime - 90.

    def check_valid_response(self, fmt):
        return fmt == dht_sensor.DIAGNOSTIC_FORMAT \
            and self.diagnostic_response is not None

    def lookup_query_command(self, request, response, oid):
        mcu = self

        class Query:
            def send(self, args):
                mcu.diagnostic_requests.append((request, response, oid, args))
                return mcu.diagnostic_response

        return Query()


class Reactor:
    now = 102.

    def monotonic(self):
        return self.now

    def mutex(self):
        return contextlib.nullcontext()


class Printer:
    config_error = ValueError

    def __init__(self):
        self.reactor = Reactor()
        self.pins = pins.PrinterPins()
        self.objects = {'pins': self.pins}
        self.gcode = gcode.GCodeDispatch(self)
        self.objects['gcode'] = self.gcode
        self.factories = {}
        self.a, self.b = MCU(), MCU()
        self.pins.register_chip('nano_a', self.a)
        self.pins.register_chip('nano_b', self.b)

    def lookup_object(self, name):
        return self.objects[name]

    def get_reactor(self):
        return self.reactor

    def get_start_args(self):
        return {}

    def register_event_handler(self, event, callback):
        pass

    def get_state_message(self):
        return 'Printer is ready', 'ready'

    def send_event(self, event):
        pass

    def lookup_objects(self, prefix):
        return [(key, value) for key, value in self.objects.items()
                if key.startswith(prefix + ' ')]

    def add_object(self, name, obj):
        if name in self.objects:
            raise ValueError('duplicate object')
        self.objects[name] = obj

    def load_object(self, config, name):
        assert name == 'heaters'
        return self

    def add_sensor_factory(self, name, factory):
        self.factories[name] = factory


class TestDHTHost(unittest.TestCase):
    def setUp(self):
        self.printer = Printer()
        self.sensor = dht_sensor.DHTSensor(Config(self.printer))
        self.measurements = []
        self.sensor.setup_callback(lambda *args: self.measurements.append(args))

    def response(self, status=0, clock=16000000, **fields):
        result = dict(clock=clock, status=status, temp10=-101, humidity10=658)
        result.update(fields)
        self.sensor._handle_result(result)

    def test_registration_pin_allocation_and_mcu_messages(self):
        dht_sensor.load_config(Config(self.printer))
        self.assertEqual(set(self.printer.factories), {'DHT11', 'DHT22'})
        self.assertIs(self.printer.objects['dht_status chamber'], self.sensor)
        with self.assertRaises(pins.error):
            self.printer.pins.lookup_pin('nano_b:PD2')
        self.sensor._build_config()
        commands = self.printer.b.commands
        self.assertEqual(commands[0], (
            'config_dht oid=1 pin=PD2 sensor_type=22 pull_up=0', {}))
        self.assertIn(('query_dht oid=1 clock=48000001 rest_ticks=80000000',
                       {'is_init': True}), commands)
        self.assertIn(('query_dht oid=1 clock=0 rest_ticks=0',
                       {'on_restart': True}), commands)
        self.assertEqual(self.sensor.get_report_time_delta(), 5.)

    def test_signed_message_codec_and_timestamp(self):
        fmt = msgproto.MessageFormat([1], dht_sensor.RESULT_FORMAT)
        encoded = bytes(fmt.encode([1, 16000000, 0, -101, 658]))
        decoded, end = fmt.parse(encoded, 0)
        self.assertEqual(end, len(encoded))
        self.printer.b.responses[1](decoded)
        self.assertEqual(self.measurements, [(11., -10.1)])
        status = self.sensor.get_status(102.)
        self.assertEqual(status['temperature'], -10.1)
        self.assertEqual(status['humidity'], 65.8)
        self.assertEqual(status['age_seconds'], 1.)
        self.assertTrue(status['valid'])

    def test_initial_and_stale_values_are_null(self):
        status = self.sensor.get_status(100.)
        self.assertFalse(status['valid'])
        self.assertIsNone(status['temperature'])
        self.assertIsNone(status['humidity'])
        self.assertIsNone(status['age_seconds'])
        self.response()
        self.assertTrue(self.sensor.get_status(115.)['valid'])
        status = self.sensor.get_status(116.)
        self.assertFalse(status['valid'])
        self.assertIsNone(status['temperature'])
        self.assertIsNone(status['humidity'])
        self.assertEqual(status['age_seconds'], 15.)

    def test_errors_ignore_payload_and_do_not_refresh_age(self):
        self.response()
        for status in range(1, 7):
            self.sensor._handle_result(dict(status=status,
                                            clock=16000000 + status))
        self.sensor._handle_result(dict(status=99, clock=16000007))
        status = self.sensor.get_status(102.)
        self.assertEqual(status['ok_count'], 1)
        self.assertEqual(status['timeout_errors'], 2)
        self.assertEqual(status['checksum_errors'], 1)
        self.assertEqual(status['protocol_errors'], 3)
        self.assertEqual(status['range_errors'], 1)
        self.assertEqual(status['last_error'], 'UNKNOWN_STATUS')
        self.assertTrue(status['valid'])
        self.assertEqual(status['age_seconds'], 1.)
        self.assertEqual(len(self.measurements), 1)
        self.assertFalse(self.sensor.get_status(116.)['valid'])

    def test_duplicates_out_of_order_and_delayed_packets(self):
        self.response()
        self.response(status=4)
        self.response(clock=10000000)
        self.assertEqual(len(self.measurements), 1)
        self.assertEqual(self.sensor.ok_count, 1)
        self.assertEqual(self.sensor.checksum_errors, 0)
        self.assertFalse(self.sensor.get_status(1000.)['valid'])
        self.response(clock=32000000)
        self.assertEqual(self.measurements[-1][0], 12.)
        self.assertFalse(self.sensor.get_status(1000.)['valid'])

    def test_clock_epoch_is_converted_by_mcu_before_deduplication(self):
        self.printer.b.clock32_to_clock64 = lambda clock: 0x100000000 + clock
        self.response(clock=16000000)
        self.assertAlmostEqual(self.measurements[0][0],
                               0x100000000 / 16000000. + 11.)

    def test_two_mcus_keep_oid_type_status_separate(self):
        other = dht_sensor.DHTSensor(Config(
            self.printer, 'temperature_sensor exchanger',
            sensor_type='DHT11', sensor_pin='nano_a:PB5'))
        self.assertEqual(other._oid, self.sensor._oid)
        other._build_config()
        self.assertIn('sensor_type=11', self.printer.a.commands[0][0])
        self.printer.a.responses[1](dict(clock=16000000, status=0,
                                       temp10=246, humidity10=553))
        self.assertEqual(other.get_status(101.)['humidity'], 55.3)
        self.assertEqual(self.sensor.get_status(101.)['ok_count'], 0)
        self.response()
        self.assertEqual(other.get_status(101.)['temperature'], 24.6)

    def test_heaters_period_and_second_sensor_rejected(self):
        for name in ('extruder', 'extruder1', 'heater_bed',
                     'heater_generic test', 'temperature_fan test'):
            with self.subTest(name=name), self.assertRaisesRegex(
                    ValueError, 'only be used'):
                dht_sensor.DHTSensor(Config(Printer(), name))
        for options in (dict(dht_auxiliary_mcu=False),
                        dict(dht_report_time=2.), dict(dht_report_time=121.),
                        dict(sensor_type='DHT11', dht_report_time=1.),
                        dict(dht_max_age=5.), dict(sensor_pin='~nano_b:PD2')):
            with self.subTest(options=options), self.assertRaises(ValueError):
                dht_sensor.DHTSensor(Config(Printer(), **options))
        with self.assertRaisesRegex(ValueError, 'one sensor per MCU'):
            dht_sensor.DHTSensor(Config(
                self.printer, 'temperature_sensor extra',
                sensor_pin='nano_b:PB5'))

    def test_build_capability_and_platform_rejected(self):
        for constants in ({}, dict(DHT_VERSION=1, MCU='atmega2560',
                                   CLOCK_FREQ=16000000),
                          dict(DHT_VERSION=1, MCU='atmega328p',
                               CLOCK_FREQ=8000000)):
            with self.subTest(constants=constants):
                self.printer.b.constants = constants
                with self.assertRaisesRegex(ValueError, 'ATmega328P'):
                    self.sensor._build_config()

    def test_spi_and_pin_alias_conflicts_use_standard_resolver(self):
        sensor = dht_sensor.DHTSensor(Config(
            self.printer, 'temperature_sensor exchanger',
            sensor_type='DHT11', sensor_pin='nano_a:PB5'))
        sensor._build_config()
        resolver = self.printer.pins.get_pin_resolver('nano_a')
        resolver.reserve_pin('PB5', 'spi')
        with self.assertRaisesRegex(pins.error, 'reserved for spi'):
            resolver.update_command(self.printer.a.commands[0][0])
        resolver = pins.PinResolver()
        resolver.alias_pin('DHT_PIN', 'PB5')
        resolver.update_command('config_dht pin=DHT_PIN')
        with self.assertRaisesRegex(pins.error, 'is an alias'):
            resolver.update_command('config_digital_out pin=PB5')

    def test_range_errors_never_call_temperature_callback(self):
        self.sensor.setup_minmax(-5., 40.)
        self.response()
        self.assertEqual(self.measurements, [])
        self.response(clock=32000000, temp10=250, humidity10=1001)
        self.assertEqual(self.measurements, [])
        self.response(clock=48000000, temp10=250)
        self.assertEqual(self.measurements, [(13., 25.)])
        self.assertEqual(self.sensor.range_errors, 2)

    def test_console_query_reports_live_status_without_sampling(self):
        output = []
        dispatch = self.printer.gcode
        dispatch.register_output_handler(output.append)
        dispatch._handle_ready()

        def query():
            output.clear()
            dispatch.run_script_from_command('QUERY_DHT SENSOR=chamber')
            report = '\n'.join(output)
            self.assertIn('DHT chamber:', report)
            return json.loads(report[report.index('{'):])

        self.assertEqual(query(), self.sensor.get_status(102.))
        self.assertIn('QUERY_DHT', dispatch.get_command_help())
        self.response()
        before_commands = list(self.printer.b.commands)
        self.assertEqual(query(), self.sensor.get_status(102.))
        self.assertEqual(self.printer.b.commands, before_commands)
        self.printer.reactor.now = 116.
        status = query()
        self.assertFalse(status['valid'])
        self.assertIsNone(status['humidity'])
        self.assertEqual(status['ok_count'], 1)
        self.assertEqual(status['last_error'], 'OK')

    def test_console_query_selects_sensor_and_rejects_unknown_name(self):
        dht_sensor.DHTSensor(Config(
            self.printer, 'temperature_sensor exchanger',
            sensor_type='DHT11', sensor_pin='nano_a:PB5'))
        self.printer.a.responses[1](dict(clock=16000000, status=0,
                                       temp10=246, humidity10=553))
        output = []
        dispatch = self.printer.gcode
        dispatch.register_output_handler(output.append)
        dispatch._handle_ready()
        output.clear()
        dispatch.run_script_from_command('QUERY_DHT SENSOR=exchanger')
        report = '\n'.join(output)
        self.assertIn('DHT exchanger:', report)
        self.assertEqual(json.loads(report[report.index('{'):])['humidity'],
                         55.3)
        with self.assertLogs(level='WARNING'), \
                self.assertRaises(gcode.CommandError):
            dispatch.run_script_from_command('QUERY_DHT SENSOR=missing')

    def test_pulse_diagnostics_decode_and_query_without_resampling(self):
        fmt = msgproto.MessageFormat([1], dht_sensor.DIAGNOSTIC_FORMAT)
        encoded = bytes(fmt.encode([1, 16000000, 1, 5, 17, 132, 0]))
        params, end = fmt.parse(encoded, 0)
        self.assertEqual(end, len(encoded))
        self.printer.b.diagnostic_response = params
        self.response()
        output = []
        dispatch = self.printer.gcode
        dispatch.register_output_handler(output.append)
        dispatch._handle_ready()
        output.clear()
        dispatch.run_script_from_command(
            'QUERY_DHT SENSOR=chamber DIAGNOSTICS=1')
        report = '\n'.join(output)
        status = json.loads(report[report.index('{'):])
        self.assertEqual(status['last_pulse_error'], {
            'phase': 'BIT_HIGH', 'bits_received': 17, 'duration_us': 8.25,
            'level': 0, 'measurement_clock': 16000000})
        self.assertEqual(status['ok_count'], 1)
        self.assertEqual(self.printer.b.commands, [])
        self.assertEqual(self.printer.b.diagnostic_requests, [(
            'query_dht_diagnostics oid=%c', dht_sensor.DIAGNOSTIC_FORMAT,
            1, [1])])

    def test_pulse_diagnostics_report_no_previous_error(self):
        self.printer.b.diagnostic_response = dict(present=0)
        output = []
        dispatch = self.printer.gcode
        dispatch.register_output_handler(output.append)
        dispatch._handle_ready()
        output.clear()
        dispatch.run_script_from_command(
            'QUERY_DHT SENSOR=chamber DIAGNOSTICS=1')
        report = '\n'.join(output)
        self.assertIsNone(json.loads(report[report.index('{'):])
                          ['last_pulse_error'])

    def test_old_firmware_keeps_normal_query_and_rejects_diagnostics(self):
        dispatch = self.printer.gcode
        dispatch._handle_ready()
        dispatch.run_script_from_command('QUERY_DHT SENSOR=chamber')
        with self.assertLogs(level='WARNING'), self.assertRaisesRegex(
                gcode.CommandError, 'updated Nano firmware'):
            dispatch.run_script_from_command(
                'QUERY_DHT SENSOR=chamber DIAGNOSTICS=1')
        self.assertEqual(self.printer.b.diagnostic_requests, [])


if __name__ == '__main__':
    unittest.main()
