import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock


# Load the real bus helper without importing native host libraries via mcu.
# Keep the stub local so running this together with other tests is harmless.
class MCUError(Exception):
    pass


MCU_MODULE = types.ModuleType('mcu')
MCU_MODULE.error = MCUError
MCU_MODULE.get_printer_mcu = lambda printer, name: printer.mcus[name]
BUS_PATH = pathlib.Path(__file__).resolve().parents[1] / 'klippy/extras/bus.py'
BUS_SPEC = importlib.util.spec_from_file_location('i2c_bus_under_test', BUS_PATH)
bus = importlib.util.module_from_spec(BUS_SPEC)
with mock.patch.dict(sys.modules, {'mcu': MCU_MODULE}):
    BUS_SPEC.loader.exec_module(bus)


class FakeReactor:
    def __init__(self):
        self.now = 10.
        self.async_callbacks = []

    def monotonic(self):
        return self.now

    def register_async_callback(self, callback):
        self.async_callbacks.append(callback)

    def dispatch_async_callbacks(self):
        callbacks, self.async_callbacks = self.async_callbacks, []
        for callback in callbacks:
            callback(self.now)


class FakeStats:
    def __init__(self):
        self.clients = []
        self.errors = []
        self.skips = []

    def register_client(self, client, name):
        self.clients.append((client, name))

    def note_error(self, client, status):
        self.errors.append((client, status))

    def note_skip(self, client):
        self.skips.append(client)


class FakePins:
    error = ValueError

    def __init__(self):
        self.reserved = []

    def get_pin_resolver(self, name):
        return self

    def reserve_pin(self, pin, bus_name):
        self.reserved.append((pin, bus_name))


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.stats = FakeStats()
        self.pins = FakePins()
        self.shutdowns = []
        self.events = {}
        self.mcus = {}

    def get_reactor(self):
        return self.reactor

    def register_event_handler(self, event, callback):
        self.events.setdefault(event, []).append(callback)

    def lookup_object(self, name):
        if name == 'pins':
            return self.pins
        raise AssertionError('Unexpected object lookup: %s' % name)

    def load_object(self, config, name):
        assert name == 'i2c_stats'
        return self.stats

    def invoke_shutdown(self, message):
        self.shutdowns.append(message)


class FakeQuery:
    def __init__(self):
        self.responses = []
        self.calls = []

    def send(self, args, **kwargs):
        self.calls.append((args, kwargs))
        if self.responses:
            return self.responses.pop(0)
        return {'i2c_bus_status': 'SUCCESS',
                'response': b'\x12\x34'[:args[2]]}


class FakeMCU:
    def __init__(self, constants=None, legacy=False):
        self.printer = FakePrinter()
        self.printer.mcus['mcu'] = self
        self.constants = {'I2C_BUS_RECOVERY': 1} if constants is None else constants
        self.legacy = legacy
        self.query = FakeQuery()
        self.commands = []
        self.callbacks = []
        self.responses = []
        self.oid = 0

    def get_printer(self):
        return self.printer

    def get_name(self):
        return 'mcu'

    def create_oid(self):
        self.oid += 1
        return self.oid

    def add_config_cmd(self, command, **kwargs):
        self.commands.append(command)

    def alloc_command_queue(self):
        return object()

    def register_config_callback(self, callback):
        self.callbacks.append(callback)

    def get_enumerations(self):
        return {'i2c_bus': {'twi0': 0, 'twi1': 1}}

    def get_constants(self):
        return self.constants

    def try_lookup_command(self, command):
        return self.legacy and command.startswith('i2c_read ')

    def lookup_query_command(self, *args, **kwargs):
        return self.query

    def lookup_command(self, *args, **kwargs):
        return self.query

    def register_serial_response(self, *args):
        self.responses.append(args)

    def is_fileoutput(self):
        return False


class FakeConfig:
    def __init__(self, printer, **settings):
        self.printer = printer
        self.settings = settings

    def get_printer(self):
        return self.printer

    def get_name(self):
        return 'ads1x1x laser_adc'

    def get(self, name, default=None):
        return self.settings.get(name, default)

    def getint(self, name, default=None, **kwargs):
        return int(self.get(name, default))


class I2CBusFaultTest(unittest.TestCase):
    def make_client(self, error_tolerant=True, **kwargs):
        mcu = FakeMCU(**kwargs)
        config = FakeConfig(mcu.printer, i2c_bus='twi1')
        client = bus.MCU_I2C_from_config(
            config, default_addr=0x48, error_tolerant=error_tolerant)
        client.build_config()
        return mcu, client

    def test_tolerated_error_is_counted_once_and_discards_response(self):
        mcu, client = self.make_client()
        mcu.query.responses.append(
            {'i2c_bus_status': 'BUS_TIMEOUT', 'response': b'\xde\xad'})
        with self.assertRaisesRegex(bus.I2CError, 'BUS_TIMEOUT'):
            client.i2c_read([0], 2)
        self.assertEqual(mcu.printer.stats.errors, [(client, 'BUS_TIMEOUT')])
        self.assertEqual(mcu.printer.shutdowns, [])
        self.assertEqual(len(mcu.query.calls), 1)

    def test_cooldown_skips_then_success_and_later_failure_resume(self):
        mcu, client = self.make_client()
        mcu.query.responses = [
            {'i2c_bus_status': 'NACK', 'response': b''},
            {'i2c_bus_status': 'SUCCESS', 'response': b'\x12\x34'},
            {'i2c_bus_status': 'BUS_TIMEOUT', 'response': b''}]
        with self.assertRaises(bus.I2CError):
            client.i2c_read([0], 2)
        for eventtime in (10.25, 10.999):
            mcu.printer.reactor.now = eventtime
            with self.assertRaises(bus.I2CError):
                client.i2c_read([0], 2)
        self.assertEqual(len(mcu.query.calls), 1)
        self.assertEqual(mcu.printer.stats.skips, [client, client])
        self.assertEqual(len(mcu.printer.stats.errors), 1)
        mcu.printer.reactor.now = 11.
        self.assertEqual(client.i2c_read([0], 2)['response'], b'\x12\x34')
        with self.assertRaises(bus.I2CError):
            client.i2c_read([0], 2)
        self.assertEqual(len(mcu.query.calls), 3)
        self.assertEqual([status for _, status in mcu.printer.stats.errors],
                         ['NACK', 'BUS_TIMEOUT'])
        self.assertEqual(mcu.printer.shutdowns, [])

    def test_success_with_truncated_or_oversized_data_is_rejected(self):
        for tolerant in (True, False):
            for read_len, payload in ((2, b''), (2, b'\x01'),
                                      (2, b'\x01\x02\x03'), (0, b'\x01')):
                with self.subTest(tolerant=tolerant, read_len=read_len,
                                  payload=payload):
                    mcu, client = self.make_client(error_tolerant=tolerant)
                    mcu.query.responses.append(
                        {'i2c_bus_status': 'SUCCESS', 'response': payload})
                    if tolerant:
                        with self.assertRaisesRegex(bus.I2CError,
                                                    'INVALID_RESPONSE'):
                            client.i2c_transfer([0], read_len)
                        self.assertEqual(mcu.printer.shutdowns, [])
                    else:
                        self.assertIsNone(client.i2c_transfer([0], read_len))
                        self.assertEqual(len(mcu.printer.shutdowns), 1)
                        self.assertIn('INVALID_RESPONSE',
                                      mcu.printer.shutdowns[0])
                    self.assertEqual(mcu.printer.stats.errors,
                                     [(client, 'INVALID_RESPONSE')])
                    self.assertEqual(len(mcu.query.calls), 1)

    def test_normal_client_still_shuts_down_on_reported_error(self):
        mcu, client = self.make_client(error_tolerant=False)
        mcu.query.responses.append(
            {'i2c_bus_status': 'START_READ_NACK', 'response': b''})
        self.assertIsNone(client.i2c_read([0], 2))
        self.assertEqual(len(mcu.printer.shutdowns), 1)
        self.assertIn('START_READ_NACK', mcu.printer.shutdowns[0])
        self.assertEqual(mcu.printer.stats.errors,
                         [(client, 'START_READ_NACK')])

    def test_async_status_is_counted_without_shutdown(self):
        mcu = FakeMCU()
        config = FakeConfig(mcu.printer, i2c_bus='twi1')
        client = bus.MCU_I2C_from_config(
            config, default_addr=0x48, async_write_only=True)
        client.build_config()
        callback = mcu.responses[0][0]
        callback({'i2c_bus_status': 'SUCCESS'})
        callback({'i2c_bus_status': 'NACK'})
        callback({'i2c_bus_status': 'BUS_TIMEOUT'})
        # Receive-thread callbacks must defer all mutable statistics to the
        # reactor; distinct queued statuses must retain their own values.
        self.assertEqual(mcu.printer.stats.errors, [])
        self.assertEqual(len(mcu.printer.reactor.async_callbacks), 2)
        mcu.printer.reactor.dispatch_async_callbacks()
        self.assertEqual(mcu.printer.stats.errors,
                         [(client, 'NACK'), (client, 'BUS_TIMEOUT')])
        self.assertEqual(mcu.printer.shutdowns, [])

    def test_tolerant_write_noack_uses_acknowledged_error_path(self):
        mcu, client = self.make_client()
        mcu.query.responses.append(
            {'i2c_bus_status': 'NACK', 'response': b''})
        with self.assertRaises(bus.I2CError):
            client.i2c_write_noack([1, 2, 3], minclock=123, reqclock=456)
        args, kwargs = mcu.query.calls[0]
        self.assertEqual(args, [client.get_oid(), [1, 2, 3], 0])
        self.assertEqual((kwargs['minclock'], kwargs['reqclock']), (123, 456))
        self.assertEqual(mcu.printer.stats.errors, [(client, 'NACK')])

    def test_tolerance_rejects_unsupported_configurations(self):
        cases = [
            ('missing recovery support', {}, False, None, False),
            ('disabled recovery support', {'I2C_BUS_RECOVERY': 0},
             False, None, False),
            ('old transfer protocol', {'I2C_BUS_RECOVERY': 1},
             True, None, False),
            ('software bus', {'I2C_BUS_RECOVERY': 1},
             False, ('PA18', 'PA17'), False),
            ('unacknowledged writes', {'I2C_BUS_RECOVERY': 1},
             False, None, True),
        ]
        for name, constants, legacy, sw_pins, async_write in cases:
            with self.subTest(name=name):
                mcu = FakeMCU(constants=constants, legacy=legacy)
                client = bus.MCU_I2C(
                    mcu, 'twi1', 0x48, 100000, sw_pins=sw_pins,
                    async_write_only=async_write, error_tolerant=True)
                with self.assertRaises(MCUError):
                    client.build_config()
                self.assertFalse(client._configured)

    def test_from_config_registers_named_client_and_resolves_default_bus(self):
        mcu = FakeMCU(constants={
            'I2C_BUS_RECOVERY': 1, 'BUS_PINS_twi0': 'PA18,PA17'})
        config = FakeConfig(mcu.printer)
        client = bus.MCU_I2C_from_config(
            config, default_addr=0x48, error_tolerant=True)
        self.assertEqual(mcu.printer.stats.clients,
                         [(client, 'ads1x1x laser_adc')])
        self.assertIn(client.build_config, mcu.callbacks)
        self.assertIn(client._handle_connect,
                      mcu.printer.events['klippy:connect'])
        client.build_config()
        self.assertTrue(client._configured)
        self.assertEqual(client.bus, 'twi0')
        self.assertEqual(mcu.printer.stats.clients[0][0].bus, 'twi0')
        self.assertEqual(mcu.printer.pins.reserved,
                         [('PA18', 'twi0'), ('PA17', 'twi0')])
        self.assertIn('i2c_set_bus oid=1 i2c_bus=twi0 rate=100000 address=72',
                      mcu.commands)


if __name__ == '__main__':
    unittest.main()
