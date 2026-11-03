import contextlib
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock

import klippy.extras
from klippy import pins


# Isolate the hardware interface so these sensor tests also run on Windows,
# where the complete MCU transport imports unavailable POSIX modules.
bus_interface = types.ModuleType('klippy.extras.bus')


class I2CError(Exception):
    pass


bus_interface.I2CError = I2CError
bus_interface.MCU_I2C_from_config = mock.Mock()
spec = importlib.util.spec_from_file_location(
    'klippy.extras.ads1x1x_fault_test_target',
    pathlib.Path(__file__).parents[1] / 'klippy' / 'extras' / 'ads1x1x.py')
ads1x1x = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {'pins': pins,
                                  'klippy.extras.bus': bus_interface}):
    with mock.patch.object(klippy.extras, 'bus', bus_interface, create=True):
        spec.loader.exec_module(ads1x1x)


class FakeReactor:
    NOW = 0.

    def __init__(self):
        self.now = 10.

    def monotonic(self):
        return self.now

    def pause(self, until):
        self.now = until

    def mutex(self):
        return contextlib.nullcontext()

    def register_timer(self, callback, waketime):
        return object()


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.shutdowns = []

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name):
        return mock.Mock()

    def add_object(self, name, obj):
        pass

    def register_event_handler(self, name, callback):
        pass

    def invoke_shutdown(self, reason):
        self.shutdowns.append(reason)


class FakeConfig:
    def __init__(self, printer, error_tolerant):
        self.printer = printer
        self.error_tolerant = error_tolerant

    def get_printer(self):
        return self.printer

    def get_name(self):
        return 'ads1x1x test'

    def get(self, name, default=None):
        return default

    def getchoice(self, name, choices, default=None):
        return choices['ADS1115' if name == 'chip' else default]

    def getfloat(self, name, default=None, **kwargs):
        return default

    def getboolean(self, name, default=None):
        return self.error_tolerant


class FakeI2C:
    def __init__(self):
        self.mcu = mock.Mock()
        self.mcu.estimated_print_time.side_effect = lambda now: now + 1000.
        self.error_tolerant = True
        self.fail_at = None
        self.ready = True
        self.raw = 16384
        self.writes = []

    def get_mcu(self):
        return self.mcu

    def i2c_write(self, data):
        self.writes.append(data)
        if self.fail_at == 'write':
            raise I2CError('BUS_TIMEOUT')

    def i2c_read(self, reg, count):
        is_config = reg == [ads1x1x.ADS1X1X_REG_POINTER['CONFIG']]
        if self.fail_at == ('ready' if is_config else 'conversion'):
            raise I2CError('NACK')
        value = (0x8000 if self.ready else 0) if is_config else self.raw
        return {'response': bytes((value >> 8, value & 0xff))}


class ADS1X1XI2CFaultTest(unittest.TestCase):
    def setUp(self):
        self.printer = FakePrinter()
        self.i2c = FakeI2C()
        bus_interface.MCU_I2C_from_config.reset_mock()
        bus_interface.MCU_I2C_from_config.return_value = self.i2c
        self.config = FakeConfig(self.printer, True)
        self.chip = ads1x1x.ADS1X1X_chip(self.config)
        self.pin = self.chip.setup_pin('adc', {'pin': 'AIN1'})
        self.samples = []
        self.pin.setup_adc_callback(self.samples.append)
        self.pin.setup_adc_sample(.3, minval=.1, maxval=.9,
                                  range_check_count=1)
        self.pin._handle_connect()
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(ads1x1x.logging, 'warning').start()

    def sample(self):
        eventtime = self.printer.reactor.monotonic()
        nexttime = self.pin._process_sample(eventtime)
        self.assertEqual(nexttime, eventtime + self.pin.report_time)
        self.printer.reactor.now = max(nexttime, self.printer.reactor.now)

    def test_tolerant_policy_is_explicit_and_defaults_to_strict(self):
        self.assertTrue(bus_interface.MCU_I2C_from_config.call_args.kwargs[
            'error_tolerant'])
        config = FakeConfig(self.printer, False)
        ads1x1x.ADS1X1X_chip(config)
        self.assertFalse(bus_interface.MCU_I2C_from_config.call_args.kwargs[
            'error_tolerant'])

    def test_failures_at_every_transfer_stage_skip_samples_and_recover(self):
        for stage in ('write', 'ready', 'conversion'):
            with self.subTest(stage=stage):
                previous = self.pin.get_last_value()
                previous_count = len(self.samples)
                self.i2c.fail_at = stage
                for _ in range(10):
                    self.sample()
                self.assertEqual(self.pin.get_last_value(), previous)
                self.assertEqual(len(self.samples), previous_count)
                self.assertEqual(self.pin.invalid_count, 0)
                self.assertEqual(self.printer.shutdowns, [])
                self.i2c.fail_at = None
                self.sample()
                self.assertEqual(len(self.samples), previous_count + 1)
                value = self.samples[-1][0][1]
                self.assertAlmostEqual(value, 16384 / 32767. * 4.096 / 3.3)

    def test_status_shows_absent_and_aging_samples(self):
        status = self.chip.get_status(self.printer.reactor.monotonic())
        self.assertIsNone(status['channels']['AIN1']['last_sample_age'])
        self.sample()
        self.i2c.fail_at = 'write'
        before = self.pin.get_last_value()
        for _ in range(5):
            self.sample()
        age = self.chip.get_status(self.printer.reactor.monotonic())[
            'channels']['AIN1']['last_sample_age']
        self.assertGreaterEqual(age, 1.5)
        self.assertEqual(self.pin.get_last_value(), before)
        self.i2c.fail_at = None
        self.sample()
        self.assertGreater(self.pin.get_last_value()[0], before[0])

    def test_bus_gaps_do_not_clear_real_range_violations(self):
        self.i2c.raw = 32767
        self.sample()
        self.assertEqual(self.pin.invalid_count, 1)
        self.assertEqual(self.printer.shutdowns, [])
        self.i2c.fail_at = 'conversion'
        for _ in range(10):
            self.sample()
        self.assertEqual(self.pin.invalid_count, 1)
        self.assertEqual(self.printer.shutdowns, [])
        self.i2c.fail_at = None
        self.sample()
        self.assertEqual(self.pin.invalid_count, 2)
        self.assertEqual(len(self.printer.shutdowns), 1)

    def test_conversion_timeout_keeps_existing_invalid_sample_protection(self):
        self.i2c.ready = False
        self.sample()
        self.sample()
        self.assertEqual(self.pin.invalid_count, 2)
        self.assertEqual(len(self.printer.shutdowns), 1)
        self.assertEqual(self.samples, [])

    def test_reset_failure_has_one_attempt_and_no_duplicate_log(self):
        self.i2c.fail_at = 'write'
        with mock.patch.object(ads1x1x.logging, 'exception') as log:
            self.chip._handle_connect()
        self.assertEqual(self.i2c.writes, [[0x06, 0x00, 0x00]])
        log.assert_not_called()


if __name__ == '__main__':
    unittest.main()
