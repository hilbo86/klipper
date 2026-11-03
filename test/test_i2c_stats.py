import unittest
from unittest import mock

from klippy.extras import i2c_stats


class FakeReactor:
    now = 0.

    def monotonic(self):
        return self.now


class FakeGCode:
    def __init__(self):
        self.messages = []

    def register_command(self, name, callback, **kwargs):
        self.name = name
        self.callback = callback

    def respond_info(self, message, log=True):
        self.messages.append((message, log))


class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.gcode = FakeGCode()

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name):
        assert name == 'gcode'
        return self.gcode


class FakeConfig:
    def __init__(self):
        self.printer = FakePrinter()

    def get_printer(self):
        return self.printer


class FakeMCU:
    def __init__(self, name):
        self.name = name

    def get_name(self):
        return self.name


class FakeClient:
    def __init__(self, mcu='mcu', oid=1, bus='twi1', address=0x48):
        self.mcu = FakeMCU(mcu)
        self.oid = oid
        self.bus = bus
        self.address = address

    def get_mcu(self):
        return self.mcu

    def get_oid(self):
        return self.oid

    def get_i2c_address(self):
        return self.address


class FakeGcmd:
    def respond_info(self, message):
        self.message = message


class I2CStatsTest(unittest.TestCase):
    def setUp(self):
        self.config = FakeConfig()
        self.stats = i2c_stats.load_config(self.config)
        self.reactor = self.config.printer.reactor
        self.client = FakeClient()
        self.stats.register_client(self.client, 'ads1x1x laser_adc')

    def snapshot(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.now
        return self.stats.get_status(eventtime)

    def client_snapshot(self, eventtime=None):
        return self.snapshot(eventtime)['clients']['mcu:1']

    def test_registration_and_live_metadata(self):
        self.assertEqual(self.config.printer.gcode.name, 'I2C_ERRORS')
        snapshot = self.client_snapshot()
        self.assertEqual(snapshot['name'], 'ads1x1x laser_adc')
        self.assertEqual(snapshot['address'], 0x48)
        self.assertEqual(snapshot['errors_last_hour'], 0)
        self.assertIsNone(snapshot['last_error_time'])
        self.client.bus = 'twi0'
        self.assertEqual(self.client_snapshot()['bus'], 'twi0')
        self.stats.register_client(self.client, 'ads1x1x laser_adc')
        self.assertEqual(len(self.stats.clients), 1)

    @mock.patch('klippy.extras.i2c_stats.logging.warning')
    def test_multiple_clients_and_error_types(self, warning):
        other = FakeClient(mcu='aux', oid=1, address=0x40)
        self.stats.register_client(other, 'display status')
        self.reactor.now = 100.25
        self.stats.note_error(self.client, 'BUS_TIMEOUT')
        self.stats.note_error(self.client, 'NACK')
        self.stats.note_error(other, 'START_NACK')
        snapshot = self.snapshot()
        self.assertEqual(snapshot['errors_last_hour'], 3)
        self.assertEqual(snapshot['errors_total'], 3)
        self.assertEqual(snapshot['clients']['mcu:1']['by_status_last_hour'],
                         {'BUS_TIMEOUT': 1, 'NACK': 1})
        self.assertEqual(snapshot['clients']['aux:1']['errors_total'], 1)
        self.assertEqual(snapshot['clients']['mcu:1']['last_error'], 'NACK')
        self.assertEqual(snapshot['clients']['mcu:1']['last_error_time'],
                         100.25)

    @mock.patch('klippy.extras.i2c_stats.logging.warning')
    def test_hour_expiry_and_lifetime_total(self, warning):
        self.reactor.now = 10.75
        self.stats.note_error(self.client, 'NACK')
        self.reactor.now = 11.25
        self.stats.note_error(self.client, 'BUS_TIMEOUT')
        self.assertEqual(self.client_snapshot(3609.99)['errors_last_hour'], 2)
        self.assertEqual(self.client_snapshot(3610.)['errors_last_hour'], 1)
        snapshot = self.client_snapshot(3611.)
        self.assertEqual(snapshot['errors_last_hour'], 0)
        self.assertEqual(snapshot['by_status_last_hour'], {})
        self.assertEqual(snapshot['errors_total'], 2)
        self.assertEqual(snapshot['last_error'], 'BUS_TIMEOUT')

    @mock.patch('klippy.extras.i2c_stats.logging.warning')
    def test_error_storm_counts_every_failure_with_bounded_storage(self,
                                                                  warning):
        self.reactor.now = 50.
        for unused in range(10000):
            self.stats.note_error(self.client, 'BUS_TIMEOUT')
        self.assertEqual(self.client_snapshot()['errors_last_hour'], 10000)
        self.assertEqual(len(self.stats.clients['mcu:1'].buckets), 1)
        self.assertEqual(warning.call_count, 1)
        for second in range(51, 5051):
            self.reactor.now = float(second)
            self.stats.note_error(self.client, 'NACK')
        self.assertEqual(len(self.stats.clients['mcu:1'].buckets), 3600)
        snapshot = self.client_snapshot()
        self.assertEqual(snapshot['errors_last_hour'], 3600)
        self.assertEqual(snapshot['errors_total'], 15000)
        self.assertEqual(snapshot['by_status_last_hour'], {'NACK': 3600})

    @mock.patch('klippy.extras.i2c_stats.logging.warning')
    def test_full_stale_window_expires_before_a_new_error(self, warning):
        for second in range(3600):
            self.reactor.now = float(second)
            self.stats.note_error(self.client,
                                  'NACK' if second % 2 else 'BUS_TIMEOUT')
        self.assertEqual(len(self.stats.clients['mcu:1'].buckets), 3600)
        # No intermediate status polls: error recording must expire the old
        # counters too, and remove status keys whose count becomes zero.
        self.reactor.now = 7200.
        self.stats.note_error(self.client, 'START_NACK')
        self.assertEqual(len(self.stats.clients['mcu:1'].buckets), 1)
        snapshot = self.client_snapshot()
        self.assertEqual(snapshot['errors_total'], 3601)
        self.assertEqual(snapshot['errors_last_hour'], 1)
        self.assertEqual(snapshot['by_status_last_hour'], {'START_NACK': 1})
        self.assertEqual(self.client_snapshot(10800.)['by_status_last_hour'],
                         {})

    @mock.patch('klippy.extras.i2c_stats.logging.warning')
    def test_skipped_requests_and_success_are_not_bus_errors(self, warning):
        self.stats.note_skip(self.client)
        self.stats.note_skip(self.client)
        self.stats.note_error(self.client, 'SUCCESS')
        snapshot = self.client_snapshot()
        self.assertEqual(snapshot['skipped_total'], 2)
        self.assertEqual(snapshot['errors_total'], 0)
        self.assertEqual(snapshot['errors_last_hour'], 0)
        warning.assert_not_called()

    @mock.patch('klippy.extras.i2c_stats.logging.warning')
    def test_log_rate_limit_is_per_client_and_keeps_all_counts(self, warning):
        other = FakeClient(oid=2)
        self.stats.register_client(other, 'display')
        for eventtime in (0., 1., 59.99):
            self.reactor.now = eventtime
            self.stats.note_error(self.client, 'NACK')
        self.assertEqual(warning.call_count, 1)
        self.stats.note_error(other, 'NACK')
        self.assertEqual(warning.call_count, 2)
        self.reactor.now = 60.
        self.stats.note_error(self.client, 'NACK')
        self.assertEqual(warning.call_count, 3)
        self.assertEqual(self.snapshot()['errors_total'], 5)

    @mock.patch('klippy.extras.i2c_stats.logging.warning')
    def test_errors_appear_on_console_without_flooding_or_double_logging(
            self, warning):
        gcode = self.config.printer.gcode
        self.stats.note_error(self.client, 'BUS_TIMEOUT')
        self.assertEqual(len(gcode.messages), 1)
        message, log = gcode.messages[0]
        self.assertFalse(log)
        for expected in ('I2C error', 'ads1x1x laser_adc', 'mcu=mcu',
                         'bus=twi1', 'oid=1', 'address=0x48', 'BUS_TIMEOUT'):
            self.assertIn(expected, message)
        self.reactor.now = 1.
        self.stats.note_error(self.client, 'NACK')
        self.stats.note_skip(self.client)
        self.stats.note_error(self.client, 'SUCCESS')
        self.assertEqual(len(gcode.messages), 1)
        # A different client gets its own immediate diagnostic.
        other = FakeClient(oid=2, address=0x40)
        self.stats.register_client(other, 'display status')
        self.stats.note_error(other, 'START_NACK')
        self.assertEqual(len(gcode.messages), 2)
        self.assertIn('display status', gcode.messages[-1][0])
        self.reactor.now = 60.
        self.stats.note_error(self.client, 'NACK')
        self.assertEqual(len(gcode.messages), 3)
        self.assertEqual(warning.call_count, 3)
        self.assertEqual(self.snapshot()['errors_total'], 4)

    @mock.patch('klippy.extras.i2c_stats.logging.warning')
    def test_gcode_reports_client_and_status(self, warning):
        self.stats.note_error(self.client, 'BUS_TIMEOUT')
        self.stats.note_skip(self.client)
        gcmd = FakeGcmd()
        self.config.printer.gcode.callback(gcmd)
        for expected in ('last hour: 1', 'ads1x1x laser_adc', 'mcu=mcu',
                         'bus=twi1', 'oid=1', 'address=0x48',
                         'BUS_TIMEOUT=1', 'skipped=1'):
            self.assertIn(expected, gcmd.message)


if __name__ == '__main__':
    unittest.main()
