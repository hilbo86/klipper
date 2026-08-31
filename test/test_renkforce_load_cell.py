import unittest

from klippy.extras import load_cell_probe_renkforce


class FakeADC:
    def setup_minmax(self, *args):
        self.minmax = args

    def setup_adc_callback(self, report_time, callback):
        self.report_time = report_time
        self.callback = callback


class FakePins:
    def __init__(self):
        self.adc = FakeADC()

    def setup_pin(self, pin_type, pin_name):
        return self.adc


class FakeObject:
    def register_command(self, *args, **kwargs):
        pass

    def set(self, *args):
        pass


class FakePrinter:
    def __init__(self):
        self.pins = FakePins()
        self.objects = {
            "pins": self.pins,
            "gcode": FakeObject(),
            "configfile": FakeObject(),
        }

    def get_reactor(self):
        return FakeObject()

    def lookup_object(self, name):
        return self.objects[name]

    def register_event_handler(self, *args):
        pass


class FakeSection:
    def getfloat(self, name):
        return 8.0


class FakeConfig:
    def __init__(self, force_calibration=None, orientation="normal"):
        self.printer = FakePrinter()
        self.values = {
            "adc": "PA0",
            "adc_rate": 10.0,
            "max_abs_force": 5000.0,
            "sensor_orientation": orientation,
        }
        if force_calibration is not None:
            self.values["force_calibration"] = force_calibration

    def get_printer(self):
        return self.printer

    def get_name(self):
        return "load_cell_probe_renkforce"

    def get(self, name, default=None):
        return self.values.get(name, default)

    def getfloat(self, name, default=None, **kwargs):
        return float(self.values.get(name, default))

    def getint(self, name, default=None, **kwargs):
        return int(self.values.get(name, default))

    def getchoice(self, name, choices, default=None):
        return choices[self.values.get(name, default)]

    def getsection(self, name):
        return FakeSection()


class LoadCellSampleApiTest(unittest.TestCase):
    def test_uncalibrated_default_is_not_published_as_grams(self):
        sensor = load_cell_probe_renkforce.LoadCellProbe(FakeConfig())
        sensor._adc_callback(1.0, 42.0)
        status = sensor.get_status(1.0)
        self.assertFalse(status["is_calibrated"])
        self.assertIsNone(status["force_g"])
        self.assertEqual(status["last_force"], 0.0)

    def test_timestamped_samples_orientation_and_rolling_status(self):
        sensor = load_cell_probe_renkforce.LoadCellProbe(
            FakeConfig(force_calibration=2.0, orientation="inverted"))
        samples = []
        sensor.add_client(samples.append)
        sensor.add_client(samples.append)
        sensor._adc_callback(1.0, 10.0)
        sensor._adc_callback(1.5, 8.0)

        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[-1]["print_time"], 1.5)
        self.assertEqual(samples[-1]["raw_adc"], 8.0)
        self.assertEqual(samples[-1]["absolute_force_g"], -16.0)
        self.assertEqual(samples[-1]["force_g"], 4.0)
        status = sensor.get_status(1.5)
        self.assertEqual(status["force_g"], 2.0)
        self.assertEqual(status["min_force_g"], 0.0)
        self.assertEqual(status["max_force_g"], 4.0)
        self.assertEqual(status["sample_rate"], 2.0)

        sensor._adc_callback(2.1, 7.0)
        status = sensor.get_status(2.1)
        self.assertEqual(status["min_force_g"], 4.0)
        self.assertEqual(status["max_force_g"], 6.0)

    def test_legacy_subscribe_is_idempotent_and_unsubscribes(self):
        sensor = load_cell_probe_renkforce.LoadCellProbe(
            FakeConfig(force_calibration=1.0))
        forces = []
        callback = forces.append
        sensor.subscribe_force(callback)
        sensor.subscribe_force(callback)
        sensor._adc_callback(1.0, 3.0)
        sensor._adc_callback(1.1, 5.0)
        sensor.unsubscribe_force(callback)
        sensor._adc_callback(1.2, 7.0)
        self.assertEqual(forces, [0.0, 2.0])


if __name__ == "__main__":
    unittest.main()
