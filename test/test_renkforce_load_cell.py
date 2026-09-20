import ast
import pathlib
import struct
import unittest

from klippy.extras import load_cell_probe_renkforce


class FakeADC:
    def setup_minmax(self, *args):
        self.minmax = args

    def setup_adc_callback(self, report_time, callback):
        self.report_time = report_time
        self.callback = callback


class FakeBatchADC:
    def setup_adc_sample(self, report_time, sample_time=0.0, sample_count=1,
                         batch_num=1, minval=0.0, maxval=1.0,
                         range_check_count=0):
        self.sampling = (report_time, sample_time, sample_count, batch_num,
                         minval, maxval, range_check_count)

    def setup_adc_callback(self, callback):
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
    def __init__(self, force_calibration=None, orientation="normal", adc=None):
        self.printer = FakePrinter()
        if adc is not None:
            self.printer.pins.adc = adc
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
    def test_current_mcu_adc_class_connects_and_delivers_samples(self):
        # Load the real, standalone ADC class without importing the serial/C
        # transport dependencies. This also runs on Windows and prevents our
        # ADC test doubles from hiding a future core API change.
        source = pathlib.Path(__file__).resolve().parents[1] / 'klippy/mcu.py'
        tree = ast.parse(source.read_text())
        adc_node = next(node for node in tree.body
                        if isinstance(node, ast.ClassDef)
                        and node.name == 'MCU_adc')
        namespace = {'struct': struct}
        exec(compile(ast.Module(body=[adc_node], type_ignores=[]),
                     str(source), 'exec'), namespace)

        class MCU:
            def register_config_callback(self, callback):
                self.config_callback = callback

            def clock32_to_clock64(self, clock):
                return clock

            def clock_to_print_time(self, clock):
                return clock / 1000.0

        adc = namespace['MCU_adc'](MCU(), {'pin': 'PA0'})
        sensor = load_cell_probe_renkforce.LoadCellProbe(
            FakeConfig(force_calibration=20000.0, adc=adc))
        self.assertEqual(adc._report_time, 0.1)
        self.assertEqual(adc._sample_time, 0.1)
        self.assertEqual(adc._sample_count, 1)
        self.assertEqual(adc._min_sample, -0.25)
        self.assertEqual(adc._max_sample, 0.25)
        adc._inv_max_adc = 0.001
        adc._report_clock = 100
        samples = []
        sensor.add_client(samples.append)
        adc._old_handle_analog_in_state({'next_clock': 1100, 'value': 100})
        adc._handle_analog_in_state({'next_clock': 1200,
                                     'values': struct.pack('<H', 110)})
        self.assertEqual([sample['print_time'] for sample in samples],
                         [1.0, 1.1])
        self.assertAlmostEqual(samples[-1]['force_g'], 200.0)

    def test_mcu_adc_sampling_preserves_rate_and_force_limits(self):
        adc = FakeBatchADC()
        load_cell_probe_renkforce.LoadCellProbe(
            FakeConfig(force_calibration=20000.0, adc=adc))
        self.assertEqual(adc.sampling, (0.1, 0.1, 1, 1, -0.25, 0.25, 0))

    def test_adc_batch_preserves_every_sample_and_original_timestamp(self):
        adc = FakeBatchADC()
        sensor = load_cell_probe_renkforce.LoadCellProbe(
            FakeConfig(force_calibration=2.0, orientation="inverted", adc=adc))
        samples = []
        sensor.add_client(samples.append)
        adc.callback([(1.0, 10.0), (1.1, 8.0), (1.2, 7.0)])
        adc.callback([])
        self.assertEqual([sample["print_time"] for sample in samples],
                         [1.0, 1.1, 1.2])
        self.assertEqual([sample["absolute_force_g"] for sample in samples],
                         [-20.0, -16.0, -14.0])
        self.assertEqual([sample["force_g"] for sample in samples],
                         [0.0, 4.0, 6.0])
        self.assertEqual(sensor._last_time, 1.2)
        self.assertEqual(sensor.get_status(1.2)["max_force_g"], 6.0)

    def test_legacy_hx711_setup_and_callback_remain_supported(self):
        adc = FakeADC()
        sensor = load_cell_probe_renkforce.LoadCellProbe(
            FakeConfig(force_calibration=20000.0, adc=adc))
        samples = []
        sensor.add_client(samples.append)
        self.assertEqual(adc.minmax, (0.1, 1, -0.25, 0.25))
        self.assertEqual(adc.report_time, 0.1)
        adc.callback(1.0, 0.1)
        adc.callback(1.1, 0.11)
        self.assertEqual(len(samples), 2)
        self.assertAlmostEqual(samples[-1]["force_g"], 200.0)

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
