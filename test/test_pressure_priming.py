import unittest

from klippy.extras.pressure_priming import PressurePriming


class CommandError(Exception):
    pass


class FakeGCmd:
    def __init__(self, params=None):
        self.params = params or {}
        self.messages = []

    def error(self, message):
        return CommandError(message)

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_float(self, name, default=None, minval=None, maxval=None,
                  above=None, below=None):
        value = float(self.params.get(name, default))
        if minval is not None and value < minval:
            raise self.error("%s below minimum" % (name,))
        if maxval is not None and value > maxval:
            raise self.error("%s above maximum" % (name,))
        if above is not None and value <= above:
            raise self.error("%s must be above limit" % (name,))
        if below is not None and value >= below:
            raise self.error("%s must be below limit" % (name,))
        return value

    def respond_info(self, message):
        self.messages.append(message)


class FakeReactor:
    def __init__(self, times=None):
        self.times = list(times or [0.0])
        self.last_time = self.times[-1]

    def monotonic(self):
        if self.times:
            self.last_time = self.times.pop(0)
        return self.last_time


class FakeHeater:
    min_extrude_temp = 170.0
    max_temp = 292.0


class FakeExtruder:
    max_e_velocity = 20.0

    def __init__(self, name="extruder", target=180.0):
        self.name = name
        self.target = target
        self.heater = FakeHeater()

    def get_name(self):
        return self.name

    def get_status(self, eventtime):
        return {"target": self.target}

    def get_heater(self):
        return self.heater


class FakeTool:
    def __init__(self, extruder):
        self.extruder = extruder

    def get_extruder(self):
        return self.extruder


class FakeLoadCell:
    def __init__(self):
        self.added = []
        self.removed = []

    def add_client(self, callback):
        self.added.append(callback)

    def remove_client(self, callback):
        self.removed.append(callback)

    def get_status(self, eventtime):
        return {"is_calibrated": True}


class FakeMonitor:
    def __init__(self):
        self.claimed = []
        self.released = []

    def claim_operation(self, owner):
        self.claimed.append(owner)

    def release_operation(self, owner):
        self.released.append(owner)


class FakeHeaters:
    def __init__(self):
        self.calls = []

    def set_temperature(self, heater, target, wait=False):
        self.calls.append((heater, target, wait))


class FakePrinter:
    def __init__(self, heaters):
        self.heaters = heaters

    def lookup_object(self, name, default=None):
        if name == "heaters":
            return self.heaters
        return default


class FakeGCode:
    def __init__(self):
        self.scripts = []

    def run_script_from_command(self, script):
        self.scripts.append(script)


def make_priming(times=None):
    priming = object.__new__(PressurePriming)
    priming.reactor = FakeReactor(times)
    priming.force_threshold_default = 600.0
    priming.force_safety_limit = 3000.0
    priming.max_prime_length_default = 5.0
    priming.baseline_samples = 3
    priming.sample_timeout = 1.0
    priming.extruder = FakeExtruder()
    priming.tool = FakeTool(priming.extruder)
    priming.load_cell = FakeLoadCell()
    priming.monitor = FakeMonitor()
    priming.heaters = FakeHeaters()
    priming.printer = FakePrinter(priming.heaters)
    priming.gcode = FakeGCode()
    priming.baseline_values = []
    priming.baseline_force = None
    priming.sample_window = None
    priming.window_values = []
    priming.overpressure = False
    priming.active_force_limit = None
    priming._capture_baseline = lambda gcmd: setattr(
        priming, "baseline_force", 0.0)
    return priming


class PressurePrimingTest(unittest.TestCase):
    def assert_cleanup(self, priming):
        self.assertEqual(len(priming.load_cell.added), 1)
        self.assertEqual(len(priming.load_cell.removed), 1)
        self.assertEqual(priming.monitor.claimed, ["PRESSURE_PRIME"])
        self.assertEqual(priming.monitor.released, ["PRESSURE_PRIME"])
        self.assertIsNone(priming.active_force_limit)
        self.assertIsNone(priming.baseline_force)

    def test_success_requires_two_consecutive_above_threshold_samples(self):
        priming = make_priming()
        forces = iter((590.0, 610.0, 615.0))
        calls = []

        def extrude_segment(gcmd, speed):
            calls.append(speed)
            return next(forces)

        priming._extrude_segment = extrude_segment
        gcmd = FakeGCmd({"LIMIT": 2500.0})
        priming.cmd_PRESSURE_PRIME(gcmd)

        self.assertEqual(len(calls), 3)
        self.assertEqual(priming.force_safety_limit, 3000.0)
        self.assertEqual(
            priming.heaters.calls,
            [(priming.extruder.heater, 210.0, True),
             (priming.extruder.heater, 180.0, False)])
        self.assertIn("SUCCESS after 3mm", gcmd.messages[-1])
        self.assertIn("Nr   3", gcmd.messages[-1])
        self.assert_cleanup(priming)

    def test_timeout_reports_failure_and_restores_resources(self):
        priming = make_priming([0.0, 0.0, 0.0, 10.0])
        priming._extrude_segment = lambda gcmd, speed: 700.0
        gcmd = FakeGCmd({"LENGTH": 2.0})

        with self.assertRaisesRegex(CommandError, "timed out"):
            priming.cmd_PRESSURE_PRIME(gcmd)

        self.assertIn("FAILURE after 0mm", gcmd.messages[-1])
        self.assertIn("Pressure priming timed out", gcmd.messages[-1])
        self.assertEqual(
            priming.heaters.calls[-1],
            (priming.extruder.heater, 180.0, False))
        self.assert_cleanup(priming)

    def test_overpressure_reports_failure_and_restores_resources(self):
        priming = make_priming()

        def overpressure(gcmd, speed):
            raise gcmd.error("Pressure-prime force safety limit exceeded")

        priming._extrude_segment = overpressure
        gcmd = FakeGCmd()

        with self.assertRaisesRegex(CommandError, "safety limit exceeded"):
            priming.cmd_PRESSURE_PRIME(gcmd)

        self.assertIn("FAILURE after 0mm", gcmd.messages[-1])
        self.assertIn("safety limit exceeded", gcmd.messages[-1])
        self.assert_cleanup(priming)

    def test_callback_uses_temporary_command_limit(self):
        priming = make_priming()
        priming.baseline_force = 100.0
        priming.active_force_limit = 500.0
        priming._sample_callback({
            "absolute_force_g": 650.0, "print_time": 1.0})
        self.assertTrue(priming.overpressure)

    def test_limit_cannot_raise_configured_safety_ceiling(self):
        priming = make_priming()
        gcmd = FakeGCmd({"LIMIT": 3001.0})

        with self.assertRaisesRegex(CommandError, "LIMIT above maximum"):
            priming.cmd_PRESSURE_PRIME(gcmd)

        self.assertEqual(priming.force_safety_limit, 3000.0)
        self.assertEqual(priming.load_cell.added, [])
        self.assertEqual(priming.monitor.claimed, [])


if __name__ == "__main__":
    unittest.main()
