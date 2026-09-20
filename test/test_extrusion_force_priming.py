import unittest

from klippy.extras.extrusion_force_motion import MotionForceSampler
from klippy.extras.extrusion_force_priming import (
    ExtrusionForcePriming, PrimingThermalEvidence)


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

    def wait_moves(self):
        pass


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


class FakeConfig:
    def __init__(self, reactor):
        self.reactor = reactor

    def get_printer(self):
        return self

    def get_reactor(self):
        return self.reactor

    def getfloat(self, name, default=None, **kwargs):
        return default

    def getint(self, name, default=None, **kwargs):
        return default


class FakeThermal:
    def capture_baseline(self, *args):
        pass

    def start(self):
        self.stopped = False

    def confirmed(self):
        return False

    def stop(self):
        self.stopped = True


def make_priming(times=None):
    priming = object.__new__(ExtrusionForcePriming)
    priming.reactor = FakeReactor(times)
    priming.motion = MotionForceSampler(FakeConfig(priming.reactor))
    priming.motion.idle = lambda gcmd, tool: [0.0] * 4
    priming.moving_samples = [0.0] * 4
    priming.thermal = FakeThermal()
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
    priming.overpressure = False
    priming.active_force_limit = None
    priming._capture_baseline = lambda gcmd: setattr(
        priming, "baseline_force", 0.0)
    return priming


class ExtrusionForcePrimingTest(unittest.TestCase):
    def assert_cleanup(self, priming):
        self.assertEqual(len(priming.load_cell.added), 1)
        self.assertEqual(len(priming.load_cell.removed), 1)
        self.assertEqual(priming.monitor.claimed, ["PRESSURE_PRIME"])
        self.assertEqual(priming.monitor.released, ["PRESSURE_PRIME"])
        self.assertIsNone(priming.active_force_limit)
        self.assertIsNone(priming.baseline_force)
        self.assertTrue(priming.thermal.stopped)

    def test_success_requires_two_consecutive_above_threshold_samples(self):
        priming = make_priming()
        forces = iter((590.0, 610.0, 615.0))
        calls = []

        def extrude_segment(gcmd, speed, length):
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
        priming = make_priming([0.0, 0.0, 0.0, 100.0])
        priming._extrude_segment = lambda gcmd, speed, length: 700.0
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

        def overpressure(gcmd, speed, length):
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

    def low_force_priming(self, forces, thermal=False, params=None):
        priming = make_priming()
        iterator = iter(forces)
        priming.thermal.confirmed = lambda: thermal

        def extrude(gcmd, speed, length):
            force = next(iterator)
            priming.moving_samples = [force] * 4
            return force

        priming._extrude_segment = extrude
        return priming, FakeGCmd(params)

    def test_low_force_flow_needs_motion_and_heat(self):
        priming, gcmd = self.low_force_priming([80.0, 85.0], thermal=True)
        priming.cmd_PRESSURE_PRIME(gcmd)
        self.assertIn("SUCCESS after 2mm", gcmd.messages[-1])
        self.assertTrue(any("increased heater power" in message
                            for message in gcmd.messages))
        self.assert_cleanup(priming)

    def test_empty_extruder_motion_without_heat_does_not_succeed(self):
        priming, gcmd = self.low_force_priming([80.0] * 5)
        with self.assertRaisesRegex(CommandError, "Maximum"):
            priming.cmd_PRESSURE_PRIME(gcmd)
        self.assertIn("FAILURE after 5mm", gcmd.messages[-1])
        self.assert_cleanup(priming)

    def test_heating_without_directional_force_does_not_succeed(self):
        for force in (0.0, 10.0, -80.0):
            with self.subTest(force=force):
                priming, gcmd = self.low_force_priming(
                    [force] * 5, thermal=True)
                with self.assertRaisesRegex(CommandError, "Maximum"):
                    priming.cmd_PRESSURE_PRIME(gcmd)
                self.assert_cleanup(priming)

    def test_low_force_confirmations_must_be_consecutive(self):
        priming, gcmd = self.low_force_priming(
            [80.0, 0.0, 80.0, 0.0, 80.0], thermal=True)
        with self.assertRaisesRegex(CommandError, "Maximum"):
            priming.cmd_PRESSURE_PRIME(gcmd)

    def test_fractional_length_is_not_rounded_up(self):
        priming = make_priming()
        lengths = []

        def extrude(gcmd, speed, length):
            lengths.append(length)
            return 0.0

        priming._extrude_segment = extrude
        gcmd = FakeGCmd({"LENGTH": 2.5})
        with self.assertRaisesRegex(CommandError, "Maximum"):
            priming.cmd_PRESSURE_PRIME(gcmd)
        self.assertEqual(lengths, [1.0, 1.0, 0.5])
        self.assertIn("FAILURE after 2.5mm", gcmd.messages[-1])


class ThermalClock:
    def __init__(self):
        self.time = 0.0
        self.timer = None

    def monotonic(self):
        return self.time

    def pause(self, time):
        self.time = time

    def register_timer(self, callback, time):
        self.timer = callback
        return callback

    def unregister_timer(self, timer):
        self.timer = None


class ThermalHeater:
    def __init__(self, status=None):
        self.status = status or (lambda time: (210.0, 0.2, 210.0))

    def get_status(self, time):
        temp, power, target = self.status(time)
        return {"temperature": temp, "power": power, "target": target}


class PrimingThermalEvidenceTest(unittest.TestCase):
    def make_thermal(self, heater=None):
        thermal = PrimingThermalEvidence(FakeConfig(ThermalClock()))
        thermal.capture_baseline(FakeGCmd(), heater or ThermalHeater(), 210.0)
        return thermal

    def feed(self, thermal, powers, temp=210.0, target=210.0):
        start = thermal.reactor.monotonic()
        for index, power in enumerate(powers):
            thermal.heater = ThermalHeater(
                lambda time: (temp, power, target))
            thermal._sample(start + 0.1 * index)

    def test_sustained_extra_heater_load_confirms_flow(self):
        thermal = self.make_thermal()
        self.feed(thermal, [0.26] * 23)
        self.assertTrue(thermal.confirmed())

    def test_idle_power_and_isolated_pwm_peak_do_not_confirm(self):
        for powers in ([0.2] * 23, [0.2] * 22 + [1.0], [0.26] * 10):
            with self.subTest(powers=powers):
                thermal = self.make_thermal()
                self.feed(thermal, powers)
                self.assertFalse(thermal.confirmed())

    def test_temperature_or_target_change_rejects_heater_evidence(self):
        for temp, target in ((205.0, 210.0), (210.0, 215.0), (215.0, 210.0)):
            thermal = self.make_thermal()
            self.feed(thermal, [0.3] * 23, temp=temp, target=target)
            self.assertFalse(thermal.confirmed())

    def test_unsettled_warmup_disables_thermal_fallback(self):
        thermal = self.make_thermal(ThermalHeater(
            lambda time: (180.0 + time, 0.8, 210.0)))
        self.assertIsNone(thermal.baseline_power)
        thermal.start()
        self.assertIsNone(thermal.timer)
        self.assertFalse(thermal.confirmed())

    def test_baseline_waits_for_warmup_to_finish(self):
        thermal = self.make_thermal(ThermalHeater(
            lambda time: (min(210.0, 207.0 + time),
                          0.8 if time < 3.0 else 0.2, 210.0)))
        self.assertGreater(thermal.reactor.monotonic(), 7.0)
        self.assertAlmostEqual(thermal.baseline_power, 0.2, delta=0.02)

    def test_idle_pwm_noise_raises_required_increase(self):
        thermal = self.make_thermal(ThermalHeater(
            lambda time: (210.0, 0.15 + (int(time * 10) % 2) * 0.1, 210.0)))
        self.feed(thermal, [0.25] * 23)
        self.assertFalse(thermal.confirmed())

    def test_timer_is_removed_on_stop(self):
        thermal = self.make_thermal()
        thermal.start()
        self.assertIsNotNone(thermal.reactor.timer)
        thermal.stop()
        self.assertIsNone(thermal.reactor.timer)


if __name__ == "__main__":
    unittest.main()
