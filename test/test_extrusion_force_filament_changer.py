import unittest

from klippy.extras.extrusion_force_filament_changer import (
    ExtrusionForceFilamentChanger)
from klippy.extras.extrusion_force_motion import MotionForceSampler
from klippy.extras.extrusion_force_priming import ExtrusionForcePriming
if __package__:
    from .test_extrusion_force_priming import (
        CommandError, FakeConfig, FakeGCmd, FakeThermal, ThermalClock)
else:
    from test_extrusion_force_priming import (
        CommandError, FakeConfig, FakeGCmd, FakeThermal, ThermalClock)


class MotionForceSamplerTest(unittest.TestCase):
    def sampler(self):
        return MotionForceSampler(FakeConfig(ThermalClock()))

    def test_directional_motion_below_target_is_confirmed_twice(self):
        for direction in (-1.0, 1.0):
            sampler = self.sampler()
            moving = [1000.0 + direction * 80.0] * 4
            idle = [1000.0] * 4
            self.assertFalse(sampler.check(
                moving, idle, idle, 1000.0, direction, 500.0))
            self.assertTrue(sampler.check(
                moving, idle, idle, 1000.0, direction, 500.0))

    def test_spring_noise_wrong_direction_and_target_peaks_are_rejected(self):
        idle = [1000.0] * 4
        cases = [
            ([1080.0] * 4, idle, [1080.0] * 4),
            ([1080.0] * 4, [1080.0] * 4, idle),
            ([1010.0] * 4, idle, idle),
            ([1000.0] * 3 + [1120.0], idle, idle),
            ([1080.0] * 4, [960.0, 1040.0] * 2, idle),
            ([920.0] * 4, idle, idle),
            ([1050.0] * 3 + [1600.0], idle, idle),
            ([1080.0], idle, idle),
        ]
        for moving, before, after in cases:
            sampler = self.sampler()
            with self.subTest(moving=moving, before=before, after=after):
                for _ in range(3):
                    self.assertFalse(sampler.check(
                        moving, before, after, 1000.0, 1.0, 500.0))

    def test_missing_evidence_resets_confirmation(self):
        sampler = self.sampler()
        idle, moving = [0.0] * 4, [80.0] * 4
        self.assertFalse(sampler.check(moving, idle, idle, 0.0, 1.0, 500.0))
        self.assertFalse(sampler.check([], idle, idle, 0.0, 1.0, 500.0))
        self.assertFalse(sampler.check(moving, idle, idle, 0.0, 1.0, 500.0))
        self.assertTrue(sampler.check(moving, idle, idle, 0.0, 1.0, 500.0))

    def test_collection_uses_timestamps_and_waits_for_delayed_sample(self):
        sampler = self.sampler()
        samples = iter([(1.1, 10.0), (1.2, 20.0), (1.3, 30.0), (1.4, 0.0)])

        def deliver(time):
            sampler.reactor.time = time
            sample_time, force = next(samples)
            sampler.add_sample({"print_time": sample_time,
                                "absolute_force_g": force})

        sampler.add_sample({"print_time": 0.9, "absolute_force_g": -500.0})
        sampler.reactor.pause = deliver
        self.assertEqual(sampler.collect(FakeGCmd(), 1.0, 1.4),
                         [10.0, 20.0, 30.0])

    def test_lost_samples_time_out(self):
        sampler = self.sampler()
        with self.assertRaisesRegex(CommandError, "Timeout collecting"):
            sampler.collect(FakeGCmd(), 1.0, 1.4)


class SimReactor(ThermalClock):
    def __init__(self, printer, delay):
        super().__init__()
        self.printer = printer
        self.next_sample = 1.0 / 16.0
        self.delay = delay

    def pause(self, time):
        while self.next_sample + self.delay <= time:
            sample_time = self.next_sample
            self.time = sample_time + self.delay
            sample = {"print_time": sample_time,
                      "absolute_force_g": 1000.0
                      + self.printer.force_at(sample_time)}
            for callback in list(self.printer.cell.clients):
                callback(sample)
            self.next_sample += 1.0 / 16.0
        self.time = time


class SimTool:
    def __init__(self, printer):
        self.printer = printer
        self.position = [0.0] * 4
        self.end_time = 0.0
        self.moves = []

    def get_extruder(self):
        return self.printer.extruder

    def get_position(self):
        return list(self.position)

    def get_last_move_time(self):
        self.end_time = max(self.end_time, self.printer.reactor.time)
        return self.end_time

    def manual_move(self, position, speed):
        start = self.get_last_move_time()
        self.end_time = start + abs(position[3] - self.position[3]) / speed
        self.moves.append((start, self.end_time, self.position[3], position[3]))
        self.position = list(position)

    def dwell(self, delay):
        self.end_time = self.get_last_move_time() + delay

    def wait_moves(self):
        if self.end_time > self.printer.reactor.time:
            self.printer.reactor.pause(self.end_time)


class SimCell:
    def __init__(self):
        self.clients = []

    def add_client(self, callback):
        self.clients.append(callback)

    def remove_client(self, callback):
        self.clients.remove(callback)

    def get_status(self, time):
        return {"is_calibrated": True}


class SimExtruder:
    max_e_velocity = 20.0
    filament_area = 2.4
    max_temp = 292.0
    min_extrude_temp = 170.0

    def __init__(self):
        self.heater = self
        self.target = 200.0
        self.temperature = 170.0

    def get_name(self):
        return "extruder"

    def get_heater(self):
        return self

    def get_status(self, time):
        return {"target": self.target, "temperature": self.temperature,
                "can_extrude": True}


class SimPrinter:
    def __init__(self, mode="flow", delay=0.0):
        self.mode = mode
        self.cell = SimCell()
        self.reactor = SimReactor(self, delay)
        self.extruder = SimExtruder()
        self.tool = SimTool(self)
        self.temperatures = []
        self.scripts = []
        self.command_error = CommandError

    def force_at(self, time):
        position = 0.0
        for start, end, old_e, new_e in self.tool.moves:
            if time < start:
                break
            moving = time < end
            position = (old_e + (new_e - old_e) * (time - start) / (end - start)
                        if moving else new_e)
            if moving:
                if self.mode == "spike":
                    return 4000.0
                if self.mode == "flow":
                    return 80.0 if new_e > old_e else -80.0
                if self.mode == "warm_flow" and self.extruder.target >= 173.0:
                    return 80.0 if new_e > old_e else -80.0
                break
        if self.mode == "spring":
            return position * 50.0
        if self.mode == "warm_flow" and self.extruder.target < 173.0:
            return position * 1000.0
        return 0.0

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return {"load_cell": self.cell, "gcode": self, "heaters": self,
                "toolhead": self.tool, "extruder": self.extruder}.get(
                    name, default)

    def load_object(self, config, name):
        return self.lookup_object(name)

    def register_event_handler(self, *args):
        pass

    def register_command(self, *args, **kwargs):
        pass

    def run_script_from_command(self, script):
        self.scripts.append(script)

    def set_temperature(self, heater, target, wait=False):
        self.temperatures.append(target)
        self.extruder.target = target
        if target:
            self.extruder.temperature = target


class SimConfig(FakeConfig):
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer

    def get_name(self):
        return "extrusion_force_priming"

    def getfloat(self, name, default=None, **kwargs):
        return {"force_threshold": 600.0, "max_prime_length": 5.0}.get(
            name, default)

    def get(self, name, default=None):
        return default

    def getboolean(self, name, default=None):
        return default


class SimGCmd(FakeGCmd):
    def get_int(self, name, default=None, **kwargs):
        return int(self.get_float(name, default, **kwargs))


class FilamentChangerTest(unittest.TestCase):
    def make_changer(self, mode="flow", delay=0.0):
        printer = SimPrinter(mode, delay)
        changer = ExtrusionForceFilamentChanger(SimConfig(printer))
        changer._handle_ready()
        return changer, printer

    def assert_cleanup(self, changer, printer):
        self.assertFalse(changer.running)
        self.assertEqual(printer.cell.clients, [])
        self.assertIn("RESTORE_GCODE_STATE", printer.scripts[-1])

    def test_already_soft_unload_completes_without_holding_force_or_ramp(self):
        for delay in (0.0, 0.4):
            changer, printer = self.make_changer(delay=delay)
            gcmd = SimGCmd({"LENGTH": 8.0})
            changer.cmd_UNLOAD_FILAMENT(gcmd)
            self.assertAlmostEqual(printer.tool.position[3], -8.0)
            self.assertEqual(printer.temperatures, [170.0, 0.0])
            self.assertTrue(any("Unload motion detected" in message
                                for message in gcmd.messages))
            self.assert_cleanup(changer, printer)

    def test_soft_load_counts_probe_feed_and_holds_start_temperature(self):
        for delay in (0.0, 0.4):
            for length in (0.3, 8.0):
                changer, printer = self.make_changer(delay=delay)
                gcmd = SimGCmd({"LENGTH": length})
                changer.cmd_LOAD_FILAMENT(gcmd)
                self.assertAlmostEqual(printer.tool.position[3], length)
                self.assertEqual(printer.temperatures, [170.0, 200.0])
                self.assertTrue(any("Load motion detected" in message
                                    for message in gcmd.messages))
                self.assert_cleanup(changer, printer)

    def test_missing_filament_or_elastic_tension_do_not_confirm_unload(self):
        for mode in ("empty", "spring"):
            changer, printer = self.make_changer(mode)
            changer.unload_preload_max = 1.0
            with self.assertRaisesRegex(CommandError, "Unable to build"):
                changer.cmd_UNLOAD_FILAMENT(SimGCmd())
            self.assertAlmostEqual(printer.tool.position[3], -1.0)
            self.assertEqual(printer.temperatures[-1], 0.0)
            self.assert_cleanup(changer, printer)

    def test_missing_filament_or_static_compression_do_not_confirm_load(self):
        for mode in ("empty", "spring"):
            changer, printer = self.make_changer(mode)
            changer.load_seek_max = 2.0
            with self.assertRaisesRegex(CommandError, "No load-cell contact"):
                changer.cmd_LOAD_FILAMENT(SimGCmd())
            self.assertAlmostEqual(printer.tool.position[3], 2.0)
            self.assertEqual(printer.temperatures[-1], 200.0)
            self.assert_cleanup(changer, printer)

    def test_force_spikes_abort_both_commands_and_restore_resources(self):
        for command in ("cmd_LOAD_FILAMENT", "cmd_UNLOAD_FILAMENT"):
            changer, printer = self.make_changer("spike")
            with self.assertRaisesRegex(CommandError, "safety limit exceeded"):
                getattr(changer, command)(SimGCmd())
            self.assertEqual(len(printer.tool.moves), 1)
            self.assert_cleanup(changer, printer)

    def test_initial_force_contact_still_uses_temperature_ramp(self):
        changer, printer = self.make_changer("warm_flow")
        changer.load_seek_max = 0.5
        gcmd = SimGCmd({"LENGTH": 4.0})
        changer.cmd_LOAD_FILAMENT(gcmd)
        self.assertTrue(any("Load force reached" in message
                            for message in gcmd.messages))
        self.assertIn(173.0, printer.temperatures)
        self.assertAlmostEqual(printer.tool.position[3], 4.5)
        self.assert_cleanup(changer, printer)

    def test_initial_unload_force_still_uses_temperature_ramp(self):
        changer, printer = self.make_changer("warm_flow")
        changer.unload_preload_max = 1.0
        gcmd = SimGCmd({"LENGTH": 4.0})
        changer.cmd_UNLOAD_FILAMENT(gcmd)
        self.assertTrue(any("Unload preload established" in message
                            for message in gcmd.messages))
        self.assertIn(173.0, printer.temperatures)
        self.assertAlmostEqual(printer.tool.position[3], -4.0)
        self.assert_cleanup(changer, printer)


class PrimingSamplingIntegrationTest(unittest.TestCase):
    def test_actual_force_windows_with_slow_delayed_adc(self):
        for delay in (0.0, 0.4):
            for threshold in (70.0, 600.0):
                printer = SimPrinter(delay=delay)
                priming = ExtrusionForcePriming(SimConfig(printer))
                priming.thermal = FakeThermal()
                priming.thermal.confirmed = lambda: True
                priming._handle_ready()
                gcmd = SimGCmd({"THRESHOLD": threshold})
                priming.cmd_PRESSURE_PRIME(gcmd)
                self.assertAlmostEqual(printer.tool.position[3], 2.0)
                self.assertIn("SUCCESS after 2mm", gcmd.messages[-1])
                self.assertEqual(printer.cell.clients, [])
                self.assertTrue(priming.thermal.stopped)
                self.assertEqual(printer.temperatures, [210.0, 200.0])


if __name__ == "__main__":
    unittest.main()
