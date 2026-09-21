import unittest
from unittest import mock

from klippy.extras import homing
from klippy.extras.load_cell_homing_guard import (
    HomingCollisionDetector, LoadCellHomingGuard)


class HomingCollisionDetectorTest(unittest.TestCase):
    def test_dynamic_noise_threshold_and_sign_independent_collision(self):
        for sign in (-1, 1):
            with self.subTest(sign=sign):
                detector = HomingCollisionDetector(30.0, 8.0, 0.02)
                detector.arm([100.0, 101.0, 99.0, 100.0])
                self.assertEqual(detector.baseline, 100.0)
                self.assertEqual(detector.threshold, 30.0)
                result = None
                for index in range(5):
                    result = detector.update(index * 0.01,
                                             100.0 + sign * 40.0)
                    if result is not None:
                        break
                self.assertIsNotNone(result)
                self.assertEqual(result["delta_g"], sign * 40.0)

    def test_one_sample_spike_does_not_trigger(self):
        detector = HomingCollisionDetector(30.0, 8.0, 0.0)
        detector.arm([0.0, 0.0, 0.0])
        for index, force in enumerate([0, 0, 1000, 0, 0, 0]):
            self.assertIsNone(detector.update(index * 0.01, force))
        self.assertEqual(detector.peak_delta, 0.0)

    def test_relative_baseline_and_rate_require_force_deviation(self):
        detector = HomingCollisionDetector(
            20.0, 8.0, 0.0, rate_threshold=1000.0,
            relative_baseline_factor=0.1)
        detector.arm([1000.0, 1000.0, 1000.0])
        self.assertEqual(detector.threshold, 100.0)
        for index, force in enumerate([1000, 1000, 1000, 1060, 1060]):
            self.assertIsNone(detector.update(index * 0.01, force))
        self.assertGreater(detector.peak_rate, 1000.0)


class HomingGuardIntegrationTest(unittest.TestCase):
    class Completion:
        def __init__(self):
            self.result = None

        def complete(self, result):
            self.result = result

    class Reactor:
        def __init__(self):
            self.now = 0.0
            self.guard = None

        def monotonic(self):
            return self.now

        def pause(self, until):
            self.now = until
            if self.guard.state == "BASELINING":
                for index, force in enumerate([100.0, 101.0, 99.0, 100.0]):
                    self.guard._handle_sample({
                        "print_time": self.now + index * 0.01,
                        "absolute_force_g": force})

        def completion(self):
            return HomingGuardIntegrationTest.Completion()

    class LoadCell:
        def get_status(self, eventtime):
            return {"is_calibrated": True}

        def add_client(self, callback):
            self.callback = callback

    class Steppers:
        def __init__(self):
            self.enabled = []

        def get_steppers(self):
            return ["stepper_x", "extruder", "extruder1"]

        def set_motors_enable(self, names, enabled):
            self.enabled.append((names, enabled))

    class Toolhead:
        def __init__(self):
            self.moves = 0

        def get_position(self):
            return [0.0, 0.0, 0.0, 0.0]

        def wait_moves(self):
            self.moves += 1

    class Monitor:
        def get_active_operation(self):
            return None

    class Printer:
        def __init__(self):
            self.reactor = HomingGuardIntegrationTest.Reactor()
            self.steppers = HomingGuardIntegrationTest.Steppers()
            self.toolhead = HomingGuardIntegrationTest.Toolhead()
            self.objects = {
                "load_cell": HomingGuardIntegrationTest.LoadCell(),
                "extrusion_force_monitor": HomingGuardIntegrationTest.Monitor(),
                "toolhead": self.toolhead, "stepper_enable": self.steppers}
            self.events = []
            self.command_error = ValueError
            self.config_error = ValueError

        def get_reactor(self):
            return self.reactor

        def lookup_object(self, name, default=None):
            return self.objects.get(name, default)

        def register_event_handler(self, event, callback):
            pass

        def send_event(self, event, *args):
            self.events.append((event, args))

    class Config:
        def __init__(self, printer):
            self.printer = printer
            self.values = {"enabled": True, "mode": "abort",
                           "minimum_collision_force": 30.0}

        def get_printer(self):
            return self.printer

        def get(self, name, default=None):
            return self.values.get(name, default)

        def getboolean(self, name, default=None):
            return self.values.get(name, default)

        def getfloat(self, name, default=None, **kwargs):
            return self.values.get(name, default)

        def getchoice(self, name, choices, default=None):
            return choices[self.values.get(name, default)]

        def error(self, message):
            return ValueError(message)

    def test_prepare_holds_extruders_and_collision_stops_completion(self):
        printer = self.Printer()
        guard = LoadCellHomingGuard(self.Config(printer))
        printer.reactor.guard = guard
        guard._handle_ready()
        self.assertTrue(guard.prepare_move([-20.0, 0.0, 0.0, 0.0]))
        self.assertEqual(printer.steppers.enabled,
                         [(["extruder", "extruder1"], True)])
        self.assertEqual(guard.detector.baseline, 100.0)
        self.assertEqual(printer.toolhead.moves, 2)
        completion = guard.start_move(1.0)
        with self.assertLogs(level="ERROR"):
            for index in range(5):
                printer.objects["load_cell"].callback({
                    "print_time": 1.0 + index * 0.01,
                    "absolute_force_g": 50.0})
        self.assertEqual(completion.result, 1)
        self.assertEqual(guard.state, "COLLISION")
        self.assertEqual(guard.collision["axis"], "x")
        self.assertEqual(guard.collision["motion_vector"], [-1.0, 0.0, 0.0])
        self.assertEqual(guard.collision_count, 1)
        guard.finish_move(was_collision=True)
        self.assertEqual(guard.get_status(0.0)["axis_peaks"]["x"]["moves"],
                         1)
        self.assertGreaterEqual(
            guard.get_status(0.0)["axis_peaks"]["x"]["force_delta_g"], 50.0)

    def test_diagnostic_mode_records_peaks_without_a_threshold(self):
        printer = self.Printer()
        config = self.Config(printer)
        config.values["mode"] = "diagnostic"
        del config.values["minimum_collision_force"]
        guard = LoadCellHomingGuard(config)
        printer.reactor.guard = guard
        guard._handle_ready()
        self.assertTrue(guard.prepare_move([0.0, 20.0, 0.0, 0.0]))
        self.assertIsNone(guard.start_move(1.0))
        for index in range(5):
            printer.objects["load_cell"].callback({
                "print_time": 1.0 + index * 0.01,
                "absolute_force_g": 150.0})
        self.assertIsNone(guard.collision)
        guard.finish_move()
        status = guard.get_status(0.0)
        self.assertIsNone(status["collision_threshold_g"])
        self.assertEqual(status["axis_peaks"]["y"]["moves"], 1)
        self.assertEqual(status["axis_peaks"]["y"]["force_delta_g"], 50.0)

    def test_homing_collision_clears_axis_and_runs_end_hooks(self):
        class Stepper:
            position = 5

            def get_name(self):
                return "stepper_x"

            def get_mcu_position(self):
                return self.position

            def mcu_to_commanded_position(self, value):
                return float(value)

            def get_commanded_position(self):
                return 5.0

            def get_past_mcu_position(self, trigger_time):
                return 6

            def calc_position_from_coord(self, coord):
                return coord[0]

            def get_step_dist(self):
                return 1.0

        class Kinematics:
            def __init__(self, stepper):
                self.stepper = stepper
                self.cleared = []

            def get_steppers(self):
                return [self.stepper]

            def calc_position(self, positions):
                return [positions["stepper_x"], 0.0, 0.0]

            def clear_homing_state(self, axes):
                self.cleared.append(axes)

        class Endstop:
            def __init__(self, stepper):
                self.stepper = stepper

            def get_steppers(self):
                return [self.stepper]

            def home_start(self, *args, **kwargs):
                return object()

            def home_wait(self, print_time):
                return 0.0

        class Toolhead:
            def __init__(self, kin):
                self.kin = kin
                self.position = [0.0, 0.0, 0.0, 0.0]
                self.moves = []

            def get_position(self):
                return list(self.position)

            def get_kinematics(self):
                return self.kin

            def flush_step_generation(self):
                pass

            def get_last_move_time(self):
                return 1.0

            def dwell(self, delay):
                pass

            def drip_move(self, pos, speed, completion):
                stepper.position = 6
                guard.collision = {"axis": "x", "print_time": 1.0}

            def set_position(self, pos):
                self.position = list(pos)

            def move(self, pos, speed):
                self.moves.append((pos, speed))

        class Guard:
            mode = "abort"
            collision = None
            collision_retract = False
            axis = "x"

            def prepare_move(self, pos, probe_pos):
                return True

            def start_move(self, print_time):
                return object()

            def finish_move(self, was_collision=False):
                self.finished_collision = was_collision

        class Printer:
            command_error = ValueError

            def __init__(self, toolhead, guard):
                self.toolhead = toolhead
                self.guard = guard
                self.events = []

            def lookup_object(self, name, default=None):
                return {"toolhead": self.toolhead,
                        "load_cell_homing_guard": self.guard}.get(name,
                                                                   default)

            def send_event(self, name, *args):
                self.events.append(name)

            def is_shutdown(self):
                return False

        stepper = Stepper()
        kin = Kinematics(stepper)
        guard = Guard()
        toolhead = Toolhead(kin)
        printer = Printer(toolhead, guard)
        endstop = Endstop(stepper)
        move = homing.HomingMove(printer, [(endstop, "x")])
        with mock.patch.object(homing, "multi_complete", return_value=object()):
            with self.assertRaisesRegex(ValueError, "Load cell collision"):
                move.homing_move([10.0, 0.0, 0.0, 0.0], 5.0)
        self.assertEqual(kin.cleared, ["x"])
        self.assertEqual(toolhead.position, [6.0, 0.0, 0.0, 0.0])
        self.assertEqual(toolhead.moves, [])
        self.assertTrue(guard.finished_collision)
        self.assertIn("homing:homing_move_end", printer.events)
        self.assertIn("load_cell_homing:collision", printer.events)


if __name__ == "__main__":
    unittest.main()
