import math
import unittest

from klippy.extras.extrusion_force_calibration import estimate_response_tau
from klippy.extras.extruder_force_current import (
    select_run_current, validate_current_curve)
from klippy.extras.extrusion_force_guard import ExtrusionForceGuardLogic
from klippy.extras.extrusion_force_control import (
    ExtrusionForceControl, SpeedController)
from klippy.extras.extrusion_force_diagnostics import (
    CollisionDetector, force_response_metrics)
from klippy.extras.extrusion_force_monitor import (
    BaselineTracker, EXTRUSION_STEADY, EXTRUSION_TRANSIENT,
    ExponentialFilter, ExtrusionForceMonitor, ExtrusionForceProcessor,
    MotionClassifier, TrapQMotionProvider, replay_rows)
from klippy.extras.extrusion_force_profile import (
    ForceProfile, ForceProfileManager, detect_knee)
from klippy.extras.filament_profile import (
    FilamentProfile, FilamentProfileManager)


class FakeProfile:
    name = "test"
    response_tau_rise = 0.2
    response_tau_fall = 0.4

    def expected_force(self, flow, temperature):
        if not 1.0 <= flow <= 10.0 or not 190.0 <= temperature <= 260.0:
            return None
        return flow * 100.0


def profile_with_points(points, tolerance=2.0):
    profile = object.__new__(ForceProfile)
    profile.points = points
    profile.temperature_tolerance = tolerance
    profile.flow_limits = {}
    return profile


class FilterAndBaselineTest(unittest.TestCase):
    def test_exponential_filter_constant_step_and_spike(self):
        force_filter = ExponentialFilter(0.2)
        self.assertEqual(force_filter.update(10.0, 0.0), 10.0)
        self.assertEqual(force_filter.update(10.0, 0.1), 10.0)
        stepped = force_filter.update(110.0, 0.2)
        self.assertGreater(stepped, 10.0)
        self.assertLess(stepped, 110.0)
        spiked = force_filter.update(1010.0, 0.21)
        self.assertLess(spiked, 110.0)

    def test_baseline_tracks_drift_and_freezes_during_extrusion(self):
        tracker = BaselineTracker(1.0, 0.2, 20.0, 5.0)
        self.assertEqual(tracker.update(100.0, 0.0, "IDLE", 1.0), 100.0)
        drifted = tracker.update(102.0, 1.0, "TRAVEL", 1.0)
        self.assertGreater(drifted, 100.0)
        frozen = tracker.update(
            250.0, 1.1, "EXTRUSION_STEADY", 1.0)
        self.assertEqual(frozen, drifted)
        self.assertEqual(tracker.update(500.0, 1.5, "TRAVEL", 1.0), frozen)

    def test_motion_classifier_requires_stable_positive_flow(self):
        classifier = MotionClassifier(0.001, 0.01, 0.5, 0.3, 0.1)
        self.assertEqual(
            classifier.classify(0.0, 1.0, 0.0, 2.0),
            EXTRUSION_TRANSIENT)
        self.assertEqual(
            classifier.classify(0.4, 1.0, 0.0, 2.0),
            EXTRUSION_STEADY)
        self.assertEqual(
            classifier.classify(0.5, 2.0, 0.0, 4.0),
            EXTRUSION_TRANSIENT)
        self.assertEqual(classifier.classify(0.6, -1.0, 0.0, 0.0), "RETRACT")
        self.assertEqual(classifier.classify(0.7, 0.0, 10.0, 0.0), "TRAVEL")

    def test_processor_replay_uses_same_model_and_dynamic_force(self):
        rows = [
            {"print_time": 0.0, "force": 100.0, "flow": 0.0,
             "temperature": 220.0},
            {"print_time": 1.0, "force": 300.0, "flow": 2.0,
             "e_velocity": 1.0, "temperature": 220.0},
            {"print_time": 1.4, "force": 310.0, "flow": 2.0,
             "e_velocity": 1.0, "temperature": 220.0},
        ]
        processor = ExtrusionForceProcessor(transient_time=0.3)
        states = replay_rows(rows, processor, lambda extruder: FakeProfile())
        self.assertEqual(states[-1]["motion_state"], EXTRUSION_STEADY)
        self.assertEqual(states[-1]["expected_force_g"], 200.0)
        self.assertIsNotNone(states[-1]["excess_force_g"])


class ProfileTest(unittest.TestCase):
    def setUp(self):
        self.profile = profile_with_points([
            {"temperature": 220.0, "flow": 2.0, "mean_force": 300.0},
            {"temperature": 220.0, "flow": 6.0, "mean_force": 700.0},
            {"temperature": 240.0, "flow": 2.0, "mean_force": 200.0},
            {"temperature": 240.0, "flow": 6.0, "mean_force": 500.0},
        ])

    def test_interpolates_flow_then_temperature(self):
        self.assertEqual(self.profile.expected_force(4.0, 220.0), 500.0)
        self.assertEqual(self.profile.expected_force(4.0, 230.0), 425.0)

    def test_does_not_extrapolate(self):
        self.assertIsNone(self.profile.expected_force(7.0, 230.0))
        self.assertIsNone(self.profile.expected_force(4.0, 250.0))

    def test_single_temperature_has_bounded_tolerance(self):
        profile = profile_with_points([
            {"temperature": 220.0, "flow": 2.0, "mean_force": 300.0},
            {"temperature": 220.0, "flow": 4.0, "mean_force": 500.0},
        ])
        self.assertEqual(profile.expected_force(3.0, 221.0), 400.0)
        self.assertIsNone(profile.expected_force(3.0, 223.0))

    def test_knee_detection_and_linear_rejection(self):
        knee = detect_knee([
            (1, 100), (2, 200), (3, 300), (4, 420),
            (5, 800), (6, 1300), (7, 1900)], 2.0)
        self.assertIsNotNone(knee)
        self.assertGreaterEqual(knee["flow"], 3.0)
        self.assertIsNone(detect_knee([
            (1, 100), (2, 200), (3, 300), (4, 400), (5, 500)]))

    def test_knee_detection_handles_noise_and_plateau(self):
        knee = detect_knee([
            (1, 102), (2, 195), (3, 306), (4, 398),
            (5, 720), (6, 1090), (7, 1510)])
        self.assertIsNotNone(knee)
        plateau = [(1, 100), (2, 200), (3, 300),
                   (4, 305), (5, 302), (6, 304)]
        self.assertIsNone(detect_knee(plateau))


class ProfileExtruderAssignmentTest(unittest.TestCase):
    class Gcmd:
        def __init__(self, **params):
            self.params = params
            self.responses = []

        def get(self, name, default=None):
            return self.params.get(name, default)

        def error(self, message):
            return ValueError(message)

        def respond_info(self, message):
            self.responses.append(message)

    class Printer:
        def __init__(self):
            self.events = []
            self.objects = {}

        def config_error(self, message):
            return ValueError(message)

        def send_event(self, event, *args):
            self.events.append((event,) + args)

        def lookup_object(self, name, default=None):
            return self.objects.get(name, default)

    def make_profile(self, name, extruders):
        profile = object.__new__(ForceProfile)
        profile.name = name
        profile.extruders = tuple(extruders)
        profile.extruder = profile.extruders[0]
        profile.valid_for = "filament"
        profile.filament_profile = None
        profile.compatibility_errors = {
            extruder: [] for extruder in profile.extruders}
        return profile

    def make_manager(self):
        manager = object.__new__(ForceProfileManager)
        manager.printer = self.Printer()
        manager.profiles = {}
        manager.active = {}
        manager.filament_manager = None
        return manager

    def test_resolve_requires_extruder_for_shared_profile(self):
        profile = self.make_profile("shared", ("extruder", "extruder1"))
        with self.assertRaisesRegex(ValueError, "specify EXTRUDER"):
            profile.resolve_extruder(self.Gcmd())
        self.assertEqual(
            profile.resolve_extruder(self.Gcmd(EXTRUDER="extruder1")),
            "extruder1")
        with self.assertRaisesRegex(ValueError, "does not apply"):
            profile.resolve_extruder(self.Gcmd(EXTRUDER="extruder2"))

    def test_single_profile_keeps_implicit_extruder(self):
        profile = self.make_profile("single", ("extruder",))
        self.assertEqual(profile.resolve_extruder(self.Gcmd()), "extruder")

    def test_profiles_are_inactive_until_filament_is_selected(self):
        manager = self.make_manager()
        profile = self.make_profile(
            "shared", ("extruder", "extruder1"))
        manager.add_profile(profile)
        self.assertIsNone(manager.get_active("extruder"))
        self.assertIsNone(manager.get_active("extruder1"))

    def test_set_without_extruder_assigns_all_supported_extruders(self):
        manager = self.make_manager()
        initial = self.make_profile(
            "initial", ("extruder", "extruder1"))
        selected = self.make_profile(
            "selected", ("extruder", "extruder1"))
        manager.add_profile(initial)
        manager.add_profile(selected)
        gcmd = self.Gcmd(PROFILE="selected")
        manager.cmd_SET_PROFILE(gcmd)
        self.assertIs(manager.get_active("extruder"), selected)
        self.assertIs(manager.get_active("extruder1"), selected)
        self.assertEqual(len(manager.printer.events), 2)

    def test_set_with_extruder_changes_only_requested_extruder(self):
        manager = self.make_manager()
        initial = self.make_profile(
            "initial", ("extruder", "extruder1"))
        selected = self.make_profile(
            "selected", ("extruder", "extruder1"))
        manager.add_profile(initial)
        manager.add_profile(selected)
        manager.active["extruder"] = initial
        manager.active["extruder1"] = initial
        manager.cmd_SET_PROFILE(self.Gcmd(
            PROFILE="selected", EXTRUDER="extruder1"))
        self.assertIs(manager.get_active("extruder"), initial)
        self.assertIs(manager.get_active("extruder1"), selected)

    def test_set_rejects_extruder_outside_profile(self):
        manager = self.make_manager()
        profile = self.make_profile("shared", ("extruder", "extruder1"))
        manager.add_profile(profile)
        with self.assertRaisesRegex(ValueError, "does not apply"):
            manager.cmd_SET_PROFILE(self.Gcmd(
                PROFILE="shared", EXTRUDER="extruder2"))

    def test_set_rejects_hardware_mismatch(self):
        manager = self.make_manager()
        profile = self.make_profile("wrong_nozzle", ("extruder",))
        profile.compatibility_errors["extruder"] = [
            "nozzle_diameter 0.6 does not match 0.4"]
        manager.add_profile(profile)
        with self.assertRaisesRegex(ValueError, "incompatible"):
            manager.cmd_SET_PROFILE(self.Gcmd(PROFILE="wrong_nozzle"))

    def test_check_extruder_compares_nozzle(self):
        profile = self.make_profile("hardware", ("extruder",))
        profile.nozzle_diameter = 0.6
        extruder = type("Extruder", (), {
            "nozzle_diameter": 0.4,
        })()
        errors = profile.check_extruder("extruder", extruder)
        self.assertEqual(len(errors), 1)
        self.assertIn("nozzle_diameter", profile.compatibility_error(
            "extruder"))


class FilamentProfileSelectionTest(unittest.TestCase):
    class Gcmd(ProfileExtruderAssignmentTest.Gcmd):
        pass

    class Printer(ProfileExtruderAssignmentTest.Printer):
        pass

    def make_managers(self):
        printer = self.Printer()
        extruder = type("Extruder", (), {
            "filament_area": 1.0,
            "get_heater": lambda self: None,
        })()
        printer.objects["extruder"] = extruder

        force_manager = object.__new__(ForceProfileManager)
        force_manager.printer = printer
        force_manager.profiles = {}
        force_manager.active = {}
        force_manager.filament_manager = None
        printer.objects["extrusion_force_profile_manager"] = force_manager

        filament_manager = object.__new__(FilamentProfileManager)
        filament_manager.printer = printer
        filament_manager.profiles = {}
        filament_manager.active = {}
        printer.objects["filament_profile_manager"] = filament_manager
        force_manager.filament_manager = filament_manager
        return printer, filament_manager, force_manager

    def make_filament(self, name):
        filament = object.__new__(FilamentProfile)
        filament.name = name
        filament.material = "ASA"
        filament.max_material_temperature = 255.0
        filament.filament_diameter = 1.70
        filament.filament_area = math.pi * (1.70 * 0.5) ** 2
        return filament

    def make_force_profile(self, name, valid_for, compatible=True):
        profile = object.__new__(ForceProfile)
        profile.name = name
        profile.extruders = ("extruder",)
        profile.extruder = "extruder"
        profile.valid_for = valid_for
        profile.filament_profile = None
        profile.compatibility_errors = {
            "extruder": [] if compatible else ["hardware mismatch"]}
        return profile

    def test_portable_selection_activates_matching_force_profile(self):
        _, filament_manager, force_manager = self.make_managers()
        filament = self.make_filament("F01_ASA_Apollox")
        profile = self.make_force_profile(
            "rf2000_asa", "F01_ASA_Apollox")
        profile.bind_filament_profile(filament)
        filament_manager.add_profile(filament)
        force_manager.add_profile(profile)

        filament_manager.cmd_SET_FILAMENT_PROFILE(self.Gcmd(
            PROFILE="F01_ASA_Apollox", EXTRUDER="extruder"))

        self.assertIs(filament_manager.get_active("extruder"), filament)
        self.assertIs(force_manager.get_active("extruder"), profile)

    def test_selection_disables_stale_force_profile_without_match(self):
        _, filament_manager, force_manager = self.make_managers()
        filament = self.make_filament("F01_ASA_Apollox")
        stale = self.make_force_profile("stale", "different")
        filament_manager.add_profile(filament)
        force_manager.active["extruder"] = stale

        filament_manager.cmd_SET_FILAMENT_PROFILE(self.Gcmd(
            PROFILE="F01_ASA_Apollox", EXTRUDER="extruder"))

        self.assertIs(filament_manager.get_active("extruder"), filament)
        self.assertIsNone(force_manager.get_active("extruder"))

    def test_active_filament_overrides_nominal_area(self):
        _, filament_manager, _ = self.make_managers()
        filament = self.make_filament("measured")
        filament_manager.add_profile(filament)
        filament_manager.activate("extruder", filament)
        self.assertAlmostEqual(
            filament_manager.get_filament_area("extruder", 123.0),
            math.pi * (1.70 * 0.5) ** 2)
        self.assertEqual(
            filament_manager.get_filament_area("extruder1", 123.0),
            123.0)

    def test_monitor_uses_active_filament_diameter_for_flow(self):
        _, filament_manager, _ = self.make_managers()
        filament = self.make_filament("measured")
        filament_manager.add_profile(filament)
        filament_manager.activate("extruder", filament)
        nominal_area = math.pi * (1.75 * 0.5) ** 2
        extruder = type("Extruder", (), {
            "filament_area": nominal_area,
            "get_status": lambda self, eventtime: {
                "temperature": 220.0, "target": 220.0},
        })()
        monitor = object.__new__(ExtrusionForceMonitor)
        monitor.extruders = {"extruder": extruder}
        monitor.reactor = type("Reactor", (), {
            "monotonic": lambda self: 0.0})()
        monitor._motion_at = lambda print_time: {
            "extruder": "extruder", "e_velocity": 2.0,
            "e_position": 1.0, "xy_velocity": 10.0}
        monitor.filament_manager = filament_manager
        monitor.profile_manager = None
        monitor.processor = type("Processor", (), {
            "process": lambda self, observation, profile: observation})()
        monitor.last_callback_time = 1.0
        monitor.callback_interval = 0.1

        monitor._handle_sample({"print_time": 1.0})

        expected_area = math.pi * (1.70 * 0.5) ** 2
        self.assertAlmostEqual(
            monitor.latest["flow_mm3_s"], 2.0 * expected_area)
        self.assertAlmostEqual(
            monitor.latest["filament_diameter"], 1.70)

    def test_selection_rejects_ambiguous_force_profiles(self):
        _, filament_manager, force_manager = self.make_managers()
        filament = self.make_filament("F01_ASA_Apollox")
        filament_manager.add_profile(filament)
        force_manager.add_profile(self.make_force_profile(
            "first", filament.name))
        force_manager.add_profile(self.make_force_profile(
            "second", filament.name))

        with self.assertRaisesRegex(ValueError, "Multiple compatible"):
            filament_manager.cmd_SET_FILAMENT_PROFILE(self.Gcmd(
                PROFILE=filament.name, EXTRUDER="extruder"))


class ResponseTest(unittest.TestCase):
    def test_estimates_first_order_rise_and_fall(self):
        tau = 0.5
        rise = [(index * 0.05,
                 100.0 + 900.0 * (1.0 - math.exp(-index * 0.05 / tau)))
                for index in range(41)]
        fall = [(index * 0.05,
                 100.0 + 900.0 * math.exp(-index * 0.05 / tau))
                for index in range(41)]
        self.assertAlmostEqual(
            estimate_response_tau(rise, 100.0, 1000.0), tau, places=2)
        self.assertAlmostEqual(
            estimate_response_tau(fall, 1000.0, 100.0, rising=False),
            tau, places=2)


def guard_state(sample_time, force=100.0, expected=100.0, excess=0.0,
                e_position=0.0, motion="EXTRUSION_STEADY"):
    return {
        "print_time": sample_time,
        "motion_state": motion,
        "flow_mm3_s": 2.0,
        "expected_dynamic_force_g": expected,
        "force_control_g": force,
        "force_trend_g": force,
        "excess_force_g": excess,
        "confidence": 1.0,
        "e_velocity": 2.0,
        "e_position": e_position,
    }


class GuardAndCurrentTest(unittest.TestCase):
    def make_guard(self):
        guard = ExtrusionForceGuardLogic(
            minimum_monitor_flow=1.0,
            minimum_expected_force=50.0,
            minimum_confidence=0.8,
            underload_ratio=0.5,
            underload_time=0.5,
            underload_filament_length=1.0,
            soft_force_margin=200.0,
            hard_force_margin=400.0,
            hard_overload_time=0.25)
        guard.set_enabled(True)
        return guard

    def test_delivery_failure_needs_time_and_filament_distance(self):
        guard = self.make_guard()
        self.assertIsNone(guard.update(guard_state(
            0.0, force=20.0, e_position=0.0))[1])
        self.assertIsNone(guard.update(guard_state(
            0.3, force=20.0, e_position=0.6))[1])
        events, fault = guard.update(guard_state(
            0.6, force=20.0, e_position=1.2))
        self.assertEqual(fault, "DELIVERY_FAILURE")
        self.assertIn("delivery_failure", events)

    def test_retract_and_transient_do_not_trigger_runout(self):
        guard = self.make_guard()
        for index in range(10):
            events, fault = guard.update(guard_state(
                index * 0.2, force=0.0, expected=100.0,
                e_position=-index, motion="RETRACT"))
            self.assertIsNone(fault)
            self.assertNotIn("delivery_failure", events)

    def test_short_underload_recovers_without_fault(self):
        guard = self.make_guard()
        guard.update(guard_state(0.0, force=20.0, e_position=0.0))
        guard.update(guard_state(0.2, force=20.0, e_position=0.4))
        events, fault = guard.update(guard_state(
            0.3, force=100.0, e_position=0.6))
        self.assertIsNone(fault)
        self.assertNotIn("delivery_failure", events)

    def test_persistent_hard_overload_becomes_jam(self):
        guard = self.make_guard()
        events, fault = guard.update(guard_state(
            0.0, force=600.0, excess=500.0))
        self.assertIsNone(fault)
        self.assertIn("hard_overload", events)
        events, fault = guard.update(guard_state(
            0.3, force=600.0, excess=500.0, e_position=0.5))
        self.assertEqual(fault, "JAM")
        self.assertIn("jam", events)

    def test_current_selection_uses_force_reserve_and_grind_ceiling(self):
        curve = [
            {"current": 0.4, "stable_force": 900.0},
            {"current": 0.5, "stable_force": 1300.0},
            {"current": 0.6, "stable_force": 1700.0},
        ]
        self.assertEqual(select_run_current(curve, 1000.0, 1.2), 0.5)
        self.assertIsNone(select_run_current(
            curve, 1000.0, 1.2, grind_force_limit=1400.0,
            grind_safety_factor=0.9))
        self.assertTrue(validate_current_curve(curve))
        self.assertFalse(validate_current_curve([
            {"current": 0.4, "stable_force": 1000.0},
            {"current": 0.5, "stable_force": 1010.0},
            {"current": 0.6, "stable_force": 1005.0},
        ]))


class ControllerAndTransformTest(unittest.TestCase):
    def make_controller(self):
        controller = SpeedController(
            minimum_speed_factor=0.5,
            speed_down_rate=0.5,
            speed_up_rate=0.1,
            soft_force_margin=200.0,
            hard_force_margin=500.0,
            recovery_delay=0.5,
            hysteresis=50.0,
            overload_debounce=0.2,
            minimum_confidence=0.7,
            control_interval=0.1)
        controller.set_enabled(True)
        return controller

    def test_overload_hysteresis_recovery_and_hard_fault(self):
        controller = self.make_controller()
        state = guard_state(0.0, excess=250.0)
        controller.update(state)
        self.assertEqual(controller.speed_factor, 1.0)
        state["print_time"] = 0.3
        controller.update(state)
        self.assertEqual(controller.state, "LIMITING")
        self.assertLess(controller.speed_factor, 1.0)
        limited = controller.speed_factor
        state.update({"print_time": 1.0, "excess_force_g": 100.0})
        controller.update(state)
        state["print_time"] = 1.6
        controller.update(state)
        self.assertEqual(controller.state, "RECOVERY")
        self.assertGreater(controller.speed_factor, limited)
        state.update({"print_time": 2.0, "excess_force_g": 600.0})
        controller.update(state)
        self.assertEqual(controller.state, "HARD_FAULT")
        self.assertEqual(controller.speed_factor, 0.5)

    def test_transform_changes_only_positive_extrusion_speed(self):
        class FakeTransform:
            def __init__(self):
                self.position = [0.0, 0.0, 0.0, 0.0]
                self.moves = []

            def get_position(self):
                return list(self.position)

            def move(self, position, speed):
                self.moves.append((list(position), speed))
                self.position = list(position)

        transform = FakeTransform()
        control = object.__new__(ExtrusionForceControl)
        control.normal_transform = transform
        control.controller = self.make_controller()
        control.controller.speed_factor = 0.5
        control.move([10.0, 0.0, 0.0, 1.0], 100.0)
        control.move([20.0, 0.0, 0.0, 1.0], 100.0)
        control.move([20.0, 0.0, 0.0, 0.0], 100.0)
        self.assertEqual([move[1] for move in transform.moves],
                         [50.0, 100.0, 100.0])

    def test_controller_never_drops_below_minimum(self):
        controller = self.make_controller()
        for index in range(20):
            controller.update(guard_state(
                index * 0.3, force=400.0, excess=300.0,
                e_position=index))
        self.assertEqual(controller.speed_factor, 0.5)


class TrapQSynchronizationTest(unittest.TestCase):
    class Move:
        print_time = 1.0
        move_t = 2.0
        start_v = 2.0
        accel = 1.0
        start_x = 5.0
        start_y = 0.0
        start_z = 0.0
        x_r = -1.0
        y_r = 0.0
        z_r = 0.0

    class FfiMain:
        def new(self, declaration):
            return [TrapQSynchronizationTest.Move()]

    class FfiLib:
        def trapq_extract_old(self, trapq, data, count, start, end):
            return 1

    def test_signed_extruder_velocity_at_sample_print_time(self):
        provider = object.__new__(TrapQMotionProvider)
        provider.name = "extruder1"
        provider.trapq = object()
        provider.extruder = True
        provider.ffi_main = self.FfiMain()
        provider.ffi_lib = self.FfiLib()
        state = provider.get_state(2.0)
        self.assertEqual(state["extruder"], "extruder1")
        self.assertEqual(state["e_position"], 2.5)
        self.assertEqual(state["e_velocity"], -3.0)

    def test_monitor_selects_extruder_that_moves_at_sample_time(self):
        class Provider:
            def __init__(self, state):
                self.state = state

            def get_state(self, print_time):
                return dict(self.state)

        monitor = object.__new__(ExtrusionForceMonitor)
        monitor.extruder_providers = {
            "extruder": Provider({"extruder": "extruder",
                                  "e_position": 10.0, "e_velocity": 0.0}),
            "extruder1": Provider({"extruder": "extruder1",
                                   "e_position": 20.0, "e_velocity": 2.0}),
        }
        monitor.toolhead_provider = Provider({"xy_velocity": 30.0})
        monitor.processor = type("Processor", (), {
            "classifier": type("Classifier", (), {
                "extrusion_epsilon": 0.001})()})()
        monitor.last_extruder = None
        state = monitor._motion_at(5.0)
        self.assertEqual(state["extruder"], "extruder1")
        self.assertEqual(state["xy_velocity"], 30.0)


class ExperimentalDiagnosticsTest(unittest.TestCase):
    def test_force_response_metrics(self):
        samples = [(0.0, 100.0), (0.1, 500.0), (0.2, 900.0),
                   (0.3, 1050.0), (0.4, 1000.0), (0.5, 1000.0)]
        metrics = force_response_metrics(samples, 100.0, 1000.0, 0.06)
        self.assertIsNotNone(metrics["tau"])
        self.assertEqual(metrics["overshoot_g"], 50.0)
        self.assertEqual(metrics["settling_time"], 0.3)

    def test_collision_detector_is_log_only_signal_qualified(self):
        detector = CollisionDetector(100.0, 500.0, 1.0, 0.1, 0.5)
        state = guard_state(1.0, motion="TRAVEL")
        state.update({"flow_mm3_s": 0.0, "xy_velocity": 20.0,
                      "force_fast_g": 300.0, "force_trend_g": 100.0,
                      "dforce_dt": 1000.0})
        self.assertTrue(detector.update(state))
        state["print_time"] = 1.1
        self.assertFalse(detector.update(state))
        state.update({"print_time": 2.0,
                      "motion_state": "EXTRUSION_TRANSIENT"})
        self.assertFalse(detector.update(state))


if __name__ == "__main__":
    unittest.main()
