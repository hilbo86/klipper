"""Power-off boundary measurement interlocks and temporary Cartesian limits."""
import io
from types import SimpleNamespace
import unittest

from laser_tool_harness import Machine, MacroError, measured_profile, move, select


class BoundaryTests(unittest.TestCase):
    def machine(self, tool='BLUE_LASER'):
        m = Machine(tool_options={'boundary_x_max': 270., 'boundary_y_max': 298.,
                                  'boundary_z_min': -2.})
        if tool != 'BLUE_LASER':
            select(m, {'IR_LASER': 57.2, 'DREMEL': 16.8}[tool])
        return m

    def test_native_limit_is_extended_for_one_step_then_restored(self):
        m = self.machine()
        head = m.objects['toolhead']
        head.position[0] = 266.
        observed = []
        m.tool_manager.base_check_move = lambda motion: observed.append(list(head.kin.limits))
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
        m.run('TOOL_BOUNDS_MOVE POSITION=267 SPEED=2')
        self.assertEqual(head.position[:3], [267., 0., 38.])
        self.assertEqual(observed, [[(0., 270.), (0., 292.), (0., 38.)]])
        self.assertEqual(head.kin.limits, [(0., 266.), (0., 292.), (0., 38.)])
        self.assertFalse(m.tool_manager.granted)

    def test_failed_native_move_also_restores_limits(self):
        m = self.machine()
        head = m.objects['toolhead']
        def fail(motion):
            raise MacroError('native move failed')
        m.tool_manager.base_check_move = fail
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
        with self.assertRaisesRegex(MacroError, 'native'):
            m.run('TOOL_BOUNDS_MOVE POSITION=1')
        self.assertEqual(head.kin.limits[0], (0., 266.))
        self.assertFalse(m.tool_manager.boundary_moving)

    def test_selected_axis_and_one_mm_step_remain_bounded(self):
        m = self.machine()
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=Y LIMIT=298')
        for command in ('TOOL_BOUNDS_MOVE POSITION=2', 'TOOL_BOUNDS_MOVE POSITION=-.1',
                        'TOOL_BOUNDS_MOVE POSITION=nan', 'TOOL_BOUNDS_MOVE POSITION=1 SPEED=11'):
            with self.subTest(command=command), self.assertRaises(MacroError):
                m.run(command)
        with self.assertRaises(MacroError):
            m.tool_manager.check_move(move([0, 0, 38], [1, 0, 38]))
        m.run('TOOL_BOUNDS_MOVE POSITION=1')
        self.assertEqual(m.objects['toolhead'].position[:3], [0., 1., 38.])

    def test_approach_uses_positioning_speed_and_preserves_normal_limits(self):
        for tool, axis, target in (('BLUE_LASER', 'X', 251.),
                                   ('BLUE_LASER', 'Y', 277.),
                                   ('IR_LASER', 'Y', 192.), ('DREMEL', 'Y', 192.)):
            with self.subTest(tool=tool, axis=axis):
                m = self.machine(tool)
                head = m.objects['toolhead']
                original = list(head.kin.limits)
                m.run('TOOL_BOUNDS_BEGIN TOOL=%s AXIS=%s LIMIT=%d'
                      % (tool, axis, 270 if axis == 'X' else 298))
                m.run('TOOL_BOUNDS_APPROACH POSITION=%s' % target)
                self.assertEqual(head.moves[-1][1], 20.)
                self.assertEqual(head.position['XYZ'.index(axis)], target)
                self.assertEqual(head.kin.limits, original)
                self.assertEqual(head.position[2], 38.)
                self.assertFalse(m.tool_manager.boundary_approaching)
                with self.assertRaises(MacroError):
                    m.run('TOOL_BOUNDS_APPROACH POSITION=%s' % target)
                with self.assertRaises(MacroError):
                    m.run('TOOL_BOUNDS_MOVE POSITION=%s' % (target + 2))
                m.run('TOOL_BOUNDS_MOVE POSITION=%s SPEED=5' % (target + 1))
                self.assertEqual(head.moves[-1][1], 5.)

    def test_approach_respects_lower_current_velocity_limit(self):
        m = self.machine()
        head = m.objects['toolhead']
        head.data['max_velocity'] = 10.
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
        m.run('TOOL_BOUNDS_APPROACH POSITION=251')
        self.assertEqual(head.moves[-1][1], 10.)

    def test_approach_rejects_z_outside_native_range_and_after_search(self):
        for fault in ('Z', 'outside', 'negative', 'nonfinite', 'after_step', 'pressed'):
            with self.subTest(fault=fault):
                m = self.machine()
                axis = 'Z' if fault == 'Z' else 'X'
                m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=%s LIMIT=%d'
                      % (axis, -2 if axis == 'Z' else 270))
                if fault == 'after_step':
                    m.run('TOOL_BOUNDS_MOVE POSITION=1')
                elif fault == 'pressed':
                    m.objects['gcode_button X_MAX'].data['state'] = 'PRESSED'
                target = {'outside': '267', 'negative': '-1', 'nonfinite': 'nan'}.get(fault, '251')
                moves_before = len(m.objects['toolhead'].moves)
                with self.assertRaises(MacroError):
                    m.run('TOOL_BOUNDS_APPROACH POSITION=' + target)
                self.assertEqual(len(m.objects['toolhead'].moves), moves_before)
                self.assertFalse(m.tool_manager.boundary_approaching)

    def test_approach_monitor_trips_on_early_trigger_and_keeps_limits(self):
        m = self.machine()
        head = m.objects['toolhead']
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
        def trigger_during_wait():
            m.objects['gcode_button X_MAX'].data['state'] = 'PRESSED'
            m.tool_manager._poll(m.now)
        head.wait_moves = trigger_during_wait
        with self.assertRaises(MacroError):
            m.run('TOOL_BOUNDS_APPROACH POSITION=251')
        self.assertIn('Grenztaster vor Suchstart', m.shutdown_message)
        self.assertEqual(head.kin.limits[0], (0., 266.))
        self.assertFalse(m.tool_manager.boundary_approaching)
        self.assertIsNone(m.tool_manager.boundary)

    def test_failed_approach_restores_motion_guard(self):
        m = self.machine()
        head = m.objects['toolhead']
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
        def fail(motion):
            raise MacroError('native move failed')
        m.tool_manager.base_check_move = fail
        with self.assertRaisesRegex(MacroError, 'native'):
            m.run('TOOL_BOUNDS_APPROACH POSITION=251')
        self.assertFalse(m.tool_manager.boundary_approaching)
        self.assertEqual(head.kin.limits[0], (0., 266.))

    def test_trigger_allows_retreat_but_never_forward(self):
        m = self.machine()
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=Y LIMIT=298')
        m.run('TOOL_BOUNDS_MOVE POSITION=1')
        m.objects['gcode_button Y_MAX'].data['state'] = 'PRESSED'
        with self.assertRaisesRegex(MacroError, 'Rueckzug'):
            m.run('TOOL_BOUNDS_MOVE POSITION=1.1')
        m.run('TOOL_BOUNDS_MOVE POSITION=0')

    def test_ir_z_never_unlocks_and_other_axes_do_not_move(self):
        m = self.machine('IR_LASER')
        with self.assertRaises(MacroError):
            m.run('TOOL_BOUNDS_BEGIN TOOL=IR_LASER AXIS=Z LIMIT=-2')
        m.run('TOOL_BOUNDS_BEGIN TOOL=IR_LASER AXIS=Y LIMIT=298')
        m.run('TOOL_BOUNDS_MOVE POSITION=1')
        self.assertEqual(m.objects['toolhead'].position[2], 38.)
        m.tool_manager.boundary_moving = True
        with self.assertRaises(MacroError):
            m.tool_manager.check_move(move([0, 1, 38], [0, 1, 37]))

    def test_z_min_extension_preserves_top_reference(self):
        m = self.machine()
        head = m.objects['toolhead']
        head.position[2] = 0.
        seen = []
        m.tool_manager.base_check_move = lambda motion: seen.append(head.kin.limits[2])
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=Z LIMIT=-2')
        m.run('TOOL_BOUNDS_MOVE POSITION=-1 SPEED=1')
        self.assertEqual(seen, [(-2., 38.)])
        self.assertEqual(head.kin.limits[2], (0., 38.))
        self.assertEqual(head.data['axis_maximum'][2], 38.)

    def test_user_search_caps_reject_larger_extension(self):
        for axis, limit in (('X', 270.1), ('Y', 298.1), ('Z', -2.1)):
            m = self.machine()
            with self.subTest(axis=axis), self.assertRaises(MacroError):
                m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=%s LIMIT=%s' % (axis, limit))
            self.assertIsNone(m.tool_manager.boundary)

    def test_search_extension_is_opt_in_and_never_enlarges_normal_jogging(self):
        m = Machine()
        with self.assertRaises(MacroError):
            m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
        m = self.machine()
        with self.assertRaises(MacroError):
            m.tool_manager.check_move(move([0, 0, 38], [267, 0, 38]))

    def test_measured_bounds_must_fit_persistent_machine_limits(self):
        with self.assertRaises(MacroError):
            Machine(tool_options={'ir_laser_bounds_calibrated': True,
                                  'ir_laser_x_min': 0., 'ir_laser_x_max': 270.,
                                  'ir_laser_y_min': 0., 'ir_laser_y_max': 240.})

    def test_power_job_homing_and_offset_calibration_are_locked(self):
        m = self.machine()
        m.tool_manager.applied_offset = (3., -7.)
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
        for command in ('_SET_PIN PIN=laser VALUE=.1', '_SET_PIN PIN=tool_enable VALUE=1',
                        'TOOL_CALIBRATION_ARM TOOL=BLUE_LASER', 'TOOL_CALIBRATION_BEGIN',
                        'JOB_START TOOL=BLUE_LASER'):
            with self.subTest(command=command), self.assertRaises(MacroError):
                m.run(command)
        m.sd.current_file = io.StringIO('JOB_START TOOL=BLUE_LASER')
        with self.assertRaises(MacroError):
            m.sd.do_resume()
        with self.assertRaises(MacroError):
            m.tool_manager._home_begin(SimpleNamespace(get_axes=lambda: [0]), [])
        self.assertEqual(m.tool_manager.applied_offset, (3., -7.))

    def test_monitor_rejects_id_reference_power_changes_and_timeout(self):
        for fault in ('id', 'reference', 'power', 'timeout', 'other_axis', 'motor_off'):
            m = self.machine()
            idle = m.objects['idle_timeout']
            m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
            if fault == 'id':
                m.objects['analog_input ID'].data['value'] = 31.
                m.now += .1
                m.tool_manager._poll(m.now)
                self.assertIsNone(m.shutdown_message)
                m.now += .1
            elif fault == 'reference':
                m.objects['toolhead'].data['homed_axes'] = ''
            elif fault == 'power':
                m.pins['laser']['value'] = .1
            elif fault == 'timeout':
                m.now += 61.
            elif fault == 'other_axis':
                m.objects['toolhead'].position[1] = 1.
            else:
                m.tool_manager._motor_off()
            m.tool_manager._poll(m.now)
            with self.subTest(fault=fault):
                self.assertIsNotNone(m.shutdown_message)
                self.assertIsNone(m.tool_manager.boundary)
                self.assertEqual(idle.idle_gcode, 'M84')

    def test_measurement_moves_survive_isolated_extreme_id_reports(self):
        for tool, nominal in (('BLUE_LASER', 80.), ('IR_LASER', 57.2),
                              ('DREMEL', 16.8)):
            with self.subTest(tool=tool):
                m = self.machine(tool)
                m.run('TOOL_BOUNDS_BEGIN TOOL=%s AXIS=X LIMIT=270' % tool)
                ignored_before = m.tool_manager.id_ignored_samples
                for index, value in enumerate((0., nominal, nominal, 100.,
                                                nominal, nominal, 31., nominal), 1):
                    m.objects['analog_input ID'].data['value'] = value
                    m.now += .1
                    m.tool_manager._poll(m.now)
                    m.run('TOOL_BOUNDS_MOVE POSITION=%d' % index)
                    status = m.tool_manager.get_status(m.now)
                    self.assertEqual(status['detected_tool'], tool)
                    self.assertEqual(status['id_ignored_samples'] - ignored_before,
                                     (index + 2) // 3)
                    self.assertIsNone(m.shutdown_message)
                    self.assertFalse(m.tool_manager.granted)
                    self.assertTrue(all(m.pins[pin]['value'] == 0 for pin in
                                        ('laser', 'aux_tool', 'tool_enable')))
                m.run('TOOL_BOUNDS_END')
                self.assertIsNone(m.tool_manager.boundary)

    def test_move_guard_rejects_confirmed_id_loss_before_timer_runs(self):
        m = self.machine()
        m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
        m.objects['analog_input ID'].data['value'] = 31.
        m.now += .1
        m.run('TOOL_BOUNDS_MOVE POSITION=1')
        m.now += .1
        with self.assertRaisesRegex(MacroError, 'Werkzeug-ID.*roh=31.0'):
            m.run('TOOL_BOUNDS_MOVE POSITION=2')
        self.assertEqual(m.objects['toolhead'].position[0], 1.)
        m.tool_manager._poll(m.now)
        self.assertIsNotNone(m.shutdown_message)

    def test_finish_inside_restores_idle_outside_also_clears_reference(self):
        for position, expected in ((266., 'xyz'), (267., '')):
            m = self.machine()
            head = m.objects['toolhead']
            head.position[0] = 266.
            m.run('TOOL_BOUNDS_BEGIN TOOL=BLUE_LASER AXIS=X LIMIT=270')
            m.run('TOOL_BOUNDS_MOVE POSITION=%s' % position)
            m.run('TOOL_BOUNDS_END')
            with self.subTest(position=position):
                self.assertEqual(head.data['homed_axes'], expected)
                self.assertIsNone(m.tool_manager.boundary)
                self.assertEqual(m.objects['idle_timeout'].idle_gcode, 'M84')

    def test_measured_bounds_apply_before_offset_calibration_without_power(self):
        options = {'ir_laser_bounds_calibrated': True,
                   'ir_laser_x_min': 0., 'ir_laser_x_max': 260.,
                   'ir_laser_y_min': 0., 'ir_laser_y_max': 240.}
        m = Machine(tool_options=options)
        select(m, 57.2)
        status = m.tool_manager.get_status(m.now)
        self.assertTrue(status['profile_bounds_calibrated'])
        self.assertFalse(status['profile_calibrated'])
        self.assertIsNone(status['geometry']['offset_x'])
        m.tool_manager.check_move(move([0, 0, 38], [5, 5, 38]))
        with self.assertRaises(MacroError):
            m.tool_manager.check_move(move([0, 0, 38], [5, 241, 38]))
        m.run('TOOL_CALIBRATION_BEGIN')
        with self.assertRaises(MacroError):
            m.tool_manager.check_move(move([0, 0, 38], [5, 241, 38]))


if __name__ == '__main__':
    unittest.main()
