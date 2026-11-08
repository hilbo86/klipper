"""Laser tool module tests; run without a configuration checkout or Jinja2.

    python -m unittest discover -s test -p test_laser_tool_manager.py
"""
import io
from types import SimpleNamespace
import unittest

from laser_tool_harness import (
    Machine, MacroError, MODULE, measured_profile, move, select, start_job)


class ToolTests(unittest.TestCase):
    def test_all_id_windows_exclude_both_boundaries(self):
        for low, high, expected in MODULE.ID_WINDOWS:
            with self.subTest(tool=expected):
                self.assertEqual(MODULE.classify_id((low+high)/2), expected)
                self.assertEqual(MODULE.classify_id(low), 'UNKNOWN')
                self.assertEqual(MODULE.classify_id(high), 'UNKNOWN')

    def test_repeated_status_reads_do_not_count_as_new_adc_samples(self):
        m = Machine()
        m.objects['analog_input ID'].data['value'] = 59.09
        m.now += .1
        for _ in range(10):
            status = m.tool_manager.get_status(m.now)
        self.assertEqual(status['stable_samples'], 0)
        self.assertEqual(status['id_filter_samples'], 1)
        self.assertEqual(status['detected_tool'], 'UNSTABLE')
        select(m, 59.09)
        self.assertEqual(m.tool_manager.detected, 'IR_LASER')
        self.assertFalse(m.tool_manager.granted)

    def test_header_contract_precedes_even_modal_and_macro_commands(self):
        for header in ('G90\nJOB_START TOOL=BLUE_LASER', 'job_init\nG28',
                       'M3 S0\nJOB_START TOOL=BLUE_LASER', '; only comment',
                       'JOB_START TOOL=BLUE_LASER OTHER=1'):
            with self.subTest(header=header), self.assertRaises(ValueError):
                MODULE.file_tool(io.StringIO(header))
        stream = io.StringIO('; export\n\nJOB_START TOOL=IR_LASER ; job\nG1 X1')
        stream.seek(2)
        self.assertEqual(MODULE.file_tool(stream), 'IR_LASER')
        self.assertEqual(stream.tell(), 2)

    def test_all_file_start_paths_hit_preflight(self):
        for header in ('G1 X10', 'JOB_START TOOL=IR_LASER', 'JOB_START TOOL=DREMEL'):
            m = Machine()
            m.sd.current_file = io.StringIO(header)
            with self.subTest(header=header), self.assertRaises(MacroError):
                m.sd.do_resume()
            self.assertFalse(m.sd.active)
            self.assertFalse(m.tool_manager.granted)

    def test_none_conflict_unknown_and_stale_never_start(self):
        for value in (31.21, 68.79, 50., 41.82, 0., float('nan')):
            m = Machine()
            select(m, value)
            with self.subTest(value=value), self.assertRaises(MacroError):
                start_job(m)
        m = Machine()
        m.objects['analog_input ID'].mcu_adc.timestamp = .1
        with self.assertRaises(MacroError):
            start_job(m)

    def test_manual_and_renamed_output_commands_need_job_grant(self):
        m = Machine()
        for command in ('SET_PIN PIN=laser VALUE=.1',
                        '_SET_PIN PIN=laser VALUE=.1',
                        'SET_PIN PIN=tool_enable VALUE=1'):
            with self.subTest(command=command), self.assertRaises(MacroError):
                m.run(command)
        m.run('_TOOLS_OFF')
        self.assertEqual(m.pins['laser']['value'], 0.)

    def test_main_cooling_cannot_be_switched_off_while_power_is_granted(self):
        m = Machine()
        m.job()
        for command in ('SET_FAN_SPEED FAN=water_pump SPEED=0',
                        'SET_FAN_SPEED FAN=heat_exchanger SPEED=0',
                        'SET_FAN_SPEED FAN=water_pump TEMPLATE=unsafe'):
            with self.subTest(command=command), self.assertRaises(MacroError):
                m.run(command)
        m.run('_TOOLS_OFF')
        m.run('SET_FAN_SPEED FAN=water_pump SPEED=0')
        with self.assertRaises(MacroError):
            m.run('_TOOL_ARM')

    def test_aux_profiles_require_measurement_and_dremel_requires_cam(self):
        for tool, value in (('IR_LASER', 59.09), ('DREMEL', 17.58)):
            m = Machine()
            select(m, value)
            with self.subTest(tool=tool), self.assertRaises(MacroError):
                start_job(m, tool)
        measured_profile(m, 'DREMEL')
        with self.assertRaises(MacroError):
            start_job(m, 'DREMEL')

    def test_id_loss_shutdown_discards_power_and_does_not_rearm(self):
        for value in (31.21, 68.79, 59.09):
            m = Machine()
            m.job()
            m.objects['analog_input ID'].data['value'] = value
            m.now += .1
            m.tool_manager._poll(m.now)
            self.assertIsNotNone(m.shutdown_message)
            self.assertTrue(all(pin['value'] == 0 for pin in m.pins.values()))
            select(m, 82.12)
            with self.assertRaises(MacroError):
                m.run('_SET_PIN PIN=laser VALUE=.5')

    def test_stale_id_and_active_temperature_failure_shutdown(self):
        for failure in ('id_stale', 'temp_stale', 'temp_open', 'reference'):
            m = Machine()
            m.job()
            if failure == 'id_stale':
                m.objects['analog_input ID'].mcu_adc.timestamp = .01
            elif failure == 'temp_stale':
                m.objects['temperature_sensor Laser'].sensor.mcu_adc.timestamp = -.1
            elif failure == 'temp_open':
                m.objects['temperature_sensor Laser'].sensor.mcu_adc.value = lambda: 1.
            else:
                m.objects['toolhead'].data['homed_axes'] = 'xy'
            m.tool_manager._poll(m.now)
            with self.subTest(failure=failure):
                self.assertIsNotNone(m.shutdown_message)
                self.assertEqual(m.pins['laser']['value'], 0.)

    def test_inactive_open_temperature_does_not_block_main_job(self):
        m = Machine()
        m.objects['temperature_sensor aux_tool'].sensor.mcu_adc.value = lambda: 1.
        m.job()
        m.tool_manager._poll(m.now)
        self.assertIsNone(m.shutdown_message)
        self.assertEqual(m.tool_manager.get_status(m.now)['valid_temperature_sensors'],
                         ['temperature_sensor Laser'])

    def test_profile_change_and_sd_resume_bypass_rejected_while_paused(self):
        m = Machine()
        m.job()
        m.run('PAUSE')
        m.sd.file_position = 100
        with self.assertRaises(MacroError):
            m.sd.do_resume()
        with self.assertRaises(MacroError):
            m.run('JOB_START TOOL=IR_LASER')
        m.run('RESUME')
        self.assertTrue(m.tool_manager.granted)

    def test_sd_pause_and_reset_cannot_leave_a_power_grant(self):
        m = Machine()
        m.job()
        with self.assertRaises(MacroError):
            m.sd._reset_file()
        m.run('M25')
        self.assertFalse(m.tool_manager.granted)
        self.assertEqual(m.pins['laser']['value'], 0.)
        self.assertTrue(m.objects['pause_resume'].is_paused)
        m.run('RESUME')
        m.sd.do_pause()
        self.assertFalse(m.tool_manager.granted)
        self.assertEqual(m.pins['laser']['value'], 0.)

    def test_failed_resume_does_not_leave_a_bypass_token(self):
        m = Machine()
        m.job()
        m.run('PAUSE')
        m.sd.file_position = 100
        m.run('AIR_ASSIST_ON')
        m.run('_TOOL_RESUME_CHECK')
        m.tool_manager._command_error()
        with self.assertRaises(MacroError):
            m.sd.do_resume()

    def test_ir_z_guard_and_profile_bounds_use_machine_positions(self):
        m = Machine()
        select(m, 59.09)
        measured_profile(m, 'IR_LASER')
        for target in ([10, 10, 37], [251, 10, 38], [10, 241, 38], [4, 10, 38]):
            with self.subTest(target=target), self.assertRaises(MacroError):
                m.tool_manager.check_move(move([10, 10, 38], target))
        with self.assertRaises(MacroError):
            m.tool_manager.check_move(move([0, 10, 38], [0, 20, 38]))
        m.tool_manager.check_move(move([10, 10, 38], [20, 20, 38]))
        m.objects['analog_input ID'].data['value'] = 31.21
        m.tool_manager._update_id(m.now)
        with self.assertRaises(MacroError):
            m.tool_manager.check_move(move([10, 10, 38], [10, 10, 37]))

    def test_ir_power_requires_z_top_and_motor_reference(self):
        m = Machine()
        select(m, 59.09)
        measured_profile(m, 'IR_LASER')
        m.objects['toolhead'].position[2] = 37.
        with self.assertRaises(MacroError):
            start_job(m, 'IR_LASER')
        self.assertFalse(m.tool_manager.granted)

    def test_job_enters_measured_lower_bounds_before_power_and_rejects_outside_power(self):
        m = Machine()
        select(m, 59.09)
        measured_profile(m, 'IR_LASER')
        start_job(m, 'IR_LASER')
        self.assertEqual(m.objects['toolhead'].position[:2], [5., 2.])
        m.objects['toolhead'].position[0] = 0.
        with self.assertRaises(MacroError):
            m.run('_SET_PIN PIN=aux_tool VALUE=.04')

    def test_eof_without_job_finish_revokes_power(self):
        m = Machine()
        m.job()
        m.sd.current_file, m.sd.active = None, False
        m.tool_manager._poll(m.now)
        self.assertFalse(m.tool_manager.granted)
        self.assertIsNone(m.tool_manager.job_tool)
        self.assertEqual(m.pins['laser']['value'], 0.)

    def test_calibration_preserves_xy_idle_action_and_invalidates_motor_off(self):
        m = Machine()
        m.print_stats['state'] = 'standby'
        idle = m.objects['idle_timeout']
        original = idle.idle_gcode
        m.run('TOOL_CALIBRATION_BEGIN')
        self.assertEqual(idle.idle_gcode, '_TOOLS_OFF')
        m.run('TOOL_CALIBRATION_ARM TOOL=BLUE_LASER')
        m.power(20/255)
        with self.assertRaises(MacroError):
            m.power(128/255)
        m.run('TOOL_CALIBRATION_HOLD')
        self.assertEqual(m.pins['laser']['value'], 0.)
        with self.assertRaises(MacroError):
            m.tool_manager.check_move(move([0, 0, 38], [1, 0, 38]))
        select(m, 59.09)
        m.tool_manager._home_begin(SimpleNamespace(get_axes=lambda: [2]), [])
        m.tool_manager._home_end(None, [])
        m.run('TOOL_CALIBRATION_ARM TOOL=IR_LASER')
        m.power(20/255)
        m.run('_TOOLS_OFF')
        m.tool_manager._motor_off()
        with self.assertRaises(MacroError):
            m.run('TOOL_CALIBRATION_ARM TOOL=IR_LASER')
        m.run('TOOL_CALIBRATION_END')
        self.assertEqual(idle.idle_gcode, original)
        self.assertFalse(m.tool_manager.calibrating)

    def test_calibration_expiration_revokes_buffered_power(self):
        m = Machine()
        m.print_stats['state'] = 'standby'
        m.run('TOOL_CALIBRATION_BEGIN')
        m.run('TOOL_CALIBRATION_ARM TOOL=BLUE_LASER')
        m.power(20/255)
        m.now += 10.
        m.tool_manager._poll(m.now)
        self.assertIsNotNone(m.shutdown_message)
        self.assertFalse(m.tool_manager.calibration_valid)
        self.assertEqual(m.pins['laser']['value'], 0.)


class IdFilterTests(unittest.TestCase):
    @staticmethod
    def report(machine, value, poll=False):
        machine.objects['analog_input ID'].data['value'] = value
        machine.now += .1
        manager = machine.tool_manager
        if poll:
            manager._poll(machine.now)
        return manager.get_status(machine.now)

    def test_measured_motor_noise_range_preserves_active_job(self):
        m = Machine()
        m.job()
        for value in [80., 79.11, 81.1, 79.2, 78.9, 80.6] * 10:
            status = self.report(m, value, poll=True)
            self.assertEqual(status['detected_tool'], 'BLUE_LASER')
            self.assertTrue(status['power_granted'])
            self.assertIsNone(m.shutdown_message)

    def test_main_acquires_across_measured_range_without_previous_recognition(self):
        for value in (78.9, 79.11, 80., 81.1):
            with self.subTest(value=value):
                m = Machine(id_value=value)
                self.assertEqual(m.tool_manager.detected, 'BLUE_LASER')
                m.job()
                self.report(m, value, poll=True)
                self.assertIsNone(m.shutdown_message)
                self.assertTrue(m.tool_manager.granted)

    def test_median_rejects_two_in_band_outliers_and_reports_raw_value(self):
        m = Machine(id_value=80.)
        for value in (80., 82.9, 82.8, 80., 80.):
            status = self.report(m, value)
            self.assertAlmostEqual(status['id_value'], value)
            self.assertAlmostEqual(status['id_filtered_value'], 80.)
            self.assertEqual(status['id_filter_samples'], 5)
            self.assertEqual(status['id_filter_size'], 5)

    def test_hysteresis_holds_each_tool_at_both_boundaries(self):
        for low, high, tool in MODULE.ID_WINDOWS:
            if tool not in MODULE.TOOLS:
                continue
            for value in (low - .1, high + .1):
                with self.subTest(tool=tool, value=value):
                    m = Machine(id_value=(low + high) / 2)
                    select(m, value)
                    self.assertEqual(m.tool_manager.detected, tool)

    def test_hysteresis_does_not_widen_initial_acquisition(self):
        for low, high, tool in MODULE.ID_WINDOWS:
            if tool not in MODULE.TOOLS:
                continue
            for value in (low - .1, high + .1):
                with self.subTest(tool=tool, value=value):
                    m = Machine(id_value=value)
                    select(m, value)
                    self.assertEqual(m.tool_manager.detected, 'UNKNOWN')
                    self.assertFalse(m.tool_manager.granted)

    def test_zero_hysteresis_preserves_strict_exit_threshold(self):
        m = Machine(tool_options={'id_hysteresis': 0.})
        m.job()
        self.report(m, 76.99, poll=True)
        self.assertIsNotNone(m.shutdown_message)
        self.assertFalse(m.tool_manager.granted)

    def test_noisy_acquisition_needs_full_window_and_three_new_medians(self):
        m = Machine(id_value=0.)
        reports = (76.99, 77.01, 76.97, 77.02, 77.03, 77.04, 76.96)
        for value in reports[:-1]:
            status = self.report(m, value)
            self.assertEqual(status['detected_tool'], 'UNSTABLE')
        status = self.report(m, reports[-1])
        self.assertEqual(status['detected_tool'], 'BLUE_LASER')
        self.assertEqual(status['stable_samples'], 3)
        self.assertAlmostEqual(status['id_filtered_value'], 77.02)

    def test_raw_escape_revokes_active_power_on_first_report(self):
        for value in (76.49, 83.51, 31.21, 68.79, 50., 41.82,
                      59.09, 17.58, 0., 100., float('nan'), float('inf')):
            with self.subTest(value=value):
                m = Machine()
                m.job()
                self.report(m, value, poll=True)
                self.assertIsNotNone(m.shutdown_message)
                self.assertFalse(m.tool_manager.granted)
                self.assertTrue(all(pin['value'] == 0 for pin in m.pins.values()))

    def test_direct_power_guard_observes_escape_before_periodic_poll(self):
        m = Machine()
        m.job()
        m.objects['analog_input ID'].data['value'] = 76.49
        m.now += .1
        with self.assertRaises(MacroError):
            m.power(.1)
        self.assertEqual(m.tool_manager.detected, 'UNSTABLE')

    def test_tool_change_refills_filter_without_synthetic_intermediate_id(self):
        m = Machine()
        for _ in range(6):
            status = self.report(m, 59.09)
            self.assertEqual(status['detected_tool'], 'UNSTABLE')
            self.assertAlmostEqual(status['id_filtered_value'], 59.09)
        status = self.report(m, 59.09)
        self.assertEqual(status['detected_tool'], 'IR_LASER')
        self.assertTrue(status['ir_z_locked'])

    def test_alternating_tools_never_form_a_filtered_tool_or_conflict_id(self):
        m = Machine()
        for value in (59.09, 82.12) * 10:
            status = self.report(m, value)
            self.assertEqual(status['detected_tool'], 'UNSTABLE')
            self.assertAlmostEqual(status['id_filtered_value'], value)
            self.assertEqual(status['id_filter_samples'], 1)

    def test_stale_id_clears_filter_and_requires_full_reacquisition(self):
        m = Machine()
        adc = m.objects['analog_input ID'].mcu_adc
        adc.timestamp = adc.get_last_value()[0]
        m.now += .7
        status = m.tool_manager.get_status(m.now)
        self.assertEqual(status['detected_tool'], 'STALE')
        self.assertIsNone(status['id_filtered_value'])
        self.assertEqual(status['id_filter_samples'], 0)
        adc.timestamp = None
        for _ in range(6):
            status = self.report(m, 82.12)
            self.assertEqual(status['detected_tool'], 'UNSTABLE')
        status = self.report(m, 82.12)
        self.assertEqual(status['detected_tool'], 'BLUE_LASER')

    def test_report_gap_does_not_reuse_old_filter_history(self):
        m = Machine()
        m.now += 1.
        status = m.tool_manager.get_status(m.now)
        self.assertEqual(status['detected_tool'], 'UNSTABLE')
        self.assertEqual(status['id_filter_samples'], 1)
        self.assertEqual(status['stable_samples'], 0)

    def test_stale_recovery_cannot_reuse_previous_hysteresis(self):
        m = Machine()
        adc = m.objects['analog_input ID'].mcu_adc
        adc.timestamp = adc.get_last_value()[0]
        m.now += .7
        self.assertEqual(m.tool_manager.get_status(m.now)['detected_tool'], 'STALE')
        adc.timestamp = None
        select(m, 76.9)
        self.assertEqual(m.tool_manager.detected, 'UNKNOWN')

    def test_timestamp_regression_and_future_report_are_invalid(self):
        for delta in (-.1, 1.):
            with self.subTest(delta=delta):
                m = Machine()
                adc = m.objects['analog_input ID'].mcu_adc
                adc.timestamp = adc.get_last_value()[0] + delta
                status = m.tool_manager.get_status(m.now)
                self.assertEqual(status['detected_tool'], 'STALE')
                self.assertEqual(status['id_filter_samples'], 0)

    def test_filter_rejects_even_window_and_nonfinite_hysteresis(self):
        for options in ({'id_filter_samples': 4},
                        {'id_hysteresis': float('nan')}):
            with self.subTest(options=options), self.assertRaises(MacroError):
                Machine(tool_options=options)


if __name__ == '__main__':
    unittest.main()
