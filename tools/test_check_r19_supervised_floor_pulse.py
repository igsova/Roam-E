import importlib.util
import contextlib
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent))
spec = importlib.util.spec_from_file_location(
    'floor_pulse', Path(__file__).with_name('check_r19_supervised_floor_pulse.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FloorGuardTest(unittest.TestCase):
    def setUp(self):
        self.row = [1000, 0, 0, 0, 0, 0, 0, 0, 0]
        self.guard = module.FeedbackGuard(self.row, 0.)

    def test_ceiling_preserved(self):
        self.assertEqual(module.PWM, 100)
        self.assertLess(module.REQUESTED_SECONDS-module.STOP_MARGIN, 1.)
        self.assertLess(module.REFRESH_SECONDS, .350)

    def test_selected_channel_commands(self):
        self.assertEqual(module.pulse_command('left'), b'PWM,100,0\n')
        self.assertEqual(module.pulse_command('right'), b'PWM,0,100\n')

    def test_invalid_channel_refused(self):
        for channel in ('both', None, 'LEFT'):
            with self.assertRaises(ValueError):
                module.pulse_command(channel)
            with self.assertRaises(ValueError):
                module.FeedbackGuard(self.row, 0., channel)

    def test_unarmed_cli_never_touches_hardware(self):
        with patch.object(sys, 'argv', ['test', '--pulse-channel', 'right']), \
                patch.object(module, 'command') as hardware, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                module.main()
            self.assertEqual(error.exception.code, 2)
            hardware.assert_not_called()

    def test_missing_channel_never_touches_hardware(self):
        with patch.object(sys, 'argv', ['test', '--arm-supervised-floor-one-second']), \
                patch.object(module, 'command') as hardware, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                module.main()
            self.assertEqual(error.exception.code, 2)
            hardware.assert_not_called()

    def test_right_channel_response(self):
        guard = module.FeedbackGuard(self.row, 0., 'right')
        guard.observe([1050, 0, 1, 0, 100, 0, 0, 0, 0], .05)
        guard.check(.14)
        self.assertTrue(guard.responded)

    def test_unselected_left_pwm_stops_right_check(self):
        guard = module.FeedbackGuard(self.row, 0., 'right')
        with self.assertRaisesRegex(RuntimeError, 'Unexpected motor output'):
            guard.observe([1050, 0, 0, 1, 100, 0, 0, 0, 0], .05)

    def test_other_encoder_alone_does_not_satisfy_response(self):
        guard = module.FeedbackGuard(self.row, 0., 'right')
        for t in range(1, 7):
            guard.observe([1000+t*50, t, 0, 0, 100, 0, 0, 0, 0], t*.05)
        with self.assertRaisesRegex(RuntimeError, 'No encoder response'):
            guard.check(.301)

    def test_initial_no_movement_tolerated(self):
        self.guard.observe([1050, 0, 0, 100, 0, 0, 0, 0, 0], .05)
        self.guard.check(.14)

    def test_stale_feedback_stops(self):
        with self.assertRaisesRegex(RuntimeError, 'Stale'):
            self.guard.check(.151)

    def test_no_encoder_response_stops(self):
        for t in range(1, 7):
            self.guard.observe([1000+t*50, 0, 0, 100, 0, 0, 0, 0, 0], t*.05)
        with self.assertRaisesRegex(RuntimeError, 'No encoder response'):
            self.guard.check(.301)

    def test_count_response_allows_continuation(self):
        self.guard.observe([1050, 1, 0, 100, 0, 0, 0, 0, 0], .05)
        self.guard.observe([1100, 2, 0, 100, 0, 0, 0, 0, 0], .1)
        self.guard.check(.14)
        self.assertTrue(self.guard.responded)

    def test_negative_count_is_not_direction_acceptance(self):
        self.guard.observe([1050, -1, 0, 100, 0, 0, 0, 0, 0], .05)
        self.assertTrue(self.guard.responded)

    def test_right_pwm_stops(self):
        with self.assertRaisesRegex(RuntimeError, 'Unexpected motor output'):
            self.guard.observe([1050, 0, 0, 100, 1, 0, 0, 0, 0], .05)

    def test_unexpected_left_pwm_stops(self):
        with self.assertRaises(RuntimeError):
            self.guard.observe([1050, 0, 0, 101, 0, 0, 0, 0, 0], .05)

    def test_velocity_mode_stops(self):
        with self.assertRaises(RuntimeError):
            self.guard.observe([1050, 0, 0, 100, 0, 1, 0, 0, 0], .05)

    def test_controller_reset_stops(self):
        with self.assertRaisesRegex(RuntimeError, 'clock'):
            self.guard.observe([200, 0, 0, 100, 0, 0, 0, 0, 0], .05)

    def test_repeated_clock_stops(self):
        with self.assertRaisesRegex(RuntimeError, 'clock'):
            self.guard.observe([1000, 0, 0, 100, 0, 0, 0, 0, 0], .05)

    def test_uint32_clock_wrap(self):
        guard = module.FeedbackGuard([2**32-25, 0, 0, 0, 0, 0, 0, 0, 0], 0.)
        guard.observe([25, 1, 0, 100, 0, 0, 0, 0, 0], .05)
        self.assertTrue(guard.responded)


class LiftedReverseGuardTest(unittest.TestCase):
    def setUp(self):
        self.row = [1000, 0, 0, 0, 0, 0, 0, 0, 0]

    def refused_cli(self, arguments):
        with patch.object(sys, 'argv', ['test', *arguments]), \
                patch.object(module, 'command') as hardware, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                module.main()
            self.assertEqual(error.exception.code, 2)
            hardware.assert_not_called()

    def test_reverse_commands_select_only_one_channel(self):
        self.assertEqual(module.pulse_command('left', 'reverse'), b'PWM,-100,0\n')
        self.assertEqual(module.pulse_command('right', 'reverse'), b'PWM,0,-100\n')
        self.assertEqual(module.pulse_command('left', 'forward'), b'PWM,100,0\n')

    def test_invalid_direction_refused(self):
        for direction in (None, 'both', 'REVERSE'):
            with self.assertRaises(ValueError):
                module.pulse_command('left', direction)
            with self.assertRaises(ValueError):
                module.FeedbackGuard(self.row, 0., 'left', direction)

    def test_reverse_without_arm_never_touches_hardware(self):
        self.refused_cli(['--pulse-channel', 'left', '--pulse-direction', 'reverse'])

    def test_reverse_floor_arm_never_touches_hardware(self):
        self.refused_cli(['--pulse-channel', 'left', '--pulse-direction', 'reverse',
                          '--arm-supervised-floor-one-second'])

    def test_both_arms_never_touch_hardware(self):
        self.refused_cli(['--pulse-channel', 'left', '--pulse-direction', 'reverse',
                          '--arm-supervised-floor-one-second',
                          '--arm-supervised-lifted-one-second'])

    def test_floor_reverse_arm_rejects_forward_before_hardware(self):
        self.refused_cli(['--pulse-channel', 'right', '--pulse-direction', 'forward',
                          '--arm-supervised-floor-reverse-one-second'])

    def test_floor_reverse_arm_conflicts_with_other_arms(self):
        for other in ('--arm-supervised-floor-one-second',
                      '--arm-supervised-lifted-one-second'):
            with self.subTest(other=other):
                self.refused_cli(['--pulse-channel', 'right', '--pulse-direction', 'reverse',
                                  '--arm-supervised-floor-reverse-one-second', other])

    def test_floor_reverse_metadata_without_serial(self):
        output = io.StringIO()
        with patch.object(sys, 'argv', ['test', '--pulse-channel', 'right',
                                       '--pulse-direction', 'reverse',
                                       '--arm-supervised-floor-reverse-one-second']), \
                patch.object(module, 'command', side_effect=RuntimeError('mock stop')), \
                patch.object(module.signal, 'signal'), \
                contextlib.redirect_stdout(output):
            self.assertEqual(module.main(), 2)
        import json
        result = json.loads(output.getvalue())
        self.assertEqual(result['pwm'], -100)
        self.assertEqual(result['requested_setup'], 'floor')
        self.assertEqual(result['floor_no_progress_seconds'], .25)
        self.assertFalse(result['pulse_sent'])
        self.assertEqual(result['commands'], [])

    def test_reverse_cli_propagates_direction_without_serial(self):
        # Raise at the first read-only hardware check; main must report the
        # signed requested PWM and setup, with no serial or pulse attempted.
        output = io.StringIO()
        with patch.object(sys, 'argv', ['test', '--pulse-channel', 'left',
                                       '--pulse-direction', 'reverse',
                                       '--arm-supervised-lifted-one-second']), \
                patch.object(module, 'command', side_effect=RuntimeError('mock stop')), \
                patch.object(module.signal, 'signal'), \
                contextlib.redirect_stdout(output):
            self.assertEqual(module.main(), 2)
        import json
        result = json.loads(output.getvalue())
        self.assertEqual(result['pwm'], -100)
        self.assertEqual(result['requested_direction'], 'reverse')
        self.assertEqual(result['requested_setup'], 'lifted')
        self.assertFalse(result['pulse_sent'])
        self.assertEqual(result['commands'], [])

    def test_both_reverse_channels_allow_negative_selected_counts(self):
        for channel in ('left', 'right'):
            with self.subTest(channel=channel):
                guard = module.FeedbackGuard(self.row, 0., channel, 'reverse')
                row = [1050, 0, 0, 0, 0, 0, 0, 0, 0]
                row[guard.encoder_index] = -1
                row[guard.pwm_index] = -100
                guard.observe(row, .05)
                guard.check(.14)
                self.assertTrue(guard.responded)

    def test_positive_reverse_count_aborts(self):
        guard = module.FeedbackGuard(self.row, 0., 'left', 'reverse')
        with self.assertRaisesRegex(RuntimeError, 'increased during reverse'):
            guard.observe([1050, 1, 0, -100, 0, 0, 0, 0, 0], .05)

    def test_unselected_encoder_reverse_movement_aborts(self):
        guard = module.FeedbackGuard(self.row, 0., 'right', 'reverse')
        with self.assertRaisesRegex(RuntimeError, 'Unselected encoder'):
            guard.observe([1050, -1, -1, 0, -100, 0, 0, 0, 0], .05)

    def test_forward_pwm_in_reverse_aborts(self):
        guard = module.FeedbackGuard(self.row, 0., 'left', 'reverse')
        with self.assertRaisesRegex(RuntimeError, 'Unexpected motor output'):
            guard.observe([1050, 0, 0, 100, 0, 0, 0, 0, 0], .05)

    def test_reverse_zero_pwm_before_response_allowed(self):
        guard = module.FeedbackGuard(self.row, 0., 'right', 'reverse')
        guard.observe([1050, 0, 0, 0, 0, 0, 0, 0, 0], .05)
        guard.check(.14)
        self.assertFalse(guard.responded)

    def test_reverse_stale_telemetry_aborts(self):
        guard = module.FeedbackGuard(self.row, 0., 'left', 'reverse')
        with self.assertRaisesRegex(RuntimeError, 'Stale'):
            guard.check(.151)

    def test_reverse_no_response_aborts(self):
        guard = module.FeedbackGuard(self.row, 0., 'left', 'reverse')
        for t in range(1, 7):
            guard.observe([1000+t*50, 0, 0, -100, 0, 0, 0, 0, 0], t*.05)
        with self.assertRaisesRegex(RuntimeError, 'No encoder response'):
            guard.check(.301)

    def test_reverse_int32_count_wrap(self):
        baseline = [1000, -(2**31), 0, 0, 0, 0, 0, 0, 0]
        guard = module.FeedbackGuard(baseline, 0., 'left', 'reverse')
        guard.observe([1050, 2**31-1, 0, -100, 0, 0, 0, 0, 0], .05)
        self.assertTrue(guard.responded)


class LiftedReverseMainTest(unittest.TestCase):
    def run_fake(self, channel='left', reversed_feedback=False, setup='lifted',
                 passive_other=False, plateau=False):
        class Clock:
            now = 0.
            def monotonic(self):
                return self.now
            def sleep(self, duration):
                self.now += duration
        clock = Clock()

        class Connection:
            def __init__(self):
                self.pending = []
                self.pwm = [0, 0]
                self.counts = [0, 0]
                self.milliseconds = 0
                self.closed = False
                self.writes = []
            def reset_input_buffer(self):
                self.pending.clear()
            def write(self, data):
                self.writes.append(data)
                if data == b'STOP\n':
                    self.pwm = [0, 0]
                    self.pending.append(b'OK,STOP\n')
                elif data == b'INFO\n':
                    self.pending.append((module.EXPECTED_INFO+'\n').encode())
                elif data.startswith(b'PWM,'):
                    self.pwm = [int(x) for x in data.decode().strip().split(',')[1:]]
                else:
                    raise AssertionError('Unexpected command')
                return len(data)
            def readline(self, limit):
                clock.now += .05
                self.milliseconds += 50
                if self.pending:
                    return self.pending.pop(0)
                for i in (0, 1):
                    if self.pwm[i]:
                        if not plateau or self.counts[i] == 0:
                            self.counts[i] += 5 if reversed_feedback else -5
                        if passive_other:
                            self.counts[1-i] += 1
                fields = [self.milliseconds, *self.counts, *self.pwm, 0, 0, 0, 0]
                return ('T,'+','.join(str(x) for x in fields)+'\n').encode()
            def close(self):
                self.closed = True
        connection = Connection()
        power_timeouts = []

        def hardware_command(args, timeout=2):
            if args[0] == 'udevadm':
                return 'ID_VENDOR_ID=1a86\nID_MODEL_ID=7523'
            if args == ['vcgencmd', 'get_throttled']:
                power_timeouts.append(timeout)
                return 'throttled=0x0'
            if args == ['vcgencmd', 'pmic_read_adc', 'EXT5V_V']:
                return 'EXT5V_V volt(24)=5.03000000V'
            raise AssertionError('Unexpected hardware command')

        output = io.StringIO()
        arm = ('--arm-supervised-lifted-one-second' if setup == 'lifted' else
               '--arm-supervised-floor-reverse-one-second')
        with patch.object(sys, 'argv', ['test', '--pulse-channel', channel,
                                       '--pulse-direction', 'reverse',
                                       arm]), \
                patch.object(module, 'command', side_effect=hardware_command), \
                patch.object(module, 'time', clock), \
                patch.object(module.signal, 'signal'), \
                patch.object(module.subprocess, 'run',
                             return_value=SimpleNamespace(returncode=1, stdout='', stderr='')), \
                patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=lambda *a, **k: connection)}), \
                contextlib.redirect_stdout(output):
            exit_code = module.main()
        import json
        return exit_code, json.loads(output.getvalue()), connection, power_timeouts

    def test_each_channel_main_sends_only_selected_reverse_then_stop(self):
        for channel in ('left', 'right'):
            with self.subTest(channel=channel):
                code, result, connection, power_timeouts = self.run_fake(channel)
                self.assertEqual(code, 0)
                self.assertTrue(result['pulse_sent'])
                self.assertTrue(result['stop_confirmed'])
                self.assertTrue(result['serial_closed'])
                self.assertTrue(connection.closed)
                expected = module.pulse_command(channel, 'reverse')
                pwm_writes = [x for x in connection.writes if x.startswith(b'PWM,')]
                self.assertTrue(pwm_writes)
                self.assertTrue(all(x == expected for x in pwm_writes))
                self.assertEqual(connection.writes[-1], b'STOP\n')
                self.assertEqual(connection.pwm, [0, 0])
                selected = 0 if channel == 'left' else 1
                self.assertLess(result['encoder_delta'][selected], 0)
                self.assertEqual(result['encoder_delta'][1-selected], 0)
                self.assertEqual(power_timeouts[:2], [.2, .2])
                self.assertEqual(power_timeouts[-1], .2)
                self.assertTrue(power_timeouts[2:-1])
                self.assertTrue(all(x == .05 for x in power_timeouts[2:-1]))

    def test_main_opposite_feedback_stops_early_and_closes(self):
        code, result, connection, _ = self.run_fake(reversed_feedback=True)
        self.assertEqual(code, 2)
        self.assertIn('increased during reverse', result['reason'])
        self.assertLess(result['pulse_to_stop_elapsed_s'], module.NO_RESPONSE_SECONDS)
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(result['serial_closed'])
        self.assertTrue(connection.closed)
        self.assertEqual(connection.writes[-1], b'STOP\n')
        self.assertEqual(connection.pwm, [0, 0])

    def test_floor_main_preserves_selected_pwm_and_stop_with_passive_other(self):
        code, result, connection, power_timeouts = self.run_fake(
            'right', setup='floor', passive_other=True)
        self.assertEqual(code, 0)
        self.assertEqual(result['requested_setup'], 'floor')
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(result['serial_closed'])
        self.assertTrue(connection.closed)
        pwm_writes = [x for x in connection.writes if x.startswith(b'PWM,')]
        self.assertTrue(pwm_writes)
        self.assertTrue(all(x == b'PWM,0,-100\n' for x in pwm_writes))
        self.assertEqual(connection.writes[-1], b'STOP\n')
        self.assertEqual(connection.pwm, [0, 0])
        self.assertGreater(result['encoder_delta'][0], 0)
        self.assertLess(result['encoder_delta'][1], 0)
        self.assertLess(result['pulse_to_stop_elapsed_s'], 1.)
        self.assertEqual(power_timeouts[:2], [.2, .2])
        self.assertEqual(power_timeouts[-1], .2)
        self.assertTrue(all(x == .05 for x in power_timeouts[2:-1]))

    def test_floor_main_plateau_stops_early_then_confirms_stop(self):
        code, result, connection, _ = self.run_fake('right', setup='floor', plateau=True)
        self.assertEqual(code, 2)
        self.assertIn('Encoder progress stopped', result['reason'])
        self.assertLess(result['pulse_to_stop_elapsed_s'], .5)
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(result['serial_closed'])
        self.assertEqual(connection.pwm, [0, 0])
        self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_floor_main_opposite_feedback_stops_early(self):
        code, result, connection, _ = self.run_fake(
            'right', reversed_feedback=True, setup='floor')
        self.assertEqual(code, 2)
        self.assertIn('increased during reverse', result['reason'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(connection.closed)
        self.assertEqual(connection.pwm, [0, 0])


class FloorReverseProgressTest(unittest.TestCase):
    def setUp(self):
        self.row = [1000, 0, 0, 0, 0, 0, 0, 0, 0]

    def test_invalid_setup_rejected(self):
        with self.assertRaises(ValueError):
            module.FeedbackGuard(self.row, 0., 'right', 'reverse', 'unknown')

    def test_floor_passive_encoder_allowed_but_unselected_pwm_rejected(self):
        guard = module.FeedbackGuard(self.row, 0., 'right', 'reverse', 'floor')
        guard.observe([1050, 1, -2, 0, -100, 0, 0, 0, 0], .05)
        with self.assertRaisesRegex(RuntimeError, 'Unexpected motor output'):
            guard.observe([1100, 2, -4, 1, -100, 0, 0, 0, 0], .1)

    def test_floor_plateau_aborts_even_with_fresh_telemetry(self):
        guard = module.FeedbackGuard(self.row, 0., 'right', 'reverse', 'floor')
        guard.observe([1050, 0, -1, 0, -100, 0, 0, 0, 0], .05)
        for step in range(2, 7):
            guard.observe([1000+step*50, 0, -1, 0, -100, 0, 0, 0, 0], step*.05)
        with self.assertRaisesRegex(RuntimeError, 'Encoder progress stopped'):
            guard.check(.301)

    def test_floor_progress_refreshes_deadline(self):
        guard = module.FeedbackGuard(self.row, 0., 'right', 'reverse', 'floor')
        for step in range(1, 17):
            guard.observe([1000+step*50, 0, -step, 0, -100, 0, 0, 0, 0], step*.05)
            guard.check(step*.05)

    def test_floor_no_response_still_aborts(self):
        guard = module.FeedbackGuard(self.row, 0., 'right', 'reverse', 'floor')
        for step in range(1, 7):
            guard.observe([1000+step*50, 0, 0, 0, -100, 0, 0, 0, 0], step*.05)
        with self.assertRaisesRegex(RuntimeError, 'No encoder response'):
            guard.check(.301)


if __name__ == '__main__':
    unittest.main()
