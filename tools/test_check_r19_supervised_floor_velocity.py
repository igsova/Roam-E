import contextlib
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent))
import check_r19_supervised_floor_velocity as module


class GuardTest(unittest.TestCase):
    def setUp(self):
        self.baseline = [1000, 0, 0, 0, 0, 0, 0, 0, 0]
        self.guard = module.FeedbackGuard(self.baseline, 0., 'right')

    def test_fixed_commands_and_bounds(self):
        self.assertEqual(module.velocity_command('right'), b'VEL,0,200\n')
        self.assertEqual(module.velocity_command('left'), b'VEL,200,0\n')
        self.assertEqual(module.PWM_CEILING, 100)
        self.assertLess(module.REQUESTED_SECONDS-module.STOP_MARGIN, 1.)
        self.assertEqual(module.EXTENDED_REQUESTED_SECONDS, 1.5)
        self.assertLess(module.REFRESH_SECONDS, .350)
        with self.assertRaises(ValueError):
            module.velocity_command('both')

    def test_unarmed_or_missing_channel_never_touches_hardware(self):
        for args in ([], ['--velocity-channel', 'right'],
                     ['--arm-supervised-floor-velocity-one-second'],
                     ['--arm-supervised-floor-velocity-one-point-five-seconds']):
            with self.subTest(args=args), patch.object(sys, 'argv', ['test', *args]), \
                    patch.object(module, 'command') as hardware, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    module.main()
                self.assertEqual(error.exception.code, 2)
                hardware.assert_not_called()

    def test_conflicting_duration_arms_never_touch_hardware(self):
        with patch.object(sys, 'argv', ['test', '--velocity-channel', 'left',
                                       '--arm-supervised-floor-velocity-one-second',
                                       '--arm-supervised-floor-velocity-one-point-five-seconds']), \
                patch.object(module, 'command') as hardware, \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                module.main()
            self.assertEqual(error.exception.code, 2)
            hardware.assert_not_called()

    def test_candidate_rejects_other_channel_or_duration_without_hardware(self):
        for channel, arm in (
                ('right', '--arm-supervised-floor-velocity-one-second'),
                ('left', '--arm-supervised-floor-velocity-one-point-five-seconds')):
            with self.subTest(channel=channel, arm=arm), \
                    patch.object(sys, 'argv', ['test', '--velocity-channel', channel,
                                              arm, '--candidate-left-forward-ff85']), \
                    patch.object(module, 'command') as hardware, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    module.main()
                self.assertEqual(error.exception.code, 2)
                hardware.assert_not_called()

    def test_kp_candidate_invalid_scope_or_conflicting_flags_never_touch_hardware(self):
        for channel, arm, flags in (
                ('right', '--arm-supervised-floor-velocity-one-second',
                 ['--candidate-left-forward-ff85-kp160']),
                ('left', '--arm-supervised-floor-velocity-one-point-five-seconds',
                 ['--candidate-left-forward-ff85-kp160']),
                ('left', '--arm-supervised-floor-velocity-one-second',
                 ['--candidate-left-forward-ff85-kp160', '--candidate-left-forward-ff85'])):
            with self.subTest(channel=channel, arm=arm, flags=flags), \
                    patch.object(sys, 'argv', ['test', '--velocity-channel', channel, arm, *flags]), \
                    patch.object(module, 'command') as hardware, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    module.main()
                self.assertEqual(error.exception.code, 2)
                hardware.assert_not_called()

    def test_passive_other_counts_allowed(self):
        self.guard.observe([1050, -1, 10, 0, 72, 0, 200, -20, 200], .05)
        self.guard.acknowledged = True
        self.guard.check(.14)

    def test_three_second_arm_rejects_wrong_scope_and_conflicts_before_hardware(self):
        three = '--arm-supervised-floor-velocity-three-seconds'
        for channel, flags in (
                ('left', []),
                ('left', ['--candidate-left-forward-ff85']),
                ('right', ['--candidate-left-forward-ff85-kp160']),
                ('left', ['--candidate-left-forward-ff85-kp160',
                          '--arm-supervised-floor-velocity-one-second']),
                ('left', ['--candidate-left-forward-ff85-kp160',
                          '--arm-supervised-floor-velocity-one-point-five-seconds'])):
            with self.subTest(channel=channel, flags=flags), \
                    patch.object(sys, 'argv', ['test', '--velocity-channel', channel,
                                              three, *flags]), \
                    patch.object(module, 'command') as hardware, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    module.main()
                self.assertEqual(error.exception.code, 2)
                hardware.assert_not_called()

    def test_unselected_pwm_wrong_target_or_reverse_rejected(self):
        rows = ([1050, 0, 10, 1, 70, 0, 200, 0, 200],
                [1050, 0, 10, 0, -70, 0, 200, 0, 200],
                [1050, 0, 10, 0, 70, 200, 200, 0, 200],
                [1050, 0, -1, 0, 70, 0, 200, 0, 0])
        for row in rows:
            with self.subTest(row=row), self.assertRaises(RuntimeError):
                self.guard.observe(row, .05)

    def test_startup_idle_grace_does_not_allow_late_idle(self):
        self.guard.observe([1050, 0, 0, 0, 0, 0, 0, 0, 0], .05)
        with self.assertRaisesRegex(RuntimeError, 'target'):
            self.guard.observe([1150, 0, 0, 0, 0, 0, 0, 0, 0], .15)

    def test_clock_reset_and_duplicate_rejected(self):
        for ms in (200, 1000, 1300):
            with self.subTest(ms=ms), self.assertRaisesRegex(RuntimeError, 'clock'):
                self.guard.observe([ms, 0, 10, 0, 70, 0, 200, 0, 200], .05)

    def test_clock_and_count_wrap(self):
        guard = module.FeedbackGuard([2**32-25, 0, 2**31-1, 0, 0, 0, 0, 0, 0], 0., 'right')
        guard.observe([25, 0, -2**31, 0, 70, 0, 200, 0, 20], .05)
        self.assertTrue(guard.responded)

    def test_stale_or_missing_ack_stops(self):
        with self.assertRaisesRegex(RuntimeError, 'Stale'):
            self.guard.check(.151)
        self.guard.observe([1100, 0, 10, 0, 70, 0, 200, 0, 200], .1)
        with self.assertRaisesRegex(RuntimeError, 'acknowledgement'):
            self.guard.check(.151)

    def test_no_response_and_later_stall_stop(self):
        self.guard.acknowledged = True
        for step in range(1, 7):
            self.guard.observe([1000+50*step, 0, 0, 0, 70, 0, 200, 0, 0], .05*step)
        with self.assertRaisesRegex(RuntimeError, 'No encoder response'):
            self.guard.check(.301)
        guard = module.FeedbackGuard(self.baseline, 0., 'right')
        guard.acknowledged = True
        for step in range(1, 7):
            guard.observe([1000+50*step, 0, 10, 0, 70, 0, 200, 0, 200], .05*step)
        with self.assertRaisesRegex(RuntimeError, 'progress stopped'):
            guard.check(.301)

    def test_raw_or_filtered_overspeed_rejected(self):
        for count, measured in ((51, 200), (10, 1001)):
            with self.subTest(count=count), self.assertRaisesRegex(RuntimeError, 'speed'):
                self.guard.observe([1050, 0, count, 0, 70, 0, 200, 0, measured], .05)


class MainTest(unittest.TestCase):
    def run_fake(self, channel='right', fault=None, extended=False, candidate=False,
                 pid_candidate=False, three_seconds=False):
        class Clock:
            now = 0.
            def monotonic(self):
                return self.now
            def sleep(self, seconds):
                self.now += seconds
        clock = Clock()
        selected = 0 if channel == 'left' else 1

        class Connection:
            pending = None
            def __init__(self):
                self.pending, self.writes = [], []
                self.targets, self.pwm, self.counts = [0, 0], [0, 0], [0, 0]
                self.last_row = clock.now
                self.closed, self.active_once = False, False
                self.active_started = None
                self.ff = list(module.BASELINE_FEEDFORWARD)
                self.pid = list(module.BASELINE_PID[:3])
                self.cleanup_stop_failed = False
            def reset_input_buffer(self):
                self.pending.clear()
            def write(self, data):
                self.writes.append(data)
                line = data.decode('ascii').strip()
                if line == 'STOP':
                    if (fault == 'cleanup_stop_write_error' and self.active_once and
                            not any(self.targets) and not self.cleanup_stop_failed):
                        self.cleanup_stop_failed = True
                        raise OSError('Mock cleanup STOP write failed')
                    self.targets, self.pwm = [0, 0], [0, 0]
                    self.pending.append(b'OK,STOP\n')
                elif line == 'INFO':
                    self.pending.append((module.EXPECTED_INFO+'\n').encode())
                elif line.startswith('VEL,'):
                    if self.active_started is None:
                        self.active_started = clock.now
                    self.targets = [int(x) for x in line.split(',')[1:]]
                    self.pwm[selected], self.active_once = 70, True
                    if fault != 'missing_ack':
                        ack = 'OK,VEL,1,1' if fault == 'wrong_ack' else 'OK,'+line
                        self.pending.append((ack+'\n').encode())
                    if fault == 'write_error':
                        return len(data)-1
                elif line.startswith('PID,'):
                    if any(self.targets) or any(self.pwm):
                        raise AssertionError('PID changed while controller was active')
                    values = [int(x) for x in line.split(',')[1:]]
                    self.pid = values[:3]
                    self.ff = [values[3]]*4  # Actual protocol's FF side effect.
                    restoring = tuple(values) == module.BASELINE_PID
                    if restoring and fault == 'pid_restore_error':
                        self.pending.append(b'ERR,RANGE\n')
                    elif (restoring and fault == 'missing_pid_restore_ack') or (
                            not restoring and fault == 'missing_pid_candidate_ack'):
                        pass
                    elif not restoring and fault == 'wrong_pid_candidate_ack':
                        self.pending.append(b'OK,PID,159,200,0,75\n')
                    else:
                        self.pending.append(('OK,'+line+'\n').encode())
                    if (not restoring and fault == 'partial_pid_candidate_write') or (
                            restoring and fault == 'partial_pid_restore_write'):
                        return len(data)-1
                elif line.startswith('FF,'):
                    if any(self.targets) or any(self.pwm):
                        raise AssertionError('FF changed while controller was active')
                    self.ff = [int(x) for x in line.split(',')[1:]]
                    restoring = self.ff == list(module.BASELINE_FEEDFORWARD)
                    if restoring and fault == 'restore_error':
                        self.pending.append(b'ERR,RANGE\n')
                    elif (restoring and fault == 'missing_restore_ack') or (
                            not restoring and fault == 'missing_candidate_ack'):
                        pass
                    elif not restoring and fault == 'wrong_candidate_ack':
                        self.pending.append(b'OK,FF,84,60,75,60\n')
                    else:
                        self.pending.append(('OK,'+line+'\n').encode())
                    if not restoring and fault == 'partial_candidate_write':
                        return len(data)-1
                else:
                    raise AssertionError('Unexpected command: '+line)
                return len(data)
            def readline(self, size):
                clock.now += .01
                if self.pending:
                    return self.pending.pop(0)
                active = any(self.targets)
                late = active and clock.now-self.active_started >= 1.25
                if active and fault == 'interrupt':
                    raise RuntimeError('Interrupted; STOP required')
                if active and fault == 'partial_line':
                    return b'T,broken'
                if active and (fault == 'stale' or (fault == 'late_stale' and late)):
                    return b''
                if clock.now-self.last_row < .05:
                    return b''
                self.last_row = clock.now
                if active:
                    stalled = ((fault == 'stall' and self.counts[selected]) or
                               (fault == 'late_stall' and late))
                    if fault != 'no_response' and not stalled:
                        reverse = fault == 'reverse' or (fault == 'late_reverse' and late)
                        self.counts[selected] += -10 if reverse else 10
                    self.counts[1-selected] -= 1  # Passive floor wheel, not powered.
                targets = self.targets.copy()
                pwm = self.pwm.copy()
                if active and (fault == 'wrong_target' or (fault == 'late_wrong_target' and late)):
                    targets[1-selected] = 200
                if active and (fault == 'other_pwm' or (fault == 'late_other_pwm' and late)):
                    pwm[1-selected] = 70
                if (fault == 'configuration_motion' and not self.active_once and
                        self.ff == list(module.CANDIDATE_FEEDFORWARD)):
                    pwm[selected] = 100
                if (fault == 'pid_configuration_motion' and not self.active_once and
                        self.pid == list(module.CANDIDATE_PID[:3])):
                    pwm[selected] = 100
                ms = int(clock.now*1000)
                measured = targets.copy()
                if active and fault == 'late_overspeed' and late:
                    measured[selected] = 1001
                row = [ms, *self.counts, *pwm, *targets, *measured]
                return ('T,'+','.join(str(x) for x in row)+'\n').encode()
            def close(self):
                self.closed = True
        connection = Connection()
        power_timeouts = []

        def hardware(args, timeout=2):
            if args[0] == 'udevadm':
                return 'ID_VENDOR_ID=1a86\nID_MODEL_ID=7523'
            if args == ['vcgencmd', 'get_throttled']:
                power_timeouts.append(timeout)
                if connection.active_once and (fault == 'power' or (
                        fault == 'late_power' and clock.now-connection.active_started >= 1.25)):
                    return 'throttled=0x50005'
                return 'throttled=0x0'
            if args == ['vcgencmd', 'pmic_read_adc', 'EXT5V_V']:
                return 'EXT5V_V volt(24)=5.03V'
            raise AssertionError('Unexpected hardware access')

        output = io.StringIO()
        arm = ('--arm-supervised-floor-velocity-three-seconds' if three_seconds else
               '--arm-supervised-floor-velocity-one-point-five-seconds' if extended
               else '--arm-supervised-floor-velocity-one-second')
        arguments = ['test', '--velocity-channel', channel, arm]
        if candidate:
            arguments.append('--candidate-left-forward-ff85')
        if pid_candidate:
            arguments.append('--candidate-left-forward-ff85-kp160')
        with patch.object(sys, 'argv', arguments), \
                patch.object(module, 'time', clock), patch.object(module.signal, 'signal'), \
                patch.object(module, 'command', side_effect=hardware), \
                patch.object(module.subprocess, 'run', return_value=SimpleNamespace(
                    returncode=1, stdout='', stderr='')), \
                patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=lambda *a, **k: connection)}), \
                contextlib.redirect_stdout(output):
            code = module.main()
        return code, json.loads(output.getvalue()), connection, power_timeouts

    def test_each_channel_has_one_bounded_episode_and_stop(self):
        for channel in ('left', 'right'):
            with self.subTest(channel=channel):
                code, result, connection, timeouts = self.run_fake(channel)
                self.assertEqual(code, 0, result['reason'])
                self.assertTrue(result['velocity_sent'])
                self.assertTrue(result['stop_confirmed'])
                self.assertTrue(result['serial_closed'])
                self.assertTrue(connection.closed)
                self.assertEqual(connection.targets, [0, 0])
                self.assertEqual(connection.writes[-1], b'STOP\n')
                velocities = [c for c in connection.writes if c.startswith(b'VEL,')]
                self.assertGreater(len(velocities), 1)
                self.assertTrue(all(c == module.velocity_command(channel) for c in velocities))
                self.assertFalse(any(c.startswith((b'PWM,', b'PID,', b'FF,', b'ZERO')) for c in connection.writes))
                self.assertLess(result['velocity_to_stop_elapsed_s'], 1.)
                self.assertEqual(timeouts[:2], [.2, .2])
                self.assertEqual(timeouts[-1], .2)
                self.assertTrue(all(t == .05 for t in timeouts[2:-1]))

    def test_extended_arm_only_changes_nominal_duration(self):
        code, result, connection, _ = self.run_fake('left', extended=True)
        self.assertEqual(code, 0, result['reason'])
        self.assertEqual(result['requested_seconds'], 1.5)
        self.assertGreater(result['velocity_to_stop_elapsed_s'], 1.)
        self.assertLess(result['velocity_to_stop_elapsed_s'], 1.5)
        self.assertEqual(result['target_ticks_per_second'], 200)
        self.assertEqual(result['pwm_ceiling'], 100)
        self.assertEqual(result['no_progress_seconds'], .25)
        self.assertEqual(result['refresh_seconds'], .08)
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(connection.closed)
        self.assertTrue(all(c in (b'STOP\n', b'INFO\n', b'VEL,200,0\n')
                            for c in connection.writes))

    def test_extended_faults_still_stop_early_and_close(self):
        for fault in ('reverse', 'stall', 'no_response', 'missing_ack', 'wrong_ack',
                      'write_error', 'interrupt', 'partial_line', 'stale',
                      'wrong_target', 'other_pwm', 'power'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, extended=True)
                self.assertEqual(code, 2)
                self.assertEqual(result['requested_seconds'], 1.5)
                self.assertLess(result['velocity_to_stop_elapsed_s'], .6)
                self.assertTrue(result['stop_confirmed'])
                self.assertTrue(result['serial_closed'])
                self.assertTrue(connection.closed)
                self.assertEqual(connection.writes[-1], b'STOP\n')
                self.assertEqual(connection.targets, [0, 0])

    def test_faults_stop_early_and_close_without_retry(self):
        for fault in ('reverse', 'stall', 'no_response', 'missing_ack', 'wrong_ack',
                      'write_error', 'interrupt', 'partial_line', 'stale',
                      'wrong_target', 'other_pwm', 'power'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake(fault=fault)
                self.assertEqual(code, 2)
                self.assertIsNotNone(result['reason'])
                self.assertTrue(result['serial_closed'])
                self.assertTrue(connection.closed)
                self.assertEqual(connection.targets, [0, 0])
                self.assertEqual(connection.writes[-1], b'STOP\n')
                self.assertLess(result['velocity_to_stop_elapsed_s'], .6)
                self.assertTrue(result['stop_confirmed'])

    def test_candidate_applies_while_stopped_and_restores_after_one_episode(self):
        code, result, connection, _ = self.run_fake('left', candidate=True)
        self.assertEqual(code, 0, result['reason'])
        self.assertTrue(result['feedforward_applied'])
        self.assertTrue(result['feedforward_restored'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(result['serial_closed'])
        self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
        self.assertEqual(connection.targets, [0, 0])
        self.assertLess(result['velocity_to_stop_elapsed_s'], 1.)
        self.assertEqual(result['pwm_ceiling'], 100)
        self.assertEqual(result['no_progress_seconds'], .25)
        writes = connection.writes
        self.assertEqual([x for x in writes if x.startswith(b'FF,')],
                         [b'FF,85,60,75,60\n', b'FF,75,60,75,60\n'])
        self.assertLess(writes.index(b'FF,85,60,75,60\n'), writes.index(b'VEL,200,0\n'))
        self.assertTrue(all(x in (b'STOP\n', b'INFO\n', b'FF,85,60,75,60\n',
                                 b'FF,75,60,75,60\n', b'VEL,200,0\n') for x in writes))

    def test_candidate_active_faults_still_stop_and_restore_without_retry(self):
        for fault in ('reverse', 'stall', 'no_response', 'missing_ack', 'wrong_ack',
                      'write_error', 'interrupt', 'partial_line', 'stale',
                      'wrong_target', 'other_pwm', 'power'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, candidate=True)
                self.assertEqual(code, 2)
                self.assertLess(result['velocity_to_stop_elapsed_s'], .6)
                self.assertTrue(result['stop_confirmed'])
                self.assertTrue(result['feedforward_restored'])
                self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
                self.assertTrue(connection.closed)
                self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_candidate_config_faults_send_no_velocity_and_restore(self):
        for fault in ('missing_candidate_ack', 'wrong_candidate_ack', 'partial_candidate_write'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, candidate=True)
                self.assertEqual(code, 2)
                self.assertFalse(result['velocity_sent'])
                self.assertFalse(result['feedforward_applied'])
                self.assertTrue(result['feedforward_restored'])
                self.assertFalse(any(x.startswith(b'VEL,') for x in connection.writes))
                self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
                self.assertTrue(connection.closed)

    def test_candidate_restore_faults_are_not_reported_as_success(self):
        for fault in ('missing_restore_ack', 'restore_error'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, candidate=True)
                self.assertEqual(code, 2)
                self.assertTrue(result['stop_confirmed'])
                self.assertFalse(result['feedforward_restored'])
                self.assertIn('feedforward_restore_error', result)
                self.assertIn('Cleanup failed', result['reason'])
                self.assertTrue(connection.closed)
                self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_candidate_rejects_nonzero_output_during_configuration(self):
        code, result, connection, _ = self.run_fake('left', 'configuration_motion', candidate=True)
        self.assertEqual(code, 2)
        self.assertFalse(result['velocity_sent'])
        self.assertIn('not stopped', result['reason'])
        self.assertTrue(connection.closed)
        self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_candidate_cleanup_stop_failure_prevents_unconfirmed_restore(self):
        code, result, connection, _ = self.run_fake('left', 'cleanup_stop_write_error', candidate=True)
        self.assertEqual(code, 2)
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['feedforward_restored'])
        self.assertIn('feedforward_restore_error', result)
        self.assertFalse(any(x == b'FF,75,60,75,60\n' for x in connection.writes))
        self.assertTrue(connection.closed)
        self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_kp_candidate_applies_pid_then_ff_and_restores_both(self):
        code, result, connection, timeouts = self.run_fake('left', pid_candidate=True)
        self.assertEqual(code, 0, result['reason'])
        self.assertTrue(result['pid_applied'])
        self.assertTrue(result['feedforward_applied'])
        self.assertTrue(result['pid_restored'])
        self.assertTrue(result['feedforward_restored'])
        self.assertTrue(result['profile_restored'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(connection.closed)
        self.assertEqual(connection.pid, [80, 200, 0])
        self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
        self.assertEqual(connection.targets, [0, 0])
        self.assertEqual(result['target_ticks_per_second'], 200)
        self.assertEqual(result['pwm_ceiling'], 100)
        self.assertEqual(result['no_progress_seconds'], .25)
        self.assertLess(result['velocity_to_stop_elapsed_s'], 1.)
        writes = connection.writes
        configs = [x for x in writes if x.startswith((b'PID,', b'FF,'))]
        self.assertEqual(configs, [b'PID,160,200,0,75\n', b'FF,85,60,75,60\n',
                                  b'PID,80,200,0,75\n', b'FF,75,60,75,60\n'])
        self.assertLess(writes.index(configs[1]), writes.index(b'VEL,200,0\n'))
        self.assertGreater(writes.index(configs[2]), writes.index(b'VEL,200,0\n'))
        self.assertEqual(writes[-1], b'STOP\n')
        self.assertTrue(all(x in (b'STOP\n', b'INFO\n', b'VEL,200,0\n', *configs)
                            for x in writes))
        self.assertEqual([x['acknowledgement'] for x in result['configuration_acknowledgements']],
                         [x.decode().strip().replace('PID,', 'OK,PID,', 1)
                          if x.startswith(b'PID,') else 'OK,'+x.decode().strip()
                          for x in configs])
        self.assertTrue(all(x['expected'] for x in result['configuration_acknowledgements']))
        self.assertEqual(timeouts[:2], [.2, .2])
        self.assertEqual(timeouts[-1], .2)
        self.assertTrue(all(t == .05 for t in timeouts[2:-1]))

    def test_kp_candidate_active_faults_still_stop_and_restore_both(self):
        for fault in ('reverse', 'stall', 'no_response', 'missing_ack', 'wrong_ack',
                      'write_error', 'interrupt', 'partial_line', 'stale',
                      'wrong_target', 'other_pwm', 'power'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True)
                self.assertEqual(code, 2)
                self.assertTrue(result['stop_confirmed'])
                self.assertTrue(result['profile_restored'])
                self.assertTrue(result['pid_restored'])
                self.assertTrue(result['feedforward_restored'])
                self.assertEqual(connection.pid, [80, 200, 0])
                self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
                self.assertEqual(connection.targets, [0, 0])
                self.assertLess(result['velocity_to_stop_elapsed_s'], .6)
                self.assertTrue(connection.closed)

    def test_kp_candidate_pid_configuration_faults_prevent_velocity_restore_both(self):
        for fault in ('missing_pid_candidate_ack', 'wrong_pid_candidate_ack',
                      'partial_pid_candidate_write'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True)
                self.assertEqual(code, 2)
                self.assertFalse(result['pid_applied'])
                self.assertFalse(result['velocity_sent'])
                self.assertFalse(any(x.startswith(b'VEL,') for x in connection.writes))
                self.assertTrue(result['profile_restored'])
                self.assertEqual(connection.pid, [80, 200, 0])
                self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
                self.assertTrue(connection.closed)

    def test_kp_candidate_ff_configuration_faults_prevent_velocity_restore_both(self):
        for fault in ('missing_candidate_ack', 'wrong_candidate_ack', 'partial_candidate_write'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True)
                self.assertEqual(code, 2)
                self.assertTrue(result['pid_applied'])
                self.assertFalse(result['feedforward_applied'])
                self.assertFalse(result['velocity_sent'])
                self.assertTrue(result['profile_restored'])
                self.assertEqual(connection.pid, [80, 200, 0])
                self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
                self.assertTrue(connection.closed)

    def test_kp_candidate_pid_restore_failure_cannot_claim_full_restoration(self):
        for fault in ('missing_pid_restore_ack', 'pid_restore_error', 'partial_pid_restore_write'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True)
                self.assertEqual(code, 2)
                self.assertTrue(result['stop_confirmed'])
                self.assertFalse(result['pid_restored'])
                self.assertFalse(result['feedforward_restored'])
                self.assertFalse(result['profile_restored'])
                self.assertIn('pid_restore_error', result)
                self.assertIn('profile_restore_error', result)
                self.assertFalse(any(x == b'FF,75,60,75,60\n' for x in connection.writes))
                self.assertTrue(connection.closed)
                self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_kp_candidate_ff_restore_failure_cannot_claim_full_restoration(self):
        for fault in ('missing_restore_ack', 'restore_error'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True)
                self.assertEqual(code, 2)
                self.assertTrue(result['pid_restored'])
                self.assertFalse(result['feedforward_restored'])
                self.assertFalse(result['profile_restored'])
                self.assertIn('profile_restore_error', result)
                self.assertTrue(connection.closed)
                self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_kp_candidate_configuration_motion_prevents_velocity(self):
        for fault in ('pid_configuration_motion', 'configuration_motion'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True)
                self.assertEqual(code, 2)
                self.assertFalse(result['velocity_sent'])
                self.assertIn('not stopped', result['reason'])
                self.assertTrue(connection.closed)
                self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_kp_candidate_cleanup_stop_failure_prevents_configuration_restore(self):
        code, result, connection, _ = self.run_fake('left', 'cleanup_stop_write_error',
                                                 pid_candidate=True)
        self.assertEqual(code, 2)
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['pid_restored'])
        self.assertFalse(result['feedforward_restored'])
        self.assertFalse(result['profile_restored'])
        self.assertFalse(any(x == b'PID,80,200,0,75\n' for x in connection.writes))
        self.assertTrue(connection.closed)
        self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_three_seconds_changes_only_nominal_duration_and_still_restores(self):
        code, result, connection, timeouts = self.run_fake('left', pid_candidate=True,
                                                         three_seconds=True)
        self.assertEqual(code, 0, result['reason'])
        self.assertEqual(result['requested_seconds'], 3.)
        self.assertGreater(result['velocity_to_stop_elapsed_s'], 2.9)
        self.assertLess(result['velocity_to_stop_elapsed_s'], 3.)
        self.assertLess(result['elapsed_s'], 12.)
        for key, value in (('target_ticks_per_second', 200), ('pwm_ceiling', 100),
                           ('no_progress_seconds', .25), ('no_response_seconds', .30),
                           ('stale_seconds', .15), ('ack_seconds', .15),
                           ('startup_target_grace_seconds', .10), ('refresh_seconds', .08),
                           ('stop_scheduling_margin_s', .03),
                           ('max_feedback_ticks_per_second', 1000)):
            self.assertEqual(result[key], value)
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(result['profile_restored'])
        self.assertTrue(result['pid_restored'])
        self.assertTrue(result['feedforward_restored'])
        self.assertTrue(connection.closed)
        self.assertEqual(connection.pid, [80, 200, 0])
        self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
        self.assertEqual(connection.targets, [0, 0])
        self.assertEqual(connection.writes[-1], b'STOP\n')
        self.assertEqual([x for x in connection.writes if x.startswith((b'PID,', b'FF,'))],
                         [b'PID,160,200,0,75\n', b'FF,85,60,75,60\n',
                          b'PID,80,200,0,75\n', b'FF,75,60,75,60\n'])
        velocities = [x for x in result['commands'] if x['command'].startswith('VEL,')]
        self.assertGreater(len(velocities), 30)
        self.assertTrue(all(x['command'] == 'VEL,200,0' for x in velocities))
        self.assertTrue(all(b['elapsed_s']-a['elapsed_s'] < .350
                            for a, b in zip(velocities, velocities[1:])))
        self.assertEqual(timeouts[:2], [.2, .2])
        self.assertEqual(timeouts[-1], .2)
        self.assertTrue(all(t == .05 for t in timeouts[2:-1]))

    def test_three_seconds_early_motion_faults_keep_original_abort_times(self):
        for fault in ('reverse', 'stall', 'no_response', 'missing_ack', 'wrong_ack',
                      'write_error', 'interrupt', 'partial_line', 'stale',
                      'wrong_target', 'other_pwm', 'power'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True,
                                                          three_seconds=True)
                self.assertEqual(code, 2)
                self.assertLess(result['velocity_to_stop_elapsed_s'], .6)
                self.assertTrue(result['stop_confirmed'])
                self.assertTrue(result['profile_restored'])
                self.assertEqual(connection.targets, [0, 0])
                self.assertTrue(connection.closed)

    def test_three_seconds_faults_after_one_second_still_stop_and_restore(self):
        for fault in ('late_stall', 'late_stale', 'late_reverse', 'late_power',
                      'late_wrong_target', 'late_other_pwm', 'late_overspeed'):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True,
                                                          three_seconds=True)
                self.assertEqual(code, 2)
                self.assertGreater(result['velocity_to_stop_elapsed_s'], 1.2)
                self.assertLess(result['velocity_to_stop_elapsed_s'], 1.9)
                self.assertTrue(result['stop_confirmed'])
                self.assertTrue(result['profile_restored'])
                self.assertEqual(connection.pid, [80, 200, 0])
                self.assertEqual(connection.ff, list(module.BASELINE_FEEDFORWARD))
                self.assertEqual(connection.targets, [0, 0])
                self.assertTrue(connection.closed)
                self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_three_seconds_configuration_and_restore_faults_fail_without_retry(self):
        config_faults = ('missing_pid_candidate_ack', 'wrong_pid_candidate_ack',
                         'partial_pid_candidate_write', 'missing_candidate_ack',
                         'wrong_candidate_ack', 'partial_candidate_write')
        restore_faults = ('missing_pid_restore_ack', 'pid_restore_error',
                          'partial_pid_restore_write', 'missing_restore_ack', 'restore_error')
        for fault in (*config_faults, *restore_faults):
            with self.subTest(fault=fault):
                code, result, connection, _ = self.run_fake('left', fault, pid_candidate=True,
                                                          three_seconds=True)
                self.assertEqual(code, 2)
                if fault in config_faults:
                    self.assertFalse(result['velocity_sent'])
                    self.assertTrue(result['profile_restored'])
                else:
                    self.assertTrue(result['stop_confirmed'])
                    self.assertFalse(result['profile_restored'])
                    self.assertIn('profile_restore_error', result)
                self.assertEqual(connection.targets, [0, 0])
                self.assertTrue(connection.closed)
                self.assertEqual(connection.writes[-1], b'STOP\n')

    def test_three_seconds_cleanup_stop_failure_cannot_claim_restoration(self):
        code, result, connection, _ = self.run_fake('left', 'cleanup_stop_write_error',
                                                 pid_candidate=True, three_seconds=True)
        self.assertEqual(code, 2)
        self.assertTrue(result['stop_confirmed'])
        self.assertFalse(result['profile_restored'])
        self.assertFalse(any(x == b'PID,80,200,0,75\n' for x in connection.writes))
        self.assertTrue(connection.closed)
        self.assertEqual(connection.writes[-1], b'STOP\n')


if __name__ == '__main__':
    unittest.main()
