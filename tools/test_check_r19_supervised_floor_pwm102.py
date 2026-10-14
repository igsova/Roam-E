"""Offline only: mocked serial/power/time; never opens hardware."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent))
spec = importlib.util.spec_from_file_location(
    'floor_pwm102', Path(__file__).with_name('check_r19_supervised_floor_pwm102.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
ARM = '--arm-supervised-floor-left-forward-pwm102-one-second'


class GuardTest(unittest.TestCase):
    def setUp(self):
        self.guard = module.FeedbackGuard([1000, 0, 0, 0, 0, 0, 0, 0, 0], 0.)

    def test_fixed_command_and_limits(self):
        self.assertEqual(module.pulse_command(), b'PWM,102,0\n')
        self.assertEqual(module.EXPECTED_INFO, 'INFO,2,748,153,2000')
        self.assertEqual(module.PWM, 102)
        self.assertEqual(module.PWM_CEILING, 153)
        self.assertEqual(module.REQUESTED_SECONDS, 1.)
        self.assertEqual(module.STOP_MARGIN, .03)
        self.assertEqual(module.REFRESH_SECONDS, .08)
        self.assertEqual(module.STALE_SECONDS, .15)
        self.assertEqual(module.NO_RESPONSE_SECONDS, .30)
        self.assertEqual(module.FLOOR_NO_PROGRESS_SECONDS, .25)

    def test_parser_ceiling(self):
        self.assertEqual(module.parse_telemetry('T,50,2,0,153,0,0,0,0,0')[3], 153)
        self.assertIsNone(module.parse_telemetry('OK,STOP'))
        for line in ('T,50,0,0,154,0,0,0,0,0', 'T,50,0,0,-154,0,0,0,0,0',
                     'T,50,0', 'T,-1,0,0,0,0,0,0,0,0',
                     'T,50,2147483648,0,0,0,0,0,0,0'):
            with self.subTest(line=line), self.assertRaises(ValueError):
                module.parse_telemetry(line)

    def test_progress_and_passive_other(self):
        for step in range(1, 18):
            now = step*.05
            self.guard.acknowledge(now)
            self.guard.observe([1000+50*step, step, -step, 102, 0, 0, 0, 0, 0], now)
            self.guard.check(now)
        self.assertTrue(self.guard.responded)

    def test_plateau(self):
        for step in range(1, 8):
            now = step*.05
            self.guard.acknowledge(now)
            self.guard.observe([1000+50*step, 1, 0, 102, 0, 0, 0, 0, 0], now)
        with self.assertRaisesRegex(RuntimeError, 'progress stopped'):
            self.guard.check(.351)

    def test_no_response(self):
        for step in range(1, 7):
            now = step*.05
            self.guard.acknowledge(now)
            self.guard.observe([1000+50*step, 0, 1, 102, 0, 0, 0, 0, 0], now)
        with self.assertRaisesRegex(RuntimeError, 'No encoder response'):
            self.guard.check(.301)

    def test_wrong_direction(self):
        with self.assertRaisesRegex(RuntimeError, 'Negative'):
            self.guard.observe([1050, -1, 0, 102, 0, 0, 0, 0, 0], .05)

    def test_stale(self):
        self.guard.acknowledge(.15)
        with self.assertRaisesRegex(RuntimeError, 'Stale'):
            self.guard.check(.151)

    def test_missing_ack(self):
        self.guard.observe([1050, 1, 0, 102, 0, 0, 0, 0, 0], .15)
        with self.assertRaisesRegex(RuntimeError, 'acknowledgement'):
            self.guard.check(.151)

    def test_wrong_output_target_or_clock(self):
        for row in ([1050, 1, 0, 102, 1, 0, 0, 0, 0],
                    [1050, 1, 0, 152, 0, 0, 0, 0, 0],
                    [1050, 1, 0, 102, 0, 200, 0, 0, 0],
                    [1000, 1, 0, 102, 0, 0, 0, 0, 0],
                    [200, 1, 0, 102, 0, 0, 0, 0, 0],
                    [1300, 1, 0, 102, 0, 0, 0, 0, 0]):
            with self.subTest(row=row), self.assertRaises(RuntimeError):
                self.guard.observe(row, .05)

    def test_counter_wrap(self):
        guard = module.FeedbackGuard([2**32-25, 2**31-1, 0, 0, 0, 0, 0, 0, 0], 0.)
        guard.observe([25, -2**31, 0, 102, 0, 0, 0, 0, 0], .05)
        self.assertTrue(guard.responded)

    def test_invalid_cli_never_touches_hardware(self):
        for arguments in (['--pwm', '204'], ['--pulse-channel', 'right'],
                          ['--pulse-direction', 'reverse'], ['--seconds', '3'],
                          ['--arm-supervised-lifted-one-second']):
            with self.subTest(arguments=arguments), \
                    patch.object(sys, 'argv', ['test', *arguments]), \
                    patch.object(module, 'command') as hardware, \
                    contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    module.main()
                hardware.assert_not_called()


class MainTest(unittest.TestCase):
    def run_fake(self, armed=True, fault=None):
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
                self.pwm = 0
                self.count = 0
                self.other_count = 0
                self.milliseconds = 0
                self.closed = False
                self.writes = []
                self.ever_pulsed = False
            def reset_input_buffer(self):
                self.pending.clear()
            def write(self, data):
                self.writes.append(data)
                if data == b'STOP\n':
                    self.pwm = 0
                    if fault != 'missing_stop' or not self.ever_pulsed:
                        self.pending.append(b'OK,STOP\n')
                elif data == b'INFO\n':
                    info = 'INFO,2,748,100,2000' if fault == 'old_info' else module.EXPECTED_INFO
                    self.pending.append((info+'\n').encode())
                elif data == b'PWM,102,0\n':
                    self.pwm = 102
                    self.ever_pulsed = True
                    if fault == 'partial_write':
                        return len(data)-1
                    if fault != 'missing_ack':
                        self.pending.append(b'OK,PWM,101,0\n' if fault == 'wrong_ack'
                                            else b'OK,PWM,102,0\n')
                else:
                    raise AssertionError('Unexpected command: '+repr(data))
                return len(data)
            def readline(self, limit):
                clock.now += .05
                self.milliseconds += 50
                if self.pending:
                    return self.pending.pop(0)
                if self.pwm and fault == 'stale':
                    return b''
                if self.pwm:
                    self.other_count -= 1
                    if fault != 'no_response' and (fault != 'plateau' or self.count == 0):
                        self.count += -5 if fault == 'negative' else 5
                other_pwm = 1 if self.pwm and fault == 'other_output' else 0
                target = 200 if self.pwm and fault == 'velocity_target' else 0
                clock_value = 0 if self.pwm and fault == 'reset' else self.milliseconds
                fields = [clock_value, self.count, self.other_count, self.pwm,
                          other_pwm, target, 0, 0, 0]
                return ('T,'+','.join(str(x) for x in fields)+'\n').encode()
            def close(self):
                self.closed = True
        connection = Connection()
        def open_serial(*args, **kwargs):
            self.assertEqual(args, (module.PORT, 115200))
            self.assertEqual(kwargs, {'timeout': .01, 'write_timeout': .05,
                                      'exclusive': True})
            return connection
        def hardware_command(args, timeout=2):
            if args[0] == 'udevadm':
                return 'ID_VENDOR_ID=1a86\nID_MODEL_ID=7523'
            if args == ['vcgencmd', 'get_throttled']:
                return 'throttled=0x5' if connection.pwm and fault == 'power' else 'throttled=0x0'
            if args == ['vcgencmd', 'pmic_read_adc', 'EXT5V_V']:
                return 'EXT5V_V volt(24)=5.03000000V'
            raise AssertionError('Unexpected diagnostic command')
        output = io.StringIO()
        with patch.object(sys, 'argv', ['test', *([ARM] if armed else [])]), \
                patch.object(module, 'command', side_effect=hardware_command), \
                patch.object(module, 'time', clock), \
                patch.object(module.signal, 'signal'), \
                patch.object(module.subprocess, 'run',
                             return_value=SimpleNamespace(returncode=1, stdout='', stderr='')), \
                patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=open_serial)}), \
                contextlib.redirect_stdout(output):
            code = module.main()
        return code, json.loads(output.getvalue()), connection

    def test_default_stop_info_only(self):
        code, result, conn = self.run_fake(armed=False)
        self.assertEqual(code, 0)
        self.assertFalse(result['pulse_sent'])
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(conn.closed)
        self.assertTrue(all(x in (b'STOP\n', b'INFO\n') for x in conn.writes))

    def test_one_bounded_pulse_and_stop(self):
        code, result, conn = self.run_fake()
        self.assertEqual(code, 0)
        self.assertLess(result['pulse_to_stop_elapsed_s'], 1.06)
        self.assertGreater(result['pulse_to_stop_elapsed_s'], .90)
        self.assertTrue(result['stop_confirmed'])
        self.assertTrue(result['serial_closed'])
        self.assertGreater(result['encoder_delta'][0], 0)
        self.assertLess(result['encoder_delta'][1], 0) # Passive floor roll permitted.
        self.assertEqual(conn.writes[-1], b'STOP\n')
        self.assertTrue(all(x in (b'STOP\n', b'INFO\n', b'PWM,102,0\n') for x in conn.writes))

    def test_legacy_info_blocks_motion(self):
        code, result, conn = self.run_fake(fault='old_info')
        self.assertEqual(code, 2)
        self.assertFalse(result['pulse_sent'])
        self.assertFalse(any(x.startswith(b'PWM,') for x in conn.writes))
        self.assertTrue(conn.closed)

    def test_faults_stop_early_close_and_never_retry(self):
        for fault in ('no_response', 'plateau', 'negative', 'stale', 'missing_ack',
                      'wrong_ack', 'partial_write', 'other_output', 'velocity_target',
                      'reset', 'power'):
            with self.subTest(fault=fault):
                code, result, conn = self.run_fake(fault=fault)
                self.assertEqual(code, 2)
                self.assertIsNotNone(result['reason'])
                self.assertLess(result['pulse_to_stop_elapsed_s'], .7)
                self.assertTrue(conn.closed)
                self.assertTrue(result['stop_confirmed'])
                self.assertEqual(conn.pwm, 0)
                first_stop_after_pwm = next(i for i, x in enumerate(conn.writes)
                                            if x == b'STOP\n' and b'PWM,102,0\n' in conn.writes[:i])
                self.assertFalse(any(x.startswith(b'PWM,') for x in conn.writes[first_stop_after_pwm:]))

    def test_unconfirmed_stop_is_not_success(self):
        code, result, conn = self.run_fake(fault='missing_stop')
        self.assertEqual(code, 2)
        self.assertFalse(result['stop_confirmed'])
        self.assertTrue(conn.closed)
        self.assertEqual(conn.pwm, 0)


if __name__ == '__main__':
    unittest.main()
