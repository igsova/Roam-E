"""Mocked host checks only: no device, motor command or physics simulation."""
import contextlib
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).parent))
import check_r19_response_id as m


class MainTest(unittest.TestCase):
    def fake(self, episode='duty-forward', fault=None):
        class Clock:
            now = 0.
            def monotonic(self): return self.now
            def sleep(self, x): self.now += x
        clock = Clock()

        class Connection:
            def __init__(self):
                self.pending = []; self.targets = [0, 0]; self.duty = None; self.counts = [0, 0]
                self.enabled = False; self.fast = False; self.last_tick = 0.; self.last_ms = 0
                self.writes = []; self.closed = False; self.moved = False
                self.motion_started = None; self.event_sent = False
            def reset_input_buffer(self):
                self.pending = []; self.last_tick = clock.now; self.last_ms = int(clock.now * 1000)
            def write(self, data):
                self.writes.append(data)
                text = data.decode().strip()
                ack = None
                if text == 'STOP':
                    self.targets = [0, 0]; self.duty = None
                    if fault != 'missing_stop' or not self.moved: ack = 'OK,STOP'
                elif text == 'INFO': ack = m.INFO
                elif text == 'CAPS':
                    ack = ('CAPS,BOUNDED_DUAL_V3,128,153,300,250,1000,6000,1400,160'
                           if fault == 'v3_caps' else m.CAPS)
                elif text == 'START,1': self.enabled = True; ack = 'OK,START,1'
                elif text == 'START,0': self.enabled = False; self.fast = False; ack = 'OK,START,0'
                elif text in ('PWMF,0', 'PWMF,1'):
                    if fault != 'pwmf_restore' or text != 'PWMF,0' or not self.moved:
                        self.fast = text.endswith('1'); ack = 'OK,' + text
                elif text in ('PID,200,800,0,85', 'FF,85,85,85,85', 'PID,80,200,0,75', 'FF,75,60,75,60'):
                    ack = 'OK,' + text
                elif text.startswith(('VEL,', 'DUTY,')):
                    values = [int(x) for x in text.split(',')[1:]]
                    if text.startswith('DUTY,'):
                        self.duty = values; s = 1 if values[0] > 0 else -1
                        self.targets = [s * 200, s * 200]
                    else:
                        self.targets = values
                    if self.motion_started is None: self.motion_started = clock.now
                    self.moved = True
                    if fault != 'missing_ack': ack = 'OK,DUTY,1,1' if fault == 'wrong_ack' else 'OK,' + text
                else: raise AssertionError('Unexpected write: ' + text)
                if ack: self.pending.append((ack + '\n').encode())
                return len(data)
            def readline(self, limit):
                if self.pending:
                    clock.now += .002
                    return self.pending.pop(0)
                clock.now += .01
                if clock.now - self.last_tick < .05: return b''
                active = any(self.targets)
                if active and fault == 'stale': return b''
                ms = int(round(clock.now * 1000)); delta = ms - self.last_ms
                self.last_ms = ms; self.last_tick = clock.now
                sign = 1 if self.targets[0] >= 0 else -1
                output = [0, 0]; measured = [0, 0]; phases = [0, 0]
                if active:
                    run = clock.now - self.motion_started >= .1
                    for side in (0, 1):
                        phases[side] = 2 if run else 1
                        output[side] = (self.duty[side] if self.duty and run else sign * 128)
                        if fault == f'wrong_duty_{side}' and run: output[side] += sign
                        measured[side] = sign * (1200 if fault == f'overspeed_{side}' else 200)
                        rate = .35 if fault == f'separation_{side}' else .2
                        if fault not in ('no_response', f'no_response_{side}'):
                            self.counts[side] += (-sign if fault == f'negative_{side}' else sign) * max(1, round(rate * delta))
                    if fault == 'event' and not self.event_sent:
                        self.event_sent = True
                        return b'EVENT,BOUNDED_FAULT,5\n'
                    if self.enabled:
                        self.pending.append(('I,' + ','.join(str(x) for x in [ms, *self.counts, 1000 * ms, 1000 * ms, *output]) + '\n').encode())
                if self.enabled:
                    self.pending.append(('B,' + ','.join(str(x) for x in [ms, *phases, 0]) + '\n').encode())
                fields = [ms, *self.counts, *output, *self.targets, *measured]
                return ('T,' + ','.join(str(x) for x in fields) + '\n').encode()
            def close(self): self.closed = True
        conn = Connection()

        def command(args, timeout=2):
            if args[0] == 'udevadm': return 'ID_VENDOR_ID=1a86\nID_MODEL_ID=7523'
            if args == ['vcgencmd', 'get_throttled']:
                return 'throttled=0x5' if any(conn.targets) and fault == 'power' else 'throttled=0x0'
            if args == ['vcgencmd', 'pmic_read_adc', 'EXT5V_V']: return 'EXT5V_V=5.03V'
            raise AssertionError('Unexpected diagnostic')

        def serial(*args, **kwargs):
            self.assertEqual(args, (m.PORT, 115200))
            self.assertEqual(kwargs, {'timeout': .01, 'write_timeout': .05, 'exclusive': True})
            return conn

        class FakeMonitor:
            def __init__(self, reader, clock):
                self.reader, self.clock = reader, clock; self.last = -1.
            def start(self): pass
            def check(self):
                if self.clock() - self.last < m.POLL_INTERVAL: return []
                self.last = self.clock()
                raw = self.reader(['vcgencmd', 'get_throttled'], timeout=m.QUERY_TIMEOUT)
                return [{'began': self.last, 'completed': self.last, 'flags': int(raw.split('=')[1], 16)}]
            def close(self):
                if any(conn.targets): raise AssertionError('Power-monitor join while moving')
                return True

        out = io.StringIO()
        with patch.object(sys, 'argv', ['test', *(['--episode', episode] if episode else [])]), \
             patch.object(m, 'time', clock), patch.object(m.signal, 'signal'), \
             patch.object(m, 'command', side_effect=command), \
             patch.object(m, 'PowerMonitor', FakeMonitor), \
             patch.object(m.subprocess, 'run', return_value=SimpleNamespace(returncode=1, stdout='', stderr='')), \
             patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=serial)}), \
             contextlib.redirect_stdout(out):
            code = m.main()
        return code, json.loads(out.getvalue()), conn

    def test_default_no_motion(self):
        code, r, c = self.fake(None)
        self.assertEqual(code, 0); self.assertFalse(r['motion_sent'])
        self.assertTrue(r['stop_confirmed']); self.assertTrue(c.closed)
        self.assertEqual(set(c.writes), {b'STOP\n', b'INFO\n', b'CAPS\n'})

    def test_declared_episodes_restore(self):
        for episode, spec in m.EPISODES.items():
            with self.subTest(episode=episode):
                code, r, c = self.fake(episode)
                self.assertEqual(code, 0, r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and r['serial_closed']
                                and r['power_monitor_closed'])
                self.assertTrue(all(r['run_phase_seen']))
                self.assertTrue(all(spec['sign'] * x > 0 for x in r['encoder_delta']))
                self.assertGreater(r['motion_to_stop_s'], m.SECONDS - .05)
                self.assertLess(r['motion_to_stop_s'], m.SECONDS + .02)
                self.assertTrue(r['steps'], 'No I lines recorded')
                motion = [x for x in c.writes if x.startswith((b'VEL,', b'DUTY,'))]
                if spec['mode'] == 'duty':
                    levels = {tuple(x[1:]) for x in spec['schedule']}
                    sent = [tuple(int(v) for v in x.decode().strip().split(',')[1:]) for x in motion]
                    self.assertTrue(set(sent) <= levels and len(set(sent)) == 2)
                    self.assertEqual(sent[0], spec['schedule'][0][1:])
                    self.assertNotIn(b'PID,200,800,0,85\n', c.writes)
                    self.assertIn(b'PWMF,0\n', c.writes[:c.writes.index(motion[0])])
                else:
                    self.assertTrue(all(x == f'VEL,{spec["sign"] * 200},{spec["sign"] * 200}\n'.encode()
                                        for x in motion))
                    self.assertIn(b'PID,200,800,0,85\n', c.writes)
                    flag = b'PWMF,1\n' if spec['timer1_fast'] else b'PWMF,0\n'
                    self.assertIn(flag, c.writes[:c.writes.index(motion[0])])
                    if not spec['timer1_fast']:
                        self.assertNotIn(b'PWMF,1\n', c.writes)
                tail = c.writes[c.writes.index(b'STOP\n', c.writes.index(motion[-1])):]
                for cmd in m.RESTORE: self.assertIn(cmd, tail)
                self.assertFalse(c.enabled or c.fast)
                self.assertEqual(c.targets, [0, 0]); self.assertEqual(c.writes[-1], b'STOP\n')

    def test_step_time(self):
        code, r, c = self.fake('duty-reverse')
        self.assertEqual(code, 0, r['reason'])
        cmds = [x for x in r['commands'] if x['command'].startswith('DUTY,')]
        first = cmds[0]['elapsed_s']
        step = next(x for x in cmds if x['command'] == 'DUTY,-87,-109')['elapsed_s']
        self.assertGreaterEqual(step - first, m.STEP_AT)
        self.assertLess(step - first, m.STEP_AT + m.REFRESH + .03)

    def test_v3_image_refused(self):
        code, r, c = self.fake('duty-forward', 'v3_caps')
        self.assertEqual(code, 2); self.assertFalse(r['motion_sent'])
        self.assertNotIn(b'START,1\n', c.writes)

    def test_faults_stop_and_restore(self):
        faults = ('missing_ack', 'wrong_ack', 'stale', 'no_response', 'no_response_1',
                  'negative_0', 'overspeed_1', 'separation_0', 'wrong_duty_1', 'event', 'power')
        for fault in faults:
            with self.subTest(fault=fault):
                code, r, c = self.fake('duty-forward', fault)
                self.assertEqual(code, 2); self.assertTrue(r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and r['serial_closed'])
                self.assertLess(r['motion_to_stop_s'], m.SECONDS)
                self.assertFalse(c.enabled or c.fast)
                self.assertEqual(c.writes[-1], b'STOP\n')

    def test_restore_failure_reported(self):
        code, r, c = self.fake('vel-forward-t1fast', 'pwmf_restore')
        self.assertEqual(code, 2); self.assertIn('PWMF,0', r['reason'])
        self.assertFalse(r['restore_confirmed']); self.assertTrue(r['serial_closed'])
        self.assertEqual(c.writes[-1], b'STOP\n')

    def test_step_line_shape(self):
        self.assertEqual(m.parse_i('I,20,1,2,3,4,90,-90'), [20, 1, 2, 3, 4, 90, -90])
        for bad in ('I,20,1,2,3,4,90', 'I,20,1,2,3,4,154,0', 'I,-1,1,2,3,4,0,0', 'I,1,1,2,3,4294967296,0,0'):
            with self.subTest(bad=bad), self.assertRaises(ValueError): m.parse_i(bad)
        self.assertIsNone(m.parse_i('T,1'))

    def test_declared_duties_within_firmware_window(self):
        for spec in m.EPISODES.values():
            for _, left, right in spec.get('schedule', []):
                for value in (left, right):
                    self.assertEqual(value > 0, spec['sign'] > 0)
                    self.assertTrue(40 <= abs(value) <= 120 and abs(value) <= m.CEILING)


if __name__ == '__main__':
    unittest.main()
