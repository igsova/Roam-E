"""Mocked host checks for the one PWM230 pulse helper; no device or physics."""
import contextlib, io, json, sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).parent))
import check_r19_full_pwm as m


class FullPwmTest(unittest.TestCase):
    def fake(self, pulse=True, fault=None, duty=None):
        class Clock:
            now = 0.
            def monotonic(self): return self.now
            def sleep(self, x): self.now += x
        clock = Clock()

        class Conn:
            def __init__(self):
                self.pending = []; self.pwm = [0, 0]; self.counts = [0, 0]; self.last = 0.; self.ms = 0
                self.writes = []; self.closed = False; self.started = None
            def reset_input_buffer(self): self.pending = []; self.last = clock.now
            def write(self, data):
                self.writes.append(data); t = data.decode().strip(); ack = None
                if t == 'STOP': self.pwm = [0, 0]; ack = 'OK,STOP'
                elif t == 'INFO': ack = m.INFO
                elif t == 'CAPS': ack = m.CAPS if fault != 'old' else 'CAPS,BOUNDED_DUAL_V3,128,153,300,250,1000,6000,1400,160'
                elif t in ('PWM,230,230', 'PWM,179,179'):
                    self.pwm = [int(t.split(',')[1])] * 2; ack = None if fault == 'noack' else 'OK,' + t
                    if self.started is None: self.started = clock.now
                else: raise AssertionError(t)
                if ack: self.pending.append((ack + '\n').encode())
                return len(data)
            def readline(self, n):
                if self.pending: clock.now += .002; return self.pending.pop(0)
                clock.now += .01
                if clock.now - self.last < .05: return b''
                self.last = clock.now; self.ms += 50
                if any(self.pwm):
                    rates = [60, 62]
                    if fault == 'separate': rates = [20, 60]
                    if fault == 'fast': rates = [120, 50]
                    if fault == 'back': rates = [-4, 50]
                    if fault == 'stall': rates = [0, 50]
                    self.counts = [c + r for c, r in zip(self.counts, rates)]
                return ('T,' + ','.join(str(x) for x in [self.ms, *self.counts, *self.pwm, 0, 0, 0, 0]) + '\n').encode()
            def close(self): self.closed = True
        conn = Conn()

        def command(args, timeout=2):
            if args[0] == 'udevadm': return 'ID_VENDOR_ID=1a86\nID_MODEL_ID=7523'
            if args == ['vcgencmd', 'get_throttled']: return 'throttled=0x0'
            raise AssertionError(args)

        class Mon:
            def __init__(self, reader, clock): self.last = -1.; self.clock = clock; self.reader = reader
            def start(self): pass
            def check(self):
                if self.clock() - self.last < m.POLL_INTERVAL: return []
                self.last = self.clock(); return [{'began': self.last, 'completed': self.last, 'flags': 0}]
            def close(self): return True
        out = io.StringIO()
        with patch.object(sys, 'argv', ['t', *(['--pulse'] if pulse else []), *(['--duty', str(duty)] if duty else [])]), patch.object(m, 'time', clock), \
             patch.object(m.signal, 'signal'), patch.object(m, 'command', side_effect=command), \
             patch.object(m, 'PowerMonitor', Mon), \
             patch.object(m.subprocess, 'run', return_value=SimpleNamespace(returncode=1, stdout='', stderr='')), \
             patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=lambda *a, **k: conn)}), \
             contextlib.redirect_stdout(out):
            code = m.main()
        return code, json.loads(out.getvalue()), conn

    def test_default_no_motion(self):
        code, r, c = self.fake(False)
        self.assertEqual(code, 0); self.assertFalse(any(w.startswith(b'PWM') for w in c.writes))

    def test_pulse_success_records_peak(self):
        code, r, c = self.fake()
        self.assertEqual(code, 0, r['reason']); self.assertTrue(r['stop_confirmed'])
        self.assertGreater(r['peak_ticks_per_s'][1], 1000)
        self.assertLess(r['motion_to_stop_s'], m.SECONDS + .02)
        self.assertEqual(c.writes[-1], b'STOP\n'); self.assertTrue(c.closed)

    def test_seventy_percent(self):
        code, r, c = self.fake(duty=179)
        self.assertEqual(code, 0, r['reason']); self.assertIn(b'PWM,179,179\n', c.writes)
        self.assertNotIn(b'PWM,230,230\n', c.writes); self.assertEqual(r['duty'], 179)

    def test_faults_stop(self):
        for fault in ('separate', 'fast', 'back', 'stall', 'noack', 'old'):
            with self.subTest(fault=fault):
                code, r, c = self.fake(fault=fault)
                self.assertEqual(code, 2); self.assertTrue(r['reason'])
                self.assertEqual(c.writes[-1], b'STOP\n'); self.assertTrue(c.closed)
                if fault == 'old': self.assertFalse(r['motion_sent'])
                else: self.assertLess(r['motion_to_stop_s'], m.SECONDS)


if __name__ == '__main__':
    unittest.main()
