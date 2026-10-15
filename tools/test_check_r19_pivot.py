"""Mocked host checks for the pivot helper; no device or physics."""
import contextlib, io, json, sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).parent))
import check_r19_pivot as m


class FullPwmTest(unittest.TestCase):
    def fake(self, pulse=True, fault=None, duty=None, profile=None):
        class Clock:
            now = 0.
            def monotonic(self): return self.now
            def sleep(self, x): self.now += x
        clock = Clock()

        class Conn:
            def __init__(self):
                self.pending = []; self.tgt = 0; self.vel = [0, 0]; self.pwm = [0, 0]; self.counts = [0, 0]; self.last = 0.; self.ms = 0
                self.writes = []; self.closed = False; self.started = None
            def reset_input_buffer(self): self.pending = []; self.last = clock.now
            def write(self, data):
                self.writes.append(data); t = data.decode().strip(); ack = None
                if t == 'STOP': self.tgt = 0; self.vel = [0, 0]; ack = 'OK,STOP'
                elif t == 'INFO': ack = m.INFO
                elif t == 'CAPS': ack = m.CAPS if fault != 'old' else 'CAPS,BOUNDED_DUAL_V3,128,153,300,250,1000,6000,1400,160'
                elif t in (m.PID, m.FF, 'START,0', m.RESTORE_PID, m.RESTORE_FF): ack = 'OK,' + t
                elif t.startswith('VEL,'):
                    self.vel = [int(x) for x in t.split(',')[1:]]; self.tgt = 1; ack = None if fault == 'noack' else 'OK,' + t
                else: raise AssertionError(t)
                if ack: self.pending.append((ack + '\n').encode())
                return len(data)
            def readline(self, n):
                if self.pending: clock.now += .002; return self.pending.pop(0)
                clock.now += .01
                if clock.now - self.last < .05: return b''
                self.last = clock.now; self.ms += 50
                if self.tgt:
                    r = [self.vel[0] / 20, self.vel[1] / 20]
                    if fault == 'separate': r = [r[0] * .5, r[1]]
                    if fault == 'back': r = [-4 * r[0] / abs(r[0]), r[1]]
                    if fault == 'stall': r = [0, r[1]]
                    if fault == 'fast': r = [120 * r[0] / abs(r[0]), r[1]]
                    self.counts = [c + int(x) for c, x in zip(self.counts, r)]
                vel = self.vel if self.tgt else [0, 0]
                pwm = [90 * (v // abs(v)) if v else 0 for v in vel]
                return ('T,' + ','.join(str(x) for x in [self.ms, *self.counts, *pwm, *vel, 0, 0]) + '\n').encode()
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
        with patch.object(sys, 'argv', ['t', *(['--turn', profile or 'left'] if pulse else [])]), patch.object(m, 'time', clock), \
             patch.object(m.signal, 'signal'), patch.object(m, 'command', side_effect=command), \
             patch.object(m, 'PowerMonitor', Mon), \
             patch.object(m.subprocess, 'run', return_value=SimpleNamespace(returncode=1, stdout='', stderr='')), \
             patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=lambda *a, **k: conn)}), \
             contextlib.redirect_stdout(out):
            code = m.main()
        return code, json.loads(out.getvalue()), conn

    def test_default_no_motion(self):
        code, r, c = self.fake(False)
        self.assertEqual(code, 0); self.assertFalse(any(w.startswith((b'VEL', b'PID', b'FF')) for w in c.writes))

    def test_both_turns_planned_stop(self):
        for turn, signs in m.TURNS.items():
            with self.subTest(turn=turn):
                code, r, c = self.fake(profile=turn)
                self.assertEqual(code, 0, r['reason']); self.assertTrue(r['planned_stop'] and r['restore_confirmed'])
                sent = {w for w in c.writes if w.startswith(b'VEL,')}
                self.assertEqual(sent, {f'VEL,{signs[0]*300},{signs[1]*300}\n'.encode()})
                self.assertAlmostEqual(abs(r['encoder_rotation_deg_incl_coast']), 360, delta=15)
                self.assertEqual(r['encoder_rotation_deg_incl_coast'] > 0, turn == 'left')
                self.assertEqual(c.writes[-1], b'STOP\n')

    def test_faults_stop(self):
        for fault in ('separate', 'fast', 'back', 'stall', 'noack', 'old'):
            with self.subTest(fault=fault):
                code, r, c = self.fake(fault=fault)
                self.assertEqual(code, 2); self.assertTrue(r['reason'])
                self.assertEqual(c.writes[-1], b'STOP\n'); self.assertTrue(c.closed)
                if fault != 'old': self.assertTrue(r['restore_confirmed'])
                else: self.assertFalse(r['motion_sent'])


if __name__ == '__main__':
    unittest.main()
