"""Mocked host checks only: no device, motor command or physics simulation."""
import contextlib
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).parent))
import check_r19_bounded_speed as m

class MainTest(unittest.TestCase):
    def fake(self,episode='left-forward-1s',fault=None,profile='baseline'):
        class Clock:
            now=0.
            def monotonic(self): return self.now
            def sleep(self,x): self.now+=x
        clock=Clock()
        class Connection:
            def __init__(self):
                self.pending=[]; self.targets=[0,0]; self.counts=[0,0]
                self.enabled=False; self.last_tick=0.; self.last_ms=0
                self.writes=[]; self.closed=False; self.moved=False
                self.motion_started=None; self.event_sent=False
            def reset_input_buffer(self):
                self.pending=[]; self.last_tick=clock.now; self.last_ms=int(clock.now*1000)
            def write(self,data):
                self.writes.append(data)
                text=data.decode().strip()
                ack=None
                if text=='STOP':
                    self.targets=[0,0]
                    if fault!='missing_stop' or not self.moved: ack='OK,STOP'
                elif text=='INFO': ack=m.INFO if fault!='old_info' else 'INFO,2,748,100,2000'
                elif text=='CAPS': ack=m.CAPS if fault!='missing_caps' else 'ERR,BAD_COMMAND'
                elif text=='START,1': self.enabled=True; ack='OK,START,1'
                elif text=='START,0': self.enabled=False; ack='OK,START,0'
                elif text in ('PID,160,200,0,85','PID,240,600,0,85','FF,85,85,85,85',
                              'PID,80,200,0,75','FF,75,60,75,60'):
                    if fault!='restore' or text!='PID,80,200,0,75': ack='OK,'+text
                elif text.startswith('VEL,'):
                    self.targets=[int(x) for x in text.split(',')[1:]]
                    if self.motion_started is None: self.motion_started=clock.now
                    self.moved=True
                    if fault=='partial_write': return len(data)-1
                    if fault!='missing_ack': ack='OK,VEL,201,0' if fault=='wrong_ack' else 'OK,'+text
                else: raise AssertionError('Unexpected write: '+text)
                if ack: self.pending.append((ack+'\n').encode())
                return len(data)
            def readline(self,limit):
                if self.pending:
                    clock.now+=.002
                    return self.pending.pop(0)
                clock.now+=.01
                if clock.now-self.last_tick <.05: return b''
                active=any(self.targets)
                if active and fault=='stale': return b''
                ms=int(round(clock.now*1000)); delta=ms-self.last_ms
                self.last_ms=ms; self.last_tick=clock.now
                side=0 if self.targets[0] else 1
                sign=1 if self.targets[side]>=0 else -1
                output=[0,0]; measured=[0,0]; phases=[0,0]
                targets=self.targets[:]
                if active:
                    output[side]=sign*128; measured[side]=sign*200
                    phases[side]=1 if clock.now-self.motion_started<.1 else 2
                    self.counts[1-side]-=2 # Passive roll, unpowered side allowed.
                    if fault!='no_response' and (fault!='plateau' or self.counts[side]==0):
                        self.counts[side]+=(-sign if fault=='negative' else sign)*max(1,round(.2*delta))
                    if fault=='other_output': output[1-side]=1
                    if fault=='target': targets[side]+=1
                    if fault=='overspeed': measured[side]=sign*1200
                    if fault=='bad_phase': phases[1-side]=2
                    if fault=='never_run': phases[side]=1
                    if fault=='event' and not self.event_sent:
                        self.event_sent=True
                        return b'EVENT,BOUNDED_FAULT,5\n'
                if self.enabled and fault!='missing_phase':
                    self.pending.append(('B,'+','.join(str(x) for x in [ms,*phases,0])+'\n').encode())
                fields=[0 if active and fault=='reset' else ms,*self.counts,*output,*targets,*measured]
                return ('T,'+','.join(str(x) for x in fields)+'\n').encode()
            def close(self): self.closed=True
        conn=Connection()
        def command(args,timeout=2):
            if args[0]=='udevadm': return 'ID_VENDOR_ID=1a86\nID_MODEL_ID=7523'
            if args==['vcgencmd','get_throttled']:
                if any(conn.targets) and fault=='power_timeout':
                    raise m.subprocess.TimeoutExpired(args,timeout)
                return 'throttled=0x5' if any(conn.targets) and fault=='power' else 'throttled=0x0'
            if args==['vcgencmd','pmic_read_adc','EXT5V_V']: return 'EXT5V_V=5.03V'
            raise AssertionError('Unexpected diagnostic')
        def serial(*args,**kwargs):
            self.assertEqual(args,(m.PORT,115200))
            self.assertEqual(kwargs,{'timeout':.01,'write_timeout':.05,'exclusive':True})
            return conn
        out=io.StringIO()
        with patch.object(sys,'argv',['test','--profile',profile,*(['--episode',episode] if episode else [])]), \
             patch.object(m,'time',clock), patch.object(m.signal,'signal'), \
             patch.object(m,'command',side_effect=command), \
             patch.object(m.subprocess,'run',return_value=SimpleNamespace(returncode=1,stdout='',stderr='')), \
             patch.dict(sys.modules,{'serial':SimpleNamespace(Serial=serial)}), \
             contextlib.redirect_stdout(out):
            code=m.main()
        return code,json.loads(out.getvalue()),conn

    def test_default_no_motion_or_profile(self):
        code,r,c=self.fake(None)
        self.assertEqual(code,0); self.assertFalse(r['motion_sent'])
        self.assertTrue(r['stop_confirmed']); self.assertTrue(c.closed)
        self.assertTrue(all(x in (b'STOP\n',b'INFO\n',b'CAPS\n') for x in c.writes))

    def test_all_eight_scoped_episodes_restore(self):
        for episode,(side,sign,seconds) in m.EPISODES.items():
            with self.subTest(episode=episode):
                code,r,c=self.fake(episode)
                self.assertEqual(code,0,r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and r['serial_closed'])
                self.assertTrue(r['run_phase_seen'])
                self.assertGreater(sign*r['encoder_delta'][side],0)
                self.assertLess(r['motion_to_stop_s'],seconds+.02)
                self.assertGreater(r['motion_to_stop_s'],seconds-.05)
                targets=[0,0]; targets[side]=sign*200
                expected=('VEL,'+','.join(str(x) for x in targets)+'\n').encode()
                self.assertTrue(all(x==expected for x in c.writes if x.startswith(b'VEL,')))
                self.assertEqual(c.targets,[0,0]); self.assertFalse(c.enabled)
                self.assertEqual(c.writes[-1],b'STOP\n')

    def test_identity_blocks_all_arming(self):
        for fault in ('old_info','missing_caps'):
            with self.subTest(fault=fault):
                code,r,c=self.fake(fault=fault)
                self.assertEqual(code,2); self.assertFalse(r['motion_sent'])
                self.assertFalse(any(x.startswith(b'VEL,') for x in c.writes))
                self.assertTrue(c.closed)

    def test_responsive_profile_all_episodes_and_default_restores(self):
        for episode in (None,*m.EPISODES):
            with self.subTest(episode=episode):
                code,r,c=self.fake(episode,profile='responsive')
                self.assertEqual(code,0,r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and c.closed)
                self.assertEqual(r['profile']['pid'],[240,600,0,85])
                if episode:
                    self.assertIn(b'PID,240,600,0,85\n',c.writes)
                    self.assertIn(b'PID,80,200,0,75\n',c.writes)
                    self.assertEqual(c.targets,[0,0]); self.assertFalse(c.enabled)
                else:
                    self.assertFalse(any(x.startswith((b'PID,',b'FF,',b'START,',b'VEL,')) for x in c.writes))

    def test_responsive_fault_does_not_retry(self):
        code,r,c=self.fake(fault='plateau',profile='responsive')
        self.assertEqual(code,2)
        self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and c.closed)
        first_stop=next(i for i,x in enumerate(c.writes)
                        if x==b'STOP\n' and any(y.startswith(b'VEL,') for y in c.writes[:i]))
        self.assertFalse(any(x.startswith(b'VEL,') for x in c.writes[first_stop:]))

    def test_active_faults_stop_restore_no_retry(self):
        for fault in ('no_response','plateau','negative','stale','missing_ack','wrong_ack',
                      'partial_write','other_output','target','overspeed','bad_phase',
                      'missing_phase','reset','event','power','power_timeout'):
            with self.subTest(fault=fault):
                code,r,c=self.fake(fault=fault)
                self.assertEqual(code,2); self.assertIsNotNone(r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and c.closed,
                                str(r))
                first_stop=next(i for i,x in enumerate(c.writes)
                                if x==b'STOP\n' and any(y.startswith(b'VEL,') for y in c.writes[:i]))
                self.assertFalse(any(x.startswith(b'VEL,') for x in c.writes[first_stop:]))
                self.assertLess(r['motion_to_stop_s'],.7)

    def test_no_run_is_not_success(self):
        code,r,c=self.fake(fault='never_run')
        self.assertEqual(code,2); self.assertFalse(r['run_phase_seen'])
        self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'])

    def test_missing_stop_or_restore_not_success(self):
        for fault in ('missing_stop','restore'):
            with self.subTest(fault=fault):
                code,r,c=self.fake(fault=fault)
                self.assertEqual(code,2); self.assertTrue(c.closed)
                self.assertFalse(r['restore_confirmed'])

    def test_unknown_options_before_hardware(self):
        for args in (['--episode','left-forward-10s'],['--pwm','204'],['--seconds','5'],
                     ['--profile','arbitrary'],['--kp','0.9']):
            with self.subTest(args=args),patch.object(sys,'argv',['test',*args]), \
                 patch.object(m,'command') as hardware,contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit): m.main()
                hardware.assert_not_called()

    def test_parser_rejects_shapes_widths_ceiling(self):
        for line in ('T,50,0,0,154,0,0,0,0,0','T,50,2147483648,0,0,0,0,0,0,0',
                     'T,-1,0,0,0,0,0,0,0,0','T,50,0'):
            with self.subTest(line=line),self.assertRaises(ValueError): m.parse_t(line)
        for line in ('B,50,3,0,0','B,50,0,0,2','B,50,0'):
            with self.subTest(line=line),self.assertRaises(ValueError): m.parse_b(line)

    def test_counter_wrap_guard(self):
        g=m.Guard([2**32-25,2**31-1,0,0,0,0,0,0,0],0,0,1)
        g.observe([25,-2**31,0,128,0,200,0,100,0],.05)
        self.assertTrue(g.responded)

if __name__ == '__main__': unittest.main()
