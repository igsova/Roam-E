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
import check_r19_dual_hold as m

class MainTest(unittest.TestCase):
    def fake(self,episode='both-forward-1s',fault=None,profile='responsive'):
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
                elif text=='CAPS':
                    ack=('CAPS,BOUNDED_START_V1,128,153,300,250,1000,3000' if fault=='old_caps'
                         else ('ERR,BAD_COMMAND' if fault=='missing_caps' else m.CAPS))
                elif text=='START,1': self.enabled=True; ack='OK,START,1'
                elif text=='START,0': self.enabled=False; ack='OK,START,0'
                elif text in ('PID,160,200,0,85','PID,240,600,0,85','PID,320,400,0,85','PID,200,800,0,85','FF,85,85,85,85',
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
                    for side in (0,1):
                        output[side]=sign*128; measured[side]=sign*200
                        phases[side]=1 if clock.now-self.motion_started<.1 else 2
                        no_response=fault in ('no_response',f'no_response_{side}')
                        plateau=fault==f'plateau_{side}' and self.counts[side]!=0
                        if not no_response and not plateau:
                            self.counts[side]+=(-sign if fault==f'negative_{side}' else sign)*max(1,round(.2*delta))
                        if fault==f'wrong_output_{side}': output[side]=-sign*128
                        if fault==f'target_{side}': targets[side]+=1
                        if fault==f'overspeed_{side}': measured[side]=sign*1200
                        if fault==f'bad_phase_{side}': phases[side]=0
                        if fault==f'never_run_{side}': phases[side]=1
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
        class FakeMonitor:
            def __init__(self,reader,clock):
                self.reader,self.clock=reader,clock; self.last=-1.
            def start(self): pass
            def check(self):
                if any(conn.targets) and fault=='power_stale': raise RuntimeError('Stale Pi power sample')
                if not any(conn.targets) and fault=='power_warmup': raise RuntimeError('Pi power monitor unavailable')
                if self.clock()-self.last <m.POLL_INTERVAL: return []
                self.last=self.clock()
                raw=self.reader(['vcgencmd','get_throttled'],timeout=m.QUERY_TIMEOUT)
                return [{'began':self.last,'completed':self.last,'flags':int(raw.split('=')[1],16)}]
            def close(self):
                if any(conn.targets): raise AssertionError('Power-monitor join while moving')
                return True
        out=io.StringIO()
        with patch.object(sys,'argv',['test','--profile',profile,*(['--episode',episode] if episode else [])]), \
             patch.object(m,'time',clock), patch.object(m.signal,'signal'), \
             patch.object(m,'command',side_effect=command), \
             patch.object(m,'PowerMonitor',FakeMonitor), \
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

    def test_all_six_scoped_episodes_restore(self):
        for episode,(sign,seconds) in m.EPISODES.items():
            with self.subTest(episode=episode):
                code,r,c=self.fake(episode)
                self.assertEqual(code,0,r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and r['serial_closed'] and r['power_monitor_closed'])
                self.assertTrue(all(r['run_phase_seen']))
                self.assertTrue(all(sign*x>0 for x in r['encoder_delta']))
                self.assertLess(r['motion_to_stop_s'],seconds+.02)
                self.assertGreater(r['motion_to_stop_s'],seconds-.05)
                expected=f'VEL,{sign*200},{sign*200}\n'.encode()
                self.assertTrue(all(x==expected for x in c.writes if x.startswith(b'VEL,')))
                self.assertIn(b'PID,240,600,0,85\n',c.writes)
                self.assertIn(b'PID,80,200,0,75\n',c.writes)
                self.assertEqual(c.targets,[0,0]); self.assertFalse(c.enabled)
                self.assertEqual(c.writes[-1],b'STOP\n')

    def test_identity_blocks_all_arming(self):
        for fault in ('old_info','missing_caps','old_caps'):
            with self.subTest(fault=fault):
                code,r,c=self.fake(fault=fault)
                self.assertEqual(code,2); self.assertFalse(r['motion_sent'])
                self.assertFalse(any(x.startswith(b'VEL,') for x in c.writes))
                self.assertTrue(c.closed)

    def test_all_six_damped_episodes_restore(self):
        for episode in (None,*m.EPISODES):
            with self.subTest(episode=episode):
                code,r,c=self.fake(episode,profile='damped')
                self.assertEqual(code,0,r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and r['power_monitor_closed'])
                if episode:
                    self.assertIn(b'PID,320,400,0,85\n',c.writes)
                    self.assertIn(b'PID,80,200,0,75\n',c.writes)
                    self.assertTrue(all(r['run_phase_seen']))
                else:
                    self.assertFalse(any(x.startswith((b'PID,',b'FF,',b'START,',b'VEL,')) for x in c.writes))

    def test_active_faults_stop_restore_no_retry(self):
        faults=('no_response','stale','missing_ack','wrong_ack','partial_write',
                'missing_phase','reset','event','power','power_timeout','power_stale')
        per_side=('no_response','plateau','negative','wrong_output','target','overspeed','bad_phase')
        for fault in (*faults,*(f'{name}_{side}' for name in per_side for side in (0,1))):
            with self.subTest(fault=fault):
                code,r,c=self.fake(fault=fault)
                self.assertEqual(code,2); self.assertIsNotNone(r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and c.closed,
                                str(r))
                first_stop=next(i for i,x in enumerate(c.writes)
                                if x==b'STOP\n' and any(y.startswith(b'VEL,') for y in c.writes[:i]))
                self.assertFalse(any(x.startswith(b'VEL,') for x in c.writes[first_stop:]))
                self.assertLess(r['motion_to_stop_s'],.7)

    def test_all_six_balanced_episodes_restore(self):
        for episode in (None,*m.EPISODES):
            with self.subTest(episode=episode):
                code,r,c=self.fake(episode,profile='balanced')
                self.assertEqual(code,0,r['reason'])
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and r['power_monitor_closed'])
                if episode:
                    self.assertIn(b'PID,200,800,0,85\n',c.writes)
                    self.assertIn(b'PID,80,200,0,75\n',c.writes)
                    self.assertTrue(all(r['run_phase_seen']))
                else:
                    self.assertFalse(any(x.startswith((b'PID,',b'FF,',b'START,',b'VEL,')) for x in c.writes))

    def test_power_monitor_unavailable_blocks_motion(self):
        code,r,c=self.fake(fault='power_warmup')
        self.assertEqual(code,2); self.assertFalse(r['motion_sent'])
        self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'] and r['power_monitor_closed'])
        self.assertFalse(any(x.startswith(b'VEL,') for x in c.writes))

    def test_both_run_required(self):
        for side in (0,1):
            with self.subTest(side=side):
                code,r,c=self.fake(fault=f'never_run_{side}')
                self.assertEqual(code,2); self.assertFalse(all(r['run_phase_seen']))
                self.assertTrue(r['stop_confirmed'] and r['restore_confirmed'])

    def test_missing_stop_or_restore_not_success(self):
        for fault in ('missing_stop','restore'):
            with self.subTest(fault=fault):
                code,r,c=self.fake(fault=fault)
                self.assertEqual(code,2); self.assertTrue(c.closed)
                self.assertFalse(r['restore_confirmed'])

    def test_unknown_options_before_hardware(self):
        for args in (['--episode','both-forward-10s'],['--pwm','204'],['--seconds','5'],
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
        g=m.Guard([2**32-25,2**31-1,2**31-1,0,0,0,0,0,0],0,1)
        g.observe([25,-2**31,-2**31,128,128,200,200,100,100],.05)
        self.assertTrue(all(g.responded))

    def test_travel_and_pair_separation_guard(self):
        g=m.Guard([0,0,0,0,0,0,0,0,0],0,1)
        with self.assertRaisesRegex(RuntimeError,'travel budget'):
            for index in range(1,35):
                g.observe([index*50,index*40,index*40,128,128,200,200,800,800],index*.05)
        g=m.Guard([0,0,0,0,0,0,0,0,0],0,1)
        with self.assertRaisesRegex(RuntimeError,'separation'):
            for index in range(1,20):
                g.observe([index*50,index*25,index*10,128,128,200,200,500,200],index*.05)

if __name__ == '__main__': unittest.main()
