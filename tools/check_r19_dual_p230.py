#!/usr/bin/env python3
"""P230 (90% running ceiling) copy of the measured-FF V3 helper: one explicit bounded
dual-wheel straight-motion episode; default STOP/INFO/CAPS only.

Copy of check_r19_dual_handoff.py (V3 helper, unchanged) with one named
temporary profile: balanced PI 0.20/0.80/Kv0 plus FF from post-wire-fix
13 Oct holds (left fwd/rev, right fwd/rev) = 88/67/57/81, each set ~3 PWM
below the measured settled mean to avoid startup overshoot.
Only both-forward/reverse at +/-200ticks/s, nominal1s/3s/5s, plus 3 s holds at
+/-150 and +/-250 ticks/s (inside the firmware's 100-300 target window);
restore PID/FF and START,0 after every episode.
No arbitrary gains/target/duty/duration, repeat, sweep, ROS or upload.
Requires a supervised clear floor, reachable disconnect and operator cell checker.
No current/cell sensing, physical-direction, geometry or straight-line proof.
"""
import argparse
import json
import re
import signal
import subprocess
import time
from check_r19_lifted_wheel import PORT, command, counter_delta, resting
from r19_power_monitor import PowerMonitor, QUERY_TIMEOUT, POLL_INTERVAL, MAX_SAMPLE_AGE

INFO = 'INFO,2,748,230,2000'
CAPS = 'CAPS,BOUNDED_DUAL_V3_P230B3,128,153,230,300,250,1000,6000,1400,160,3,BK1'
MAX_BACKWARD_TICKS = 3  # 2026-10-14: 1-2 tick wobble tolerated, 3+ faults
EPISODES = {f'both-{direction}-{seconds}s': (sign, seconds, 200)
            for direction,sign in (('forward',1),('reverse',-1))
            for seconds in (1,3,5)}
EPISODES.update({f'both-{direction}-3s-{speed}': (sign, 3, speed)
                 for direction,sign in (('forward',1),('reverse',-1))
                 for speed in (150,250)})
MAX_TRAVEL_TICKS = 1300
MAX_PAIR_SEPARATION_TICKS = 160
STALE = .15
NO_RESPONSE = .30
NO_PROGRESS = .25
REFRESH = .08
STOP_MARGIN = .03
MAX_SPEED = 1000
CEILING = 230  # 2026-10-14: 90% running ceiling; startup boost stays <=153
PROFILES = {'balanced_mff':(200,800,0,85)}
# FF,<left_fwd>,<left_rev>,<right_fwd>,<right_rev>; PID resets all four to 85 first.
FEED_FORWARD = {'balanced_mff':(88,67,57,81)}

def parse_t(line):
    if not line.startswith('T,'): return None
    parts = line.split(',')
    if len(parts) !=10: raise ValueError('Wrong telemetry shape')
    row = [int(x) for x in parts[1:]]
    if not 0 <= row[0] < 2**32: raise ValueError('Invalid clock')
    if any(not -2**31 <= x < 2**31 for x in row[1:3]): raise ValueError('Invalid counters')
    if any(abs(x) > CEILING for x in row[3:5]): raise ValueError('PWM outside ceiling')
    return row

def parse_b(line):
    if not line.startswith('B,'): return None
    parts = line.split(',')
    if len(parts) !=5: raise ValueError('Wrong phase shape')
    row = [int(x) for x in parts[1:]]
    if not 0 <= row[0] < 2**32 or any(x not in (0,1,2) for x in row[1:3]) or row[3] not in (0,1):
        raise ValueError('Invalid phase diagnostic')
    return row

class Guard:
    def __init__(self, baseline, started, sign, speed=200):
        self.previous = self.baseline = baseline
        self.started = self.last_received = self.last_ack = self.last_phase = started
        self.last_progress = [started,started]
        self.sign = sign
        self.targets = [sign*speed,sign*speed]
        self.responded = [False,False]
        self.run_seen = [False,False]

    def ack(self, now): self.last_ack = now

    def phase(self, row, now):
        if row[3] or (any(x not in (1,2) for x in row[1:3]) and now-self.started > .10):
            raise RuntimeError('Unexpected startup phase/fault')
        for side in (0,1):
            if row[1+side] ==2: self.run_seen[side] = True
        self.last_phase = now

    def observe(self, row, now):
        dt = counter_delta(row[0], self.previous[0])
        if dt <=0 or dt >250: raise RuntimeError('Controller clock discontinuity')
        grace = now-self.started <=.10 and row[3:7] ==[0,0,0,0]
        if row[5:7] !=self.targets and not grace: raise RuntimeError('Unexpected velocity target')
        totals = []
        for side in (0,1):
            if not 0 <= self.sign*row[3+side] <=CEILING:
                raise RuntimeError('Unexpected motor output')
            step = self.sign*counter_delta(row[1+side],self.previous[1+side])
            total = self.sign*counter_delta(row[1+side],self.baseline[1+side])
            if step <=-MAX_BACKWARD_TICKS or total <=-MAX_BACKWARD_TICKS:
                raise RuntimeError('Encoder direction disagrees with request')
            if step*1000/dt >MAX_SPEED or abs(row[7+side]) >MAX_SPEED:
                raise RuntimeError('Feedback speed exceeded bound')
            if step >0:
                self.last_progress[side] = now; self.responded[side] = True
            total = self.sign*counter_delta(row[1+side],self.baseline[1+side])
            if total >MAX_TRAVEL_TICKS: raise RuntimeError('Per-wheel encoder travel budget exceeded')
            totals.append(total)
        if abs(totals[0]-totals[1]) >MAX_PAIR_SEPARATION_TICKS:
            raise RuntimeError('Paired encoder separation exceeded bound')
        self.previous, self.last_received = row, now

    def check(self, now):
        if now-self.last_received >STALE: raise RuntimeError('Stale telemetry')
        if now-self.last_ack >STALE: raise RuntimeError('Stale VEL acknowledgement')
        if now-self.last_phase >STALE: raise RuntimeError('Stale startup phase')
        for side in (0,1):
            if now-self.started >=NO_RESPONSE and not self.responded[side]:
                raise RuntimeError('No encoder response: '+('left' if side==0 else 'right'))
            if self.responded[side] and now-self.last_progress[side] >NO_PROGRESS:
                raise RuntimeError('Encoder progress stopped: '+('left' if side==0 else 'right'))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', choices=tuple(EPISODES))
    parser.add_argument('--profile',choices=tuple(PROFILES),default='balanced_mff')
    args = parser.parse_args()
    pid = PROFILES[args.profile]
    pid_command = 'PID,'+','.join(str(x) for x in pid)
    ff = FEED_FORWARD[args.profile]
    ff_command = 'FF,'+','.join(str(x) for x in ff)
    armed = args.episode is not None
    sign, seconds, speed = EPISODES[args.episode] if armed else (None,0,0)
    targets = [0,0]
    if armed: targets = [sign*speed,sign*speed]
    motion = ('VEL,'+','.join(str(x) for x in targets)).encode()+b'\n'
    motion_ack = 'OK,'+motion.decode().strip()
    result = {
        'episode':args.episode, 'port':PORT, 'requested_setup':'supervised floor',
        'targets':targets, 'nominal_seconds':seconds, 'pwm_ceiling':CEILING,
        'controller_info':None, 'caps':None, 'motion_sent':False,
        'commands':[], 'telemetry':[], 'phases':[], 'power_samples':[], 'events':[],
        'stop_confirmed':False, 'restore_confirmed':not armed, 'serial_closed':False,
        'run_phase_seen':[False,False], 'reason':None, 'operator_observation':None,
        'power_monitor_closed':not armed,
        'profile':{'name':args.profile, 'pid':list(pid), 'ff':list(ff), 'startup':'opt-in'},
        'restore':{'pid':[80,200,0,75], 'ff':[75,60,75,60], 'startup':0},
        'limits':{'ack_telemetry_phase_s':STALE,'response_s':NO_RESPONSE,
                  'progress_s':NO_PROGRESS,'feedback_ticks_s':MAX_SPEED,
                  'refresh_s':REFRESH,'stop_margin_s':STOP_MARGIN,
                  'per_wheel_travel_ticks':MAX_TRAVEL_TICKS,
                  'pair_separation_ticks':MAX_PAIR_SEPARATION_TICKS,
                  'power_query_s':QUERY_TIMEOUT,'power_sample_age_s':MAX_SAMPLE_AGE,
                  'power_poll_s':POLL_INTERVAL},
        'unmeasured':['physical direction','motor VM/current','individual cells'],
        'no_automatic_retry':True,
    }
    connection = None
    rows, lines = [], []
    started = time.monotonic()
    initial_flags = None
    guard = None
    power_monitor = None

    def check_flags(flags, began, completed=None):
        sample = {'elapsed_s':began-started,'flags':hex(flags)}
        if completed is not None: sample['query_s'] =completed-began
        result['power_samples'].append(sample)
        if flags &0x5 or (initial_flags is not None and (flags &~initial_flags)&0x50000):
            raise RuntimeError('Pi power fault')
        return flags

    def power(timeout=.2):
        began =time.monotonic()
        raw =command(['vcgencmd','get_throttled'],timeout=timeout)
        if not re.fullmatch(r'throttled=0x[0-9a-fA-F]+',raw): raise RuntimeError('Invalid power flags')
        return check_flags(int(raw.split('=')[1],16),began,time.monotonic())

    def monitored_power():
        for sample in power_monitor.check():
            check_flags(sample['flags'],sample['began'],sample['completed'])

    def send(data):
        if connection.write(data) !=len(data): raise RuntimeError('Partial serial write')
        result['commands'].append({'elapsed_s':time.monotonic()-started,
                                   'command':data.decode().strip()})

    def read_one(phase, active_guard=None):
        raw = connection.readline(128)
        if not raw: return
        if not raw.endswith(b'\n'): raise RuntimeError('Partial/overlong serial line')
        line = raw.decode('ascii').strip()
        lines.append(line)
        now = time.monotonic()
        if line.startswith(('ERR,','EVENT,')):
            result['events'].append({'elapsed_s':now-started,'phase':phase,'line':line})
            if phase not in ('after_stop','cleanup'): raise RuntimeError(line)
        if line.startswith('INFO,'):
            if line !=INFO: raise RuntimeError('Unexpected controller INFO')
            result['controller_info'] =line
        if line.startswith('CAPS,'):
            if line !=CAPS: raise RuntimeError('Unsupported candidate capability')
            result['caps'] =line
        if line.startswith('OK,VEL,'):
            if not armed or line !=motion_ack: raise RuntimeError('Unexpected VEL acknowledgement')
            if active_guard: active_guard.ack(now)
        if line.startswith('OK,PWM,'): raise RuntimeError('Unexpected direct PWM')
        row = parse_t(line)
        if row is not None:
            if rows and counter_delta(row[0],rows[-1][0]) <=0:
                raise RuntimeError('Controller clock restarted/repeated')
            rows.append(row)
            result['telemetry'].append({'elapsed_s':now-started,'phase':phase,'values':row})
            if active_guard: active_guard.observe(row,now)
        b = parse_b(line)
        if b is not None:
            result['phases'].append({'elapsed_s':now-started,'phase':phase,'values':b})
            if active_guard: active_guard.phase(b,now)

    def collect(duration,phase):
        deadline =time.monotonic()+duration
        while time.monotonic() <deadline: read_one(phase)

    def exchange(data,ack):
        mark =len(lines); send(data)
        deadline =time.monotonic()+.25
        while time.monotonic() <deadline:
            read_one('cleanup' if data in (b'START,0\n',b'PID,80,200,0,75\n',b'FF,75,60,75,60\n') else 'configure')
            if ack in lines[mark:]: return
        raise RuntimeError('Missing configuration/restore ACK: '+ack)

    def interrupted(signum,frame): raise RuntimeError('Interrupted; STOP required')
    for sig in (signal.SIGINT,signal.SIGTERM): signal.signal(sig,interrupted)
    try:
        identity =command(['udevadm','info','--query=property','--name='+PORT])
        if 'ID_VENDOR_ID=1a86\n' not in identity+'\n' or 'ID_MODEL_ID=7523\n' not in identity+'\n':
            raise RuntimeError('Unexpected Nano identity')
        owner =subprocess.run(['fuser',PORT],capture_output=True,text=True,timeout=2)
        if owner.returncode !=1 or owner.stdout.strip() or owner.stderr.strip():
            raise RuntimeError('Port busy/owner check failed')
        initial_flags =power(.2)
        result['initial_adc'] =command(['vcgencmd','pmic_read_adc','EXT5V_V'])
        import serial
        connection =serial.Serial(PORT,115200,timeout=.01,write_timeout=.05,exclusive=True)
        time.sleep(2); connection.reset_input_buffer()
        send(b'STOP\n'); send(b'INFO\n'); send(b'CAPS\n'); collect(.7,'preflight')
        if result['controller_info'] !=INFO or result['caps'] !=CAPS or 'OK,STOP' not in lines or not resting(rows):
            raise RuntimeError('Stopped candidate identity not confirmed')
        if armed:
            exchange((pid_command+'\n').encode(),'OK,'+pid_command)
            exchange((ff_command+'\n').encode(),'OK,'+ff_command)
            exchange(b'START,1\n','OK,START,1')
            collect(.15,'configure')
            if not resting(rows): raise RuntimeError('Configuration energized/moved motor')
        power(.2)
        if armed:
            power_monitor =PowerMonitor(command,clock=time.monotonic)
            power_monitor.start()
            collect(.15,'configure') # Warm the monitor only while stopped; drain serial too.
            monitored_power()
            if not resting(rows): raise RuntimeError('Power-monitor warmup moved motor')
        baseline =rows[-1]
        if armed:
            began =time.monotonic(); guard =Guard(baseline,began,sign,speed)
            deadline =began+seconds-STOP_MARGIN
            next_refresh =began
            try:
                while time.monotonic() <deadline:
                    now =time.monotonic(); guard.check(now); monitored_power()
                    if now >=next_refresh:
                        send(motion); result['motion_sent'] =True
                        next_refresh =time.monotonic()+REFRESH
                    if time.monotonic() >=deadline: break
                    read_one('velocity',guard)
                if not all(guard.run_seen): raise RuntimeError('Startup never confirmed RUN on both wheels')
            except Exception as error:
                result['reason'] =str(error)
            finally:
                send(b'STOP\n')
                result['motion_to_stop_s'] =time.monotonic()-began
                result['run_phase_seen'] =guard.run_seen
        else:
            send(b'STOP\n')
        mark =len(lines); collect(.9,'after_stop')
        result['stop_confirmed'] ='OK,STOP' in lines[mark:] and resting(rows)
        if not result['stop_confirmed']: raise RuntimeError('STOP/stationary state unconfirmed')
        result['encoder_delta'] =[counter_delta(rows[-1][i],baseline[i]) for i in (1,2)]
        if armed:
            exchange(b'START,0\n','OK,START,0')
            exchange(b'PID,80,200,0,75\n','OK,PID,80,200,0,75')
            exchange(b'FF,75,60,75,60\n','OK,FF,75,60,75,60')
            result['restore_confirmed'] =True
        if power_monitor is not None:
            result['power_monitor_closed'] =power_monitor.close()
            if not result['power_monitor_closed']: raise RuntimeError('Power monitor did not stop')
        power(.2)
        result['final_adc'] =command(['vcgencmd','pmic_read_adc','EXT5V_V'])
        return 0 if result['reason'] is None else 2
    except Exception as error:
        result['reason'] =result['reason'] or str(error)
        return 2
    finally:
        if connection is not None:
            try:
                send(b'STOP\n')
                if armed and not result['restore_confirmed']:
                    mark =len(lines); collect(.9,'after_stop')
                    stopped ='OK,STOP' in lines[mark:] and resting(rows)
                    result['stop_confirmed'] =result['stop_confirmed'] or stopped
                    if stopped:
                        exchange(b'START,0\n','OK,START,0')
                        exchange(b'PID,80,200,0,75\n','OK,PID,80,200,0,75')
                        exchange(b'FF,75,60,75,60\n','OK,FF,75,60,75,60')
                        result['restore_confirmed'] =True
            except Exception as error: result['cleanup_error'] =str(error)
            connection.close(); result['serial_closed'] =True
        if power_monitor is not None and not result['power_monitor_closed']:
            result['power_monitor_closed'] =power_monitor.close()
        if power_monitor is None: result['power_monitor_closed'] =True
        result['elapsed_s'] =time.monotonic()-started
        print(json.dumps(result,indent=2))

if __name__ == '__main__': raise SystemExit(main())
