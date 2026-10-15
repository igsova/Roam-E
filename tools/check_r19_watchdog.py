#!/usr/bin/env python3
"""V3 bounded watchdog probe; default STOP/INFO/CAPS, explicit --watchdog-forward only; default STOP/INFO/CAPS only.

Only both-forward/reverse at +/-200ticks/s, nominal1s/3s/5s. Named temporary
responsive0.24/0.60, damped0.32/0.40 or balanced0.20/0.80 PI, Kv0/FF85;
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

INFO = 'INFO,2,748,153,2000'
CAPS = 'CAPS,BOUNDED_DUAL_V3,128,153,300,250,1000,6000,1400,160'
EPISODES = {f'both-{direction}-{seconds}s': (sign, seconds)
            for direction,sign in (('forward',1),('reverse',-1))
            for seconds in (1,3,5)}
MAX_TRAVEL_TICKS = 1300
MAX_PAIR_SEPARATION_TICKS = 160
STALE = .15
NO_RESPONSE = .30
NO_PROGRESS = .25
REFRESH = .08
STOP_MARGIN = .03
MAX_SPEED = 1000
CEILING = 153
PROFILES = {'responsive':(240,600,0,85), 'damped':(320,400,0,85),
            'balanced':(200,800,0,85)}

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
    def __init__(self, baseline, started, sign):
        self.previous = self.baseline = baseline
        self.started = self.last_received = self.last_ack = self.last_phase = started
        self.last_progress = [started,started]
        self.sign = sign
        self.targets = [sign*200,sign*200]
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
            if step <0: raise RuntimeError('Encoder direction disagrees with request')
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

class WatchdogGuard(Guard):
    """One-command lease-expiry probe, not a normal streaming motion mode."""
    def __init__(self, baseline, started, sign):
        super().__init__(baseline, started, sign)
        self.ack_seen = False
        self.watchdog_seen = False
        self.event_after_s = None
        self.zero_samples = 0

    def ack(self, now):
        super().ack(now)
        self.ack_seen = True

    def event(self, now):
        elapsed = now-self.started
        if not .30 <= elapsed <= .50:
            raise RuntimeError('Watchdog event outside bounded expiry window')
        if not self.ack_seen or not all(self.run_seen):
            raise RuntimeError('Watchdog without initial ACK/independent RUN')
        self.watchdog_seen = True
        self.event_after_s = elapsed

    def phase(self, row, now):
        if self.watchdog_seen:
            if row[1:] !=[0,0,0]:
                raise RuntimeError('Non-idle/fault phase after watchdog')
            self.last_phase = now
        else:
            super().phase(row, now)

    def observe(self, row, now):
        if not self.watchdog_seen:
            return super().observe(row, now)
        dt = counter_delta(row[0], self.previous[0])
        if dt <=0 or dt >250:
            raise RuntimeError('Clock discontinuity after watchdog')
        if row[3:] !=[0,0,0,0,0,0]:
            raise RuntimeError('Watchdog did not clear both outputs/targets/feedback')
        totals = []
        for side in (0,1):
            step = self.sign*counter_delta(row[1+side], self.previous[1+side])
            total = self.sign*counter_delta(row[1+side], self.baseline[1+side])
            if step <0 or step*1000/dt >MAX_SPEED or total >MAX_TRAVEL_TICKS:
                raise RuntimeError('Invalid travel after watchdog')
            totals.append(total)
        if abs(totals[0]-totals[1]) >MAX_PAIR_SEPARATION_TICKS:
            raise RuntimeError('Pair separation after watchdog')
        self.zero_samples +=1
        self.previous, self.last_received = row, now

    def check(self, now):
        if now-self.last_received >STALE or now-self.last_phase >STALE:
            raise RuntimeError('Stale watchdog telemetry/phase')
        # This probe intentionally sends only one command, so subsequent ACK
        # freshness is not required. Missing INITIAL ACK still aborts at150ms.
        if not self.ack_seen and now-self.started >STALE:
            raise RuntimeError('Missing initial watchdog VEL acknowledgement')
        if not self.watchdog_seen:
            if now-self.started >.50:
                raise RuntimeError('Missing watchdog event; host STOP required')
            for side in (0,1):
                if now-self.started >=NO_RESPONSE and not self.responded[side]:
                    raise RuntimeError('No watchdog encoder response')
                if self.responded[side] and now-self.last_progress[side] >NO_PROGRESS:
                    raise RuntimeError('Watchdog encoder progress stopped')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group =parser.add_mutually_exclusive_group()
    group.add_argument('--episode', choices=tuple(EPISODES))
    group.add_argument('--watchdog-forward',action='store_true')
    parser.add_argument('--profile',choices=tuple(PROFILES),default='responsive')
    args = parser.parse_args()
    pid = PROFILES[args.profile]
    pid_command = 'PID,'+','.join(str(x) for x in pid)
    watchdog =args.watchdog_forward
    armed = args.episode is not None or watchdog
    sign, seconds = (1,.60) if watchdog else (EPISODES[args.episode] if armed else (None,0))
    targets = [0,0]
    if armed: targets = [sign*200,sign*200]
    motion = ('VEL,'+','.join(str(x) for x in targets)).encode()+b'\n'
    motion_ack = 'OK,'+motion.decode().strip()
    result = {
        'episode':'watchdog-forward' if watchdog else args.episode, 'port':PORT, 'requested_setup':'supervised floor',
        'targets':targets, 'nominal_seconds':seconds, 'pwm_ceiling':CEILING,
        'controller_info':None, 'caps':None, 'motion_sent':False,
        'commands':[], 'telemetry':[], 'phases':[], 'power_samples':[], 'events':[],
        'stop_confirmed':False, 'restore_confirmed':not armed, 'serial_closed':False,
        'run_phase_seen':[False,False], 'reason':None, 'operator_observation':None,
        'power_monitor_closed':not armed,
        'profile':{'name':args.profile, 'pid':list(pid), 'ff':[85,85,85,85], 'startup':'opt-in'},
        'restore':{'pid':[80,200,0,75], 'ff':[75,60,75,60], 'startup':0},
        'limits':{'ack_telemetry_phase_s':STALE,'response_s':NO_RESPONSE,
                  'progress_s':NO_PROGRESS,'feedback_ticks_s':MAX_SPEED,
                  'refresh_s':REFRESH,'stop_margin_s':STOP_MARGIN,
                  'per_wheel_travel_ticks':MAX_TRAVEL_TICKS,
                  'pair_separation_ticks':MAX_PAIR_SEPARATION_TICKS,
                  'power_query_s':QUERY_TIMEOUT,'power_sample_age_s':MAX_SAMPLE_AGE,
                  'power_poll_s':POLL_INTERVAL},
        'unmeasured':['physical direction','motor VM/current','individual cells'],
        'no_automatic_retry':True, 'watchdog_probe':watchdog,
        'watchdog_event_after_s':None, 'watchdog_zero_samples':0,
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
            if line =='EVENT,WATCHDOG_STOP' and watchdog and phase =='velocity' and active_guard:
                active_guard.event(now)
            elif phase not in ('after_stop','cleanup'): raise RuntimeError(line)
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
            exchange(b'FF,85,85,85,85\n','OK,FF,85,85,85,85')
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
            began =time.monotonic(); guard =(WatchdogGuard if watchdog else Guard)(baseline,began,sign)
            deadline =began+seconds-STOP_MARGIN
            next_refresh =began
            try:
                while time.monotonic() <deadline:
                    now =time.monotonic(); guard.check(now); monitored_power()
                    if (not result['motion_sent'] if watchdog else now >=next_refresh):
                        send(motion); result['motion_sent'] =True
                        next_refresh =time.monotonic()+REFRESH
                    if time.monotonic() >=deadline: break
                    read_one('velocity',guard)
                if not all(guard.run_seen): raise RuntimeError('Startup never confirmed RUN on both wheels')
                if watchdog:
                    result['watchdog_event_after_s'] =guard.event_after_s
                    result['watchdog_zero_samples'] =guard.zero_samples
                    if not guard.watchdog_seen or guard.zero_samples <2:
                        raise RuntimeError('Watchdog stop not independently confirmed')
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
