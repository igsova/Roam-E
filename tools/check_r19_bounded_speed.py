#!/usr/bin/env python3
"""One opt-in bounded-start floor episode; default STOP/INFO/CAPS only.

Only eight explicit episodes: one wheel/direction, VEL +/-200, 1s smoke or 3s
tracking. No arbitrary gains/target/duty/duration, repeats, sweep, ROS or upload.
Named temporary baseline (0.16/0.2/0) or responsive (0.24/0.6/0) PI profile,
both FF85; restore image baseline and START,0.
Requires supervised clear floor, reachable disconnect and operator cell checker.
Software does not measure current/cells, qualify geometry or verify physical direction.
"""
import argparse
import json
import re
import signal
import subprocess
import time
from check_r19_lifted_wheel import PORT, command, counter_delta, resting

INFO = 'INFO,2,748,153,2000'
CAPS = 'CAPS,BOUNDED_START_V1,128,153,300,250,1000,3000'
EPISODES = {f'{side}-{direction}-{seconds}s': (index, sign, seconds)
            for side,index in (('left',0),('right',1))
            for direction,sign in (('forward',1),('reverse',-1))
            for seconds in (1,3)}
STALE = .15
NO_RESPONSE = .30
NO_PROGRESS = .25
REFRESH = .08
STOP_MARGIN = .03
MAX_SPEED = 1000
CEILING = 153
PROFILES = {'baseline':(160,200,0,85), 'responsive':(240,600,0,85)}

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
    def __init__(self, baseline, started, side, sign):
        self.previous = baseline
        self.started = self.last_received = self.last_ack = self.last_phase = self.last_progress = started
        self.side, self.sign = side, sign
        self.targets = [0,0]; self.targets[side] = sign*200
        self.responded = self.run_seen = False

    def ack(self, now): self.last_ack = now

    def phase(self, row, now):
        selected, other = row[1+self.side], row[2-self.side]
        if row[3] or other !=0 or (selected not in (1,2) and now-self.started > .10):
            raise RuntimeError('Unexpected startup phase/fault')
        if selected ==2: self.run_seen = True
        self.last_phase = now

    def observe(self, row, now):
        dt = counter_delta(row[0], self.previous[0])
        if dt <=0 or dt >250: raise RuntimeError('Controller clock discontinuity')
        side, other = self.side, 1-self.side
        if row[3+other] !=0 or not 0 <= self.sign*row[3+side] <=CEILING:
            raise RuntimeError('Unexpected motor output')
        grace = now-self.started <=.10 and row[3:7] ==[0,0,0,0]
        if row[5:7] !=self.targets and not grace: raise RuntimeError('Unexpected velocity target')
        step = self.sign*counter_delta(row[1+side], self.previous[1+side])
        if step <0: raise RuntimeError('Encoder direction disagrees with request')
        if step*1000/dt >MAX_SPEED or abs(row[7+side]) >MAX_SPEED:
            raise RuntimeError('Feedback speed exceeded bound')
        if step >0: self.last_progress = now; self.responded = True
        self.previous, self.last_received = row, now

    def check(self, now):
        if now-self.last_received >STALE: raise RuntimeError('Stale telemetry')
        if now-self.last_ack >STALE: raise RuntimeError('Stale VEL acknowledgement')
        if now-self.last_phase >STALE: raise RuntimeError('Stale startup phase')
        if now-self.started >=NO_RESPONSE and not self.responded:
            raise RuntimeError('No encoder response')
        if self.responded and now-self.last_progress >NO_PROGRESS:
            raise RuntimeError('Encoder progress stopped')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', choices=tuple(EPISODES))
    parser.add_argument('--profile', choices=tuple(PROFILES), default='baseline')
    args = parser.parse_args()
    pid = PROFILES[args.profile]
    pid_command = 'PID,'+','.join(str(x) for x in pid)
    armed = args.episode is not None
    side, sign, seconds = EPISODES[args.episode] if armed else (None,None,0)
    targets = [0,0]
    if armed: targets[side] =sign*200
    motion = ('VEL,'+','.join(str(x) for x in targets)).encode()+b'\n'
    motion_ack = 'OK,'+motion.decode().strip()
    result = {
        'episode':args.episode, 'port':PORT, 'requested_setup':'supervised floor',
        'targets':targets, 'nominal_seconds':seconds, 'pwm_ceiling':CEILING,
        'controller_info':None, 'caps':None, 'motion_sent':False,
        'commands':[], 'telemetry':[], 'phases':[], 'power_samples':[], 'events':[],
        'stop_confirmed':False, 'restore_confirmed':not armed, 'serial_closed':False,
        'run_phase_seen':False, 'reason':None, 'operator_observation':None,
        'profile':{'name':args.profile, 'pid':list(pid), 'ff':[85,85,85,85], 'startup':'opt-in'},
        'restore':{'pid':[80,200,0,75], 'ff':[75,60,75,60], 'startup':0},
        'limits':{'ack_telemetry_phase_s':STALE,'response_s':NO_RESPONSE,
                  'progress_s':NO_PROGRESS,'feedback_ticks_s':MAX_SPEED,
                  'refresh_s':REFRESH,'stop_margin_s':STOP_MARGIN},
        'unmeasured':['physical direction','motor VM/current','individual cells'],
        'no_automatic_retry':True,
    }
    connection = None
    rows, lines = [], []
    started = time.monotonic()
    initial_flags = None
    guard = None

    def power(timeout=.05):
        raw = command(['vcgencmd','get_throttled'],timeout=timeout)
        if not re.fullmatch(r'throttled=0x[0-9a-fA-F]+',raw): raise RuntimeError('Invalid power flags')
        flags = int(raw.split('=')[1],16)
        result['power_samples'].append({'elapsed_s':time.monotonic()-started,'flags':hex(flags)})
        if flags &0x5 or (initial_flags is not None and (flags &~initial_flags)&0x50000):
            raise RuntimeError('Pi power fault')
        return flags

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
            exchange(b'FF,85,85,85,85\n','OK,FF,85,85,85,85')
            exchange(b'START,1\n','OK,START,1')
            collect(.15,'configure')
            if not resting(rows): raise RuntimeError('Configuration energized/moved motor')
        baseline =rows[-1]
        power(.2)
        if armed:
            began =time.monotonic(); guard =Guard(baseline,began,side,sign)
            deadline =began+seconds-STOP_MARGIN
            next_refresh, next_power =began,began+.15
            try:
                while time.monotonic() <deadline:
                    now =time.monotonic(); guard.check(now)
                    if now >=next_refresh:
                        send(motion); result['motion_sent'] =True
                        next_refresh =time.monotonic()+REFRESH
                    if now >=next_power and deadline-time.monotonic() >.06:
                        power(); next_power =time.monotonic()+.15
                    if time.monotonic() >=deadline: break
                    read_one('velocity',guard)
                if not guard.run_seen: raise RuntimeError('Startup never confirmed RUN')
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
        result['elapsed_s'] =time.monotonic()-started
        print(json.dumps(result,indent=2))

if __name__ == '__main__': raise SystemExit(main())
