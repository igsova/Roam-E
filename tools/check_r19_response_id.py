#!/usr/bin/env python3
"""ID_V1 response identification: one predeclared bounded paired episode; default STOP/INFO/CAPS only.

Episodes (names only; no arbitrary duty, gains, timing or repeat):
  duty-forward / duty-reverse  open-loop fixed duty after the unchanged bounded
                               startup proof; one +15 step at 1.5 s; 3 s total.
  vel-forward-t1fast / vel-reverse-t1fast
                               existing V3 balanced 200 ticks/s PI hold for 3 s
                               with timer1 switched to ~977 Hz (PWMF,1).
  vel-forward-default / vel-reverse-default
                               the same hold with the core timer1 (PWMF,0), for a
                               same-floor A/B against the t1fast holds.
Firmware keeps PWM153, 350 ms watchdog, 6 s/1400-tick/160-separation and
startup/progress/speed guards. Host adds 1300-tick travel and 150 ms freshness.
Cleanup: STOP, PWMF,0, START,0 (also restores timer1), original PID/FF, close.
Requires a supervised clear floor. No current/cell sensing or geometry proof.
"""
import argparse
import json
import re
import signal
import subprocess
import time
from check_r19_lifted_wheel import PORT, command, counter_delta, resting
from check_r19_dual_handoff import parse_t, parse_b
from r19_power_monitor import PowerMonitor, QUERY_TIMEOUT, POLL_INTERVAL, MAX_SAMPLE_AGE

INFO = 'INFO,2,748,153,2000'
CAPS = 'CAPS,BOUNDED_ID_V1,128,153,300,250,1000,6000,1400,160,40,120'
SECONDS = 3.0
STEP_AT = 1.5
# Base duties from the saved V3 settled means (R19_RESPONSE_ANALYSIS_20261013):
# forward L≈85/R≈84, reverse L≈65/R≈87 PWM at 200 ticks/s, raised a few PWM
# to reduce stall risk at load peaks. Step +15 for incremental gain/delay.
EPISODES = {
    'duty-forward': {'sign': 1, 'mode': 'duty', 'timer1_fast': False,
                     'schedule': [(0.0, 92, 92), (STEP_AT, 107, 107)]},
    'duty-reverse': {'sign': -1, 'mode': 'duty', 'timer1_fast': False,
                     'schedule': [(0.0, -72, -94), (STEP_AT, -87, -109)]},
    'vel-forward-t1fast': {'sign': 1, 'mode': 'vel', 'timer1_fast': True},
    'vel-reverse-t1fast': {'sign': -1, 'mode': 'vel', 'timer1_fast': True},
    'vel-forward-default': {'sign': 1, 'mode': 'vel', 'timer1_fast': False},
    'vel-reverse-default': {'sign': -1, 'mode': 'vel', 'timer1_fast': False},
}
BALANCED = (200, 800, 0, 85)
MAX_TRAVEL_TICKS = 1300
MAX_PAIR_SEPARATION_TICKS = 160
STALE = .15
NO_RESPONSE = .30
NO_PROGRESS = .25
REFRESH = .08
STOP_MARGIN = .03
MAX_SPEED = 1000
CEILING = 153
RESTORE = (b'PWMF,0\n', b'START,0\n', b'PID,80,200,0,75\n', b'FF,75,60,75,60\n')


def parse_i(line):
    if not line.startswith('I,'): return None
    parts = line.split(',')
    if len(parts) != 8: raise ValueError('Wrong ID step shape')
    row = [int(x) for x in parts[1:]]
    if not 0 <= row[0] < 2**32 or any(not 0 <= x < 2**32 for x in row[3:5]):
        raise ValueError('Invalid ID clock')
    if any(abs(x) > CEILING for x in row[5:7]): raise ValueError('ID PWM outside ceiling')
    return row


def duty_at(schedule, elapsed):
    current = schedule[0]
    for entry in schedule:
        if elapsed >= entry[0]: current = entry
    return current[1:]


class Guard:
    def __init__(self, baseline, started, sign, schedule=None):
        self.previous = self.baseline = baseline
        self.started = self.last_received = self.last_ack = self.last_phase = started
        self.last_progress = [started, started]
        self.sign = sign
        self.targets = [sign * 200, sign * 200]
        self.schedule = schedule
        self.allowed = set()
        self.last_duties = None
        self.responded = [False, False]
        self.run_seen = [False, False]
        self.phases = [0, 0]

    def ack(self, now): self.last_ack = now

    def commanded(self, duties):
        # A snapshot taken before the next 20 ms control step may still show
        # the previous level; anything else is a mismatch.
        duties = tuple(duties)
        self.allowed = {duties} | ({self.last_duties} if self.last_duties else set())
        self.last_duties = duties

    def phase(self, row, now):
        if row[3] or (any(x not in (1, 2) for x in row[1:3]) and now - self.started > .10):
            raise RuntimeError('Unexpected startup phase/fault')
        for side in (0, 1):
            if row[1 + side] == 2: self.run_seen[side] = True
        self.phases = row[1:3]
        self.last_phase = now

    def observe(self, row, now):
        dt = counter_delta(row[0], self.previous[0])
        if dt <= 0 or dt > 250: raise RuntimeError('Controller clock discontinuity')
        grace = now - self.started <= .10 and row[3:7] == [0, 0, 0, 0]
        if row[5:7] != self.targets and not grace: raise RuntimeError('Unexpected proof/velocity target')
        if self.schedule is not None and not grace and all(p == 2 for p in self.phases):
            if tuple(row[3:5]) not in self.allowed and tuple(row[3:5]) != (0, 0):
                raise RuntimeError('Output differs from declared duty')
        totals = []
        for side in (0, 1):
            if not 0 <= self.sign * row[3 + side] <= CEILING:
                raise RuntimeError('Unexpected motor output')
            step = self.sign * counter_delta(row[1 + side], self.previous[1 + side])
            if step < 0: raise RuntimeError('Encoder direction disagrees with request')
            if step * 1000 / dt > MAX_SPEED or abs(row[7 + side]) > MAX_SPEED:
                raise RuntimeError('Feedback speed exceeded bound')
            if step > 0:
                self.last_progress[side] = now; self.responded[side] = True
            total = self.sign * counter_delta(row[1 + side], self.baseline[1 + side])
            if total > MAX_TRAVEL_TICKS: raise RuntimeError('Per-wheel encoder travel budget exceeded')
            totals.append(total)
        if abs(totals[0] - totals[1]) > MAX_PAIR_SEPARATION_TICKS:
            raise RuntimeError('Paired encoder separation exceeded bound')
        self.previous, self.last_received = row, now

    def check(self, now):
        if now - self.last_received > STALE: raise RuntimeError('Stale telemetry')
        if now - self.last_ack > STALE: raise RuntimeError('Stale motion acknowledgement')
        if now - self.last_phase > STALE: raise RuntimeError('Stale startup phase')
        for side in (0, 1):
            if now - self.started >= NO_RESPONSE and not self.responded[side]:
                raise RuntimeError('No encoder response: ' + ('left' if side == 0 else 'right'))
            if self.responded[side] and now - self.last_progress[side] > NO_PROGRESS:
                raise RuntimeError('Encoder progress stopped: ' + ('left' if side == 0 else 'right'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', choices=tuple(EPISODES))
    args = parser.parse_args()
    armed = args.episode is not None
    spec = EPISODES[args.episode] if armed else None
    sign = spec['sign'] if armed else None
    schedule = spec.get('schedule') if armed else None
    result = {
        'episode': args.episode, 'port': PORT, 'requested_setup': 'supervised floor',
        'mode': spec['mode'] if armed else None,
        'timer1_fast': spec['timer1_fast'] if armed else False,
        'schedule': [list(x) for x in schedule] if schedule else None,
        'targets': [sign * 200, sign * 200] if armed else [0, 0],
        'nominal_seconds': SECONDS if armed else 0, 'pwm_ceiling': CEILING,
        'controller_info': None, 'caps': None, 'motion_sent': False,
        'commands': [], 'telemetry': [], 'phases': [], 'steps': [], 'power_samples': [],
        'events': [], 'stop_confirmed': False, 'restore_confirmed': not armed,
        'serial_closed': False, 'run_phase_seen': [False, False], 'reason': None,
        'operator_observation': None, 'power_monitor_closed': not armed,
        'profile': ({'name': 'balanced', 'pid': list(BALANCED), 'ff': [85, 85, 85, 85]}
                    if armed and spec['mode'] == 'vel' else None),
        'restore': {'pid': [80, 200, 0, 75], 'ff': [75, 60, 75, 60], 'startup': 0, 'timer1_fast': 0},
        'limits': {'ack_telemetry_phase_s': STALE, 'response_s': NO_RESPONSE,
                   'progress_s': NO_PROGRESS, 'feedback_ticks_s': MAX_SPEED,
                   'refresh_s': REFRESH, 'stop_margin_s': STOP_MARGIN,
                   'per_wheel_travel_ticks': MAX_TRAVEL_TICKS,
                   'pair_separation_ticks': MAX_PAIR_SEPARATION_TICKS,
                   'power_query_s': QUERY_TIMEOUT, 'power_sample_age_s': MAX_SAMPLE_AGE,
                   'power_poll_s': POLL_INTERVAL},
        'unmeasured': ['physical direction', 'motor VM/current', 'individual cells'],
        'no_automatic_retry': True,
    }
    connection = None
    rows, lines = [], []
    started = time.monotonic()
    initial_flags = None
    guard = None
    power_monitor = None
    expected_acks = []  # Current and previous motion command only.

    def check_flags(flags, began, completed=None):
        sample = {'elapsed_s': began - started, 'flags': hex(flags)}
        if completed is not None: sample['query_s'] = completed - began
        result['power_samples'].append(sample)
        if flags & 0x5 or (initial_flags is not None and (flags & ~initial_flags) & 0x50000):
            raise RuntimeError('Pi power fault')
        return flags

    def power(timeout=.2):
        began = time.monotonic()
        raw = command(['vcgencmd', 'get_throttled'], timeout=timeout)
        if not re.fullmatch(r'throttled=0x[0-9a-fA-F]+', raw): raise RuntimeError('Invalid power flags')
        return check_flags(int(raw.split('=')[1], 16), began, time.monotonic())

    def monitored_power():
        for sample in power_monitor.check():
            check_flags(sample['flags'], sample['began'], sample['completed'])

    def send(data):
        if connection.write(data) != len(data): raise RuntimeError('Partial serial write')
        result['commands'].append({'elapsed_s': time.monotonic() - started,
                                   'command': data.decode().strip()})

    def read_one(phase, active_guard=None):
        raw = connection.readline(128)
        if not raw: return
        if not raw.endswith(b'\n'): raise RuntimeError('Partial/overlong serial line')
        line = raw.decode('ascii').strip()
        lines.append(line)
        now = time.monotonic()
        if line.startswith(('ERR,', 'EVENT,')):
            result['events'].append({'elapsed_s': now - started, 'phase': phase, 'line': line})
            if phase not in ('after_stop', 'cleanup'): raise RuntimeError(line)
        if line.startswith('INFO,'):
            if line != INFO: raise RuntimeError('Unexpected controller INFO')
            result['controller_info'] = line
        if line.startswith('CAPS,'):
            if line != CAPS: raise RuntimeError('Unsupported candidate capability')
            result['caps'] = line
        if line.startswith(('OK,VEL,', 'OK,DUTY,')):
            if not armed or line not in expected_acks[-2:]: raise RuntimeError('Unexpected motion acknowledgement')
            if active_guard: active_guard.ack(now)
        if line.startswith('OK,PWM,'): raise RuntimeError('Unexpected direct PWM')
        row = parse_t(line)
        if row is not None:
            if rows and counter_delta(row[0], rows[-1][0]) <= 0:
                raise RuntimeError('Controller clock restarted/repeated')
            rows.append(row)
            result['telemetry'].append({'elapsed_s': now - started, 'phase': phase, 'values': row})
            if active_guard: active_guard.observe(row, now)
        b = parse_b(line)
        if b is not None:
            result['phases'].append({'elapsed_s': now - started, 'phase': phase, 'values': b})
            if active_guard: active_guard.phase(b, now)
        i = parse_i(line)
        if i is not None:
            result['steps'].append({'elapsed_s': now - started, 'phase': phase, 'values': i})

    def collect(duration, phase):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline: read_one(phase)

    def exchange(data, ack):
        mark = len(lines); send(data)
        deadline = time.monotonic() + .25
        while time.monotonic() < deadline:
            read_one('cleanup' if data in RESTORE else 'configure')
            if ack in lines[mark:]: return
        raise RuntimeError('Missing configuration/restore ACK: ' + ack)

    def restore():
        exchange(b'PWMF,0\n', 'OK,PWMF,0')
        exchange(b'START,0\n', 'OK,START,0')
        exchange(b'PID,80,200,0,75\n', 'OK,PID,80,200,0,75')
        exchange(b'FF,75,60,75,60\n', 'OK,FF,75,60,75,60')
        result['restore_confirmed'] = True

    def interrupted(signum, frame): raise RuntimeError('Interrupted; STOP required')
    for sig in (signal.SIGINT, signal.SIGTERM): signal.signal(sig, interrupted)
    try:
        identity = command(['udevadm', 'info', '--query=property', '--name=' + PORT])
        if 'ID_VENDOR_ID=1a86\n' not in identity + '\n' or 'ID_MODEL_ID=7523\n' not in identity + '\n':
            raise RuntimeError('Unexpected Nano identity')
        owner = subprocess.run(['fuser', PORT], capture_output=True, text=True, timeout=2)
        if owner.returncode != 1 or owner.stdout.strip() or owner.stderr.strip():
            raise RuntimeError('Port busy/owner check failed')
        initial_flags = power(.2)
        result['initial_adc'] = command(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'])
        import serial
        connection = serial.Serial(PORT, 115200, timeout=.01, write_timeout=.05, exclusive=True)
        time.sleep(2); connection.reset_input_buffer()
        send(b'STOP\n'); send(b'INFO\n'); send(b'CAPS\n'); collect(.7, 'preflight')
        if result['controller_info'] != INFO or result['caps'] != CAPS or 'OK,STOP' not in lines or not resting(rows):
            raise RuntimeError('Stopped candidate identity not confirmed')
        if armed:
            if spec['mode'] == 'vel':
                pid = 'PID,' + ','.join(str(x) for x in BALANCED)
                exchange((pid + '\n').encode(), 'OK,' + pid)
                exchange(b'FF,85,85,85,85\n', 'OK,FF,85,85,85,85')
            exchange(b'START,1\n', 'OK,START,1')
            flag = b'PWMF,1\n' if spec['timer1_fast'] else b'PWMF,0\n'
            exchange(flag, 'OK,' + flag.decode().strip())
            collect(.15, 'configure')
            if not resting(rows): raise RuntimeError('Configuration energized/moved motor')
        power(.2)
        if armed:
            power_monitor = PowerMonitor(command, clock=time.monotonic)
            power_monitor.start()
            collect(.15, 'configure')
            monitored_power()
            if not resting(rows): raise RuntimeError('Power-monitor warmup moved motor')
        baseline = rows[-1]
        if armed:
            began = time.monotonic(); guard = Guard(baseline, began, sign, schedule)
            deadline = began + SECONDS - STOP_MARGIN
            next_refresh = began
            try:
                while time.monotonic() < deadline:
                    now = time.monotonic(); guard.check(now); monitored_power()
                    if now >= next_refresh:
                        if schedule:
                            duties = duty_at(schedule, now - began)
                            guard.commanded(duties)
                            motion = 'DUTY,{},{}'.format(*duties)
                        else:
                            motion = 'VEL,{0},{0}'.format(sign * 200)
                        expected_acks.append('OK,' + motion)
                        send((motion + '\n').encode()); result['motion_sent'] = True
                        next_refresh = time.monotonic() + REFRESH
                    if time.monotonic() >= deadline: break
                    read_one('motion', guard)
                if not all(guard.run_seen): raise RuntimeError('Startup never confirmed RUN on both wheels')
            except Exception as error:
                result['reason'] = str(error)
            finally:
                send(b'STOP\n')
                result['motion_to_stop_s'] = time.monotonic() - began
                result['run_phase_seen'] = guard.run_seen
        else:
            send(b'STOP\n')
        mark = len(lines); collect(.9, 'after_stop')
        result['stop_confirmed'] = 'OK,STOP' in lines[mark:] and resting(rows)
        if not result['stop_confirmed']: raise RuntimeError('STOP/stationary state unconfirmed')
        result['encoder_delta'] = [counter_delta(rows[-1][i], baseline[i]) for i in (1, 2)]
        if armed: restore()
        if power_monitor is not None:
            result['power_monitor_closed'] = power_monitor.close()
            if not result['power_monitor_closed']: raise RuntimeError('Power monitor did not stop')
        power(.2)
        result['final_adc'] = command(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'])
        return 0 if result['reason'] is None else 2
    except Exception as error:
        result['reason'] = result['reason'] or str(error)
        return 2
    finally:
        if connection is not None:
            try:
                send(b'STOP\n')
                if armed and not result['restore_confirmed']:
                    mark = len(lines); collect(.9, 'after_stop')
                    stopped = 'OK,STOP' in lines[mark:] and resting(rows)
                    result['stop_confirmed'] = result['stop_confirmed'] or stopped
                    if stopped: restore()
            except Exception as error:
                result['cleanup_error'] = str(error)
                try: send(b'STOP\n')  # Never leave a config command as the last word.
                except Exception as stop_error: result['final_stop_error'] = str(stop_error)
            connection.close(); result['serial_closed'] = True
        if power_monitor is not None and not result['power_monitor_closed']:
            result['power_monitor_closed'] = power_monitor.close()
        if power_monitor is None: result['power_monitor_closed'] = True
        result['elapsed_s'] = time.monotonic() - started
        print(json.dumps(result, indent=2))


if __name__ == '__main__': raise SystemExit(main())
