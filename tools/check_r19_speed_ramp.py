#!/usr/bin/env python3
"""One closed-loop forward speed-step run on P230B3 (START,0 normal PI); default STOP/INFO/CAPS only.

2026-10-14: gradual acceleration toward ~1.5 m, with planned driving capped
at 4200 ticks (~1.2 m) because coast adds 10-20 cm on the
1.5 m of clear floor. Steps 300/600/900/1200 ticks/s, 1 s each, then hold 1200
until 4200 ticks average travel, then STOP. Profile PI 0.20/0.80, Kv 0.09,
static FF per direction derived from 13-14 Oct data. Host guards from 50 ms
telemetry: 4400 ticks/wheel, 160-tick separation, 2000 ticks/s, 3 backward
ticks, 300 ms response, 250 ms progress, 150 ms freshness, 7 s maximum.
Firmware keeps the 230 clamp and 350 ms watchdog. One run, no retry.
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
PROFILES = {
    'steps1200': ((0.0, 300), (1.0, 600), (2.0, 900), (3.0, 1200)),
    # 2026-10-14: 0.43 m/s (1500 ticks/s) as intended max speed.
    'steps1500': ((0.0, 400), (0.5, 800), (1.0, 1200), (1.5, 1500)),
}
STEPS = PROFILES['steps1200']
PLANNED_TICKS = 4200
SECONDS = 7.0
PID = 'PID,200,800,90,70'
FF = 'FF,70,55,42,63'  # left fwd/rev, right fwd/rev static PWM; Kv 0.09 adds per tick/s
RESTORE_PID = 'PID,80,200,0,75'
RESTORE_FF = 'FF,75,60,75,60'
MAX_TRAVEL_TICKS = 4400
MAX_PAIR_SEPARATION_TICKS = 160
MAX_SPEED = 2000
MAX_BACKWARD_TICKS = 3
STALE = .15
NO_RESPONSE = .30
NO_PROGRESS = .25
REFRESH = .08
STOP_MARGIN = .03
CEILING = 230


def parse_t(line):
    if not line.startswith('T,'): return None
    parts = line.split(',')
    if len(parts) != 10: raise ValueError('Wrong telemetry shape')
    row = [int(x) for x in parts[1:]]
    if not 0 <= row[0] < 2**32: raise ValueError('Invalid clock')
    if any(not -2**31 <= x < 2**31 for x in row[1:3]): raise ValueError('Invalid counters')
    if any(abs(x) > CEILING for x in row[3:5]): raise ValueError('PWM outside ceiling')
    return row


class Guard:
    def __init__(self, baseline, started):
        self.targets = [0, 0]
        self.previous = self.baseline = baseline
        self.started = self.last_received = self.last_ack = started
        self.last_progress = [started, started]
        self.responded = [False, False]
        self.peak = [0., 0.]

    def ack(self, now): self.last_ack = now

    def observe(self, row, now):
        dt = counter_delta(row[0], self.previous[0])
        if dt <= 0 or dt > 250: raise RuntimeError('Controller clock discontinuity')
        grace = now - self.started <= .10 and row[5:7] == [0, 0]
        if not grace and (row[5] != row[6] or row[5] not in [v for _, v in STEPS]):
            raise RuntimeError('Unexpected velocity target')
        if any(not 0 <= x <= CEILING for x in row[3:5]): raise RuntimeError('Unexpected motor output')
        totals = []
        for side in (0, 1):
            step = counter_delta(row[1 + side], self.previous[1 + side])
            total = counter_delta(row[1 + side], self.baseline[1 + side])
            if step <= -MAX_BACKWARD_TICKS or total <= -MAX_BACKWARD_TICKS:
                raise RuntimeError('Encoder direction disagrees with request')
            speed = step * 1000 / dt
            self.peak[side] = max(self.peak[side], speed)
            if speed > MAX_SPEED: raise RuntimeError('Speed exceeded test bound')
            if step > 0:
                self.last_progress[side] = now; self.responded[side] = True
            if total > MAX_TRAVEL_TICKS: raise RuntimeError('Per-wheel encoder travel budget exceeded')
            totals.append(total)
        if abs(totals[0] - totals[1]) > MAX_PAIR_SEPARATION_TICKS:
            raise RuntimeError('Paired encoder separation exceeded bound')
        self.previous, self.last_received = row, now

    def check(self, now):
        if now - self.last_received > STALE: raise RuntimeError('Stale telemetry')
        if now - self.last_ack > STALE: raise RuntimeError('Stale VEL acknowledgement')
        if now - self.started > SECONDS: raise RuntimeError('Run exceeded time bound')
        for side in (0, 1):
            if now - self.started >= NO_RESPONSE and not self.responded[side]:
                raise RuntimeError('No encoder response: ' + ('left' if side == 0 else 'right'))
            if self.responded[side] and now - self.last_progress[side] > NO_PROGRESS:
                raise RuntimeError('Encoder progress stopped: ' + ('left' if side == 0 else 'right'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', help='send the one declared speed-step run')
    parser.add_argument('--profile', choices=tuple(PROFILES), default='steps1200')
    args = parser.parse_args()
    global STEPS
    STEPS = PROFILES[args.profile]
    armed = args.run
    acks = []
    result = {'episode': 'speed-steps-forward' if armed else None, 'port': PORT, 'steps': [list(x) for x in STEPS],
              'planned_ticks': PLANNED_TICKS, 'profile': {'pid': PID, 'ff': FF},
              'nominal_seconds': SECONDS if armed else 0, 'controller_info': None, 'caps': None,
              'motion_sent': False, 'commands': [], 'telemetry': [], 'power_samples': [], 'events': [],
              'stop_confirmed': False, 'restore_confirmed': not armed, 'serial_closed': False,
              'reason': None, 'power_monitor_closed': not armed,
              'limits': {'travel': MAX_TRAVEL_TICKS, 'separation': MAX_PAIR_SEPARATION_TICKS,
                         'speed': MAX_SPEED, 'backward': MAX_BACKWARD_TICKS, 'stale': STALE,
                         'response': NO_RESPONSE, 'progress': NO_PROGRESS, 'refresh': REFRESH,
                         'power_query_s': QUERY_TIMEOUT, 'power_sample_age_s': MAX_SAMPLE_AGE,
                         'power_poll_s': POLL_INTERVAL},
              'unmeasured': ['motor VM/current', 'individual cells', 'physical distance'],
              'no_automatic_retry': True}
    connection = None
    rows, lines = [], []
    started = time.monotonic()
    initial_flags = None
    guard = None
    power_monitor = None

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

    def send(data):
        if connection.write(data) != len(data): raise RuntimeError('Partial serial write')
        result['commands'].append({'elapsed_s': time.monotonic() - started, 'command': data.decode().strip()})

    def read_one(phase, active=None):
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
        if line.startswith('OK,VEL,'):
            if not armed or line not in acks[-2:]: raise RuntimeError('Unexpected VEL ack')
            if active: active.ack(now)
        if line.startswith(('OK,PWM,', 'B,')): raise RuntimeError('Unexpected direct-PWM/opt-in output')
        row = parse_t(line)
        if row is not None:
            if rows and counter_delta(row[0], rows[-1][0]) <= 0:
                raise RuntimeError('Controller clock restarted/repeated')
            rows.append(row)
            result['telemetry'].append({'elapsed_s': now - started, 'phase': phase, 'values': row})
            if active: active.observe(row, now)

    def collect(duration, phase):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline: read_one(phase)

    def exchange(text, phase):
        mark = len(lines); send((text + '\n').encode())
        deadline = time.monotonic() + .25
        while time.monotonic() < deadline:
            read_one(phase)
            if 'OK,' + text in lines[mark:]: return
        raise RuntimeError('Missing ACK: ' + text)

    def restore():
        exchange(RESTORE_PID, 'cleanup'); exchange(RESTORE_FF, 'cleanup')
        result['restore_confirmed'] = True

    def interrupted(signum, frame): raise RuntimeError('Interrupted; STOP required')
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(sig, interrupted)
    try:
        identity = command(['udevadm', 'info', '--query=property', '--name=' + PORT])
        if 'ID_VENDOR_ID=1a86\n' not in identity + '\n' or 'ID_MODEL_ID=7523\n' not in identity + '\n':
            raise RuntimeError('Unexpected Nano identity')
        owner = subprocess.run(['fuser', PORT], capture_output=True, text=True, timeout=2)
        if owner.returncode != 1 or owner.stdout.strip() or owner.stderr.strip():
            raise RuntimeError('Port busy/owner check failed')
        initial_flags = power(.2)
        import serial
        connection = serial.Serial(PORT, 115200, timeout=.01, write_timeout=.05, exclusive=True)
        time.sleep(2); connection.reset_input_buffer()
        send(b'STOP\n'); send(b'INFO\n'); send(b'CAPS\n'); collect(.7, 'preflight')
        if result['controller_info'] != INFO or result['caps'] != CAPS or 'OK,STOP' not in lines or not resting(rows):
            raise RuntimeError('Stopped candidate identity not confirmed')
        if armed:
            exchange(PID, 'configure'); exchange(FF, 'configure'); exchange('START,0', 'configure')
            collect(.15, 'configure')
            if not resting(rows): raise RuntimeError('Configuration moved motor')
        power(.2)
        if armed:
            power_monitor = PowerMonitor(command, clock=time.monotonic)
            power_monitor.start()
            collect(.15, 'configure')
            for sample in power_monitor.check():
                check_flags(sample['flags'], sample['began'], sample['completed'])
            if not resting(rows): raise RuntimeError('Monitor warmup moved motor')
        baseline = rows[-1]
        if armed:
            began = time.monotonic(); guard = Guard(baseline, began)
            deadline = began + SECONDS - .5  # planned stop by travel well before the time guard
            next_refresh = began
            try:
                while time.monotonic() < deadline:
                    now = time.monotonic(); guard.check(now)
                    for sample in power_monitor.check():
                        check_flags(sample['flags'], sample['began'], sample['completed'])
                    travel = sum(counter_delta(guard.previous[i], baseline[i]) for i in (1, 2)) / 2
                    if travel >= PLANNED_TICKS:
                        result['planned_stop'] = True
                        break
                    if now >= next_refresh:
                        target = [v for t, v in STEPS if now - began >= t][-1]
                        motion = f'VEL,{target},{target}'
                        acks.append('OK,' + motion)
                        send((motion + '\n').encode()); result['motion_sent'] = True
                        next_refresh = time.monotonic() + REFRESH
                    if time.monotonic() >= deadline: break
                    read_one('motion', guard)
                if not result.get('planned_stop'):
                    raise RuntimeError('Time bound reached before planned travel')
            except Exception as error:
                result['reason'] = str(error)
            finally:
                send(b'STOP\n')
                result['motion_to_stop_s'] = time.monotonic() - began
                result['peak_ticks_per_s'] = guard.peak
                result['ticks_at_stop'] = [counter_delta(guard.previous[i], baseline[i]) for i in (1, 2)]
        else:
            send(b'STOP\n')
        mark = len(lines); collect(1.5, 'after_stop')
        result['stop_confirmed'] = 'OK,STOP' in lines[mark:] and resting(rows)
        if not result['stop_confirmed']: raise RuntimeError('STOP/stationary state unconfirmed')
        result['encoder_delta'] = [counter_delta(rows[-1][i], baseline[i]) for i in (1, 2)]
        if armed: restore()
        if power_monitor is not None:
            result['power_monitor_closed'] = power_monitor.close()
        power(.2)
        return 0 if result['reason'] is None else 2
    except Exception as error:
        result['reason'] = result['reason'] or str(error)
        return 2
    finally:
        if connection is not None:
            try:
                send(b'STOP\n')
                if armed and not result['restore_confirmed']:
                    collect(.9, 'after_stop'); restore(); send(b'STOP\n')
            except Exception as error: result['cleanup_error'] = str(error)
            connection.close(); result['serial_closed'] = True
        if power_monitor is not None and not result['power_monitor_closed']:
            result['power_monitor_closed'] = power_monitor.close()
        result['elapsed_s'] = time.monotonic() - started
        print(json.dumps(result, indent=2))


if __name__ == '__main__': raise SystemExit(main())
