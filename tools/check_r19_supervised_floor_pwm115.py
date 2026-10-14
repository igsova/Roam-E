#!/usr/bin/env python3
"""One fixed PWM115 (~45%) left-forward floor startup check.

Uses the already-flashed PWM153 controller; no firmware/PID/FF change.
Default STOP/INFO only. Requires the explicit nominal one-second arm and human
watching with clear floor/reachable disconnect. Battery/per-cell voltages are
operator readings, not measured by the Pi. Do not treat one successful episode
as the minimum reliable start or the minimum running duty. No automatic sweep,
retry, arbitrary PWM/duration/channel, VEL or ROS/navigation launch.
Retains exact fresh ACK/telemetry, direction, response300ms, progress250ms,
Pi power checks and STOP cleanup. Cell cutoff/current protection is NOT supplied
by these software guards; operator must monitor battery and stop if necessary.
"""
import argparse
import json
import re
import signal
import subprocess
import time

from check_r19_lifted_wheel import (
    PORT, command, counter_delta, resting,
)

EXPECTED_INFO = 'INFO,2,748,153,2000'
PWM_CEILING = 153
PWM = 115
REQUESTED_SECONDS = 1.0
STOP_MARGIN = .03
REFRESH_SECONDS = .08
STALE_SECONDS = .15
NO_RESPONSE_SECONDS = .30
FLOOR_NO_PROGRESS_SECONDS = .25


def pulse_command():
    return b'PWM,115,0\n'


def parse_telemetry(line):
    if not line.startswith('T,'):
        return None
    fields = line.split(',')
    if len(fields) != 10:
        raise ValueError('Expected protocol-v2 telemetry')
    values = [int(x) for x in fields[1:]]
    if not 0 <= values[0] < 2**32:
        raise ValueError('Invalid controller clock')
    if any(not -2**31 <= x < 2**31 for x in values[1:3]):
        raise ValueError('Invalid encoder counters')
    if any(not -PWM_CEILING <= x <= PWM_CEILING for x in values[3:5]):
        raise ValueError('Unexpected PWM outside153 ceiling')
    return values


class FeedbackGuard:
    def __init__(self, baseline, started):
        self.previous = baseline
        self.started = started
        self.last_received = started
        self.last_ack = started
        self.last_progress = started
        self.responded = False

    def acknowledge(self, now):
        self.last_ack = now

    def observe(self, row, now):
        clock_delta = counter_delta(row[0], self.previous[0])
        if clock_delta <= 0 or clock_delta > 250:
            raise RuntimeError('Controller clock reset, duplicated or discontinuous')
        if row[4] != 0 or row[3] not in (0, PWM) or row[5:7] != [0, 0]:
            raise RuntimeError('Unexpected motor output or velocity target')
        step = counter_delta(row[1], self.previous[1])
        if step < 0:
            raise RuntimeError('Negative encoder progress during left-forward pulse')
        if step > 0:
            self.last_progress = now
            self.responded = True
        # Unpowered right encoder may roll passively during a floor pivot.
        self.previous = row
        self.last_received = now

    def check(self, now):
        if now-self.last_received > STALE_SECONDS:
            raise RuntimeError('Stale controller telemetry; stopping pulse')
        if now-self.last_ack > STALE_SECONDS:
            raise RuntimeError('Missing PWM acknowledgement; stopping pulse')
        if now-self.started >= NO_RESPONSE_SECONDS and not self.responded:
            raise RuntimeError('No encoder response at PWM115; stopping pulse early')
        if self.responded and now-self.last_progress > FLOOR_NO_PROGRESS_SECONDS:
            raise RuntimeError('Encoder progress stopped; stopping pulse')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-supervised-floor-left-forward-pwm115-one-second',
                        action='store_true')
    args = parser.parse_args()
    armed = args.arm_supervised_floor_left_forward_pwm115_one_second
    result = {
        'port': PORT, 'requested_channel': 'left' if armed else None, 'pwm': PWM,
        'pwm_ceiling': PWM_CEILING,
        'requested_direction': 'forward' if armed else None,
        'requested_setup': 'floor' if armed else None,
        'requested_seconds': REQUESTED_SECONDS, 'stop_scheduling_margin_s': STOP_MARGIN,
        'refresh_seconds': REFRESH_SECONDS, 'controller_info': None,
        'pulse_sent': False, 'commands': [], 'telemetry': [], 'power_samples': [],
        'stop_confirmed': False, 'serial_closed': False,
        'reason': None, 'physical_wheel_observation': None,
        'limits': ['Physical channel mapping requires an operator observation.',
                   'The setup flag is not an independent physical readiness check.',
                   'Encoder activity alone does not verify physical direction.',
                   'Pi flags do not measure motor-branch voltage/current.',
                   'Software timing/watchdog is not a physical emergency disconnect.'],
    }
    result['floor_no_progress_seconds'] = FLOOR_NO_PROGRESS_SECONDS
    result['limits'].extend([
        'A left-wheel floor pulse pivots the robot; the unpowered right may roll.',
        'Driver/motor loaded voltage and current are not measured.',
        'Battery/cell voltages must be monitored by the operator; Pi has no cell cutoff.',
        'A single success does not qualify a minimum reliable starting duty.',
        'Default mode never sends PWM; arm flag does not verify physical readiness.',
    ])
    connection = None
    started = time.monotonic()
    rows, lines = [], []
    baseline = None
    pulse_started = None
    initial_flags = None

    def check_power(timeout=.05):
        raw = command(['vcgencmd', 'get_throttled'], timeout=timeout)
        if not re.fullmatch(r'throttled=0x[0-9a-fA-F]+', raw):
            raise RuntimeError('Invalid Pi power flags')
        flags = int(raw.split('=')[1], 16)
        result['power_samples'].append({'elapsed_s': time.monotonic()-started,
                                        'throttled': hex(flags)})
        if flags & 0x5 or (initial_flags is not None and
                          (flags & ~initial_flags) & 0x50000):
            raise RuntimeError('Pi power/throttling fault; stopping test')
        return flags

    def send(data):
        written = connection.write(data)
        if written != len(data):
            raise RuntimeError('Incomplete serial write')
        result['commands'].append({'elapsed_s': time.monotonic()-started,
                                   'command': data.decode('ascii').strip()})

    def read_one(phase, guard=None):
        raw = connection.readline(128)
        if not raw:
            return
        if not raw.endswith(b'\n'):
            raise RuntimeError('Partial/overlong telemetry line')
        line = raw.decode('ascii').strip()
        lines.append(line)
        if line.startswith(('ERR,', 'EVENT,')):
            raise RuntimeError('Controller event: '+line)
        if line.startswith('INFO,'):
            if line != EXPECTED_INFO:
                raise RuntimeError('Unexpected controller INFO')
            result['controller_info'] = line
        if line.startswith('OK,PWM,') and guard is not None:
            if line != 'OK,PWM,115,0':
                raise RuntimeError('Unexpected PWM acknowledgement')
            guard.acknowledge(time.monotonic())
        row = parse_telemetry(line)
        if row is None:
            return
        now = time.monotonic()
        if rows and counter_delta(row[0], rows[-1][0]) <= 0:
            raise RuntimeError('Controller clock restarted or repeated')
        rows.append(row)
        result['telemetry'].append({'elapsed_s': now-started,
                                    'phase': phase, 'values': row})
        if guard is not None:
            guard.observe(row, now)

    def collect(seconds, phase):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            read_one(phase)

    def interrupted(signum, frame):
        raise RuntimeError('Interrupted; STOP required')

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    try:
        identity = command(['udevadm', 'info', '--query=property', '--name='+PORT])
        if ('ID_VENDOR_ID=1a86\n' not in identity+'\n' or
                'ID_MODEL_ID=7523\n' not in identity+'\n'):
            raise RuntimeError('Unexpected Arduino USB identity')
        owner = subprocess.run(['fuser', PORT], text=True, capture_output=True, timeout=2)
        if owner.returncode != 1 or owner.stdout.strip() or owner.stderr.strip():
            raise RuntimeError('Arduino port occupied or owner check failed')
        initial_flags = check_power(timeout=.2)
        result['initial_adc'] = command(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'])
        import serial
        connection = serial.Serial(PORT, 115200, timeout=.01,
                                   write_timeout=.05, exclusive=True)
        time.sleep(2)
        connection.reset_input_buffer()
        send(b'STOP\n')
        send(b'INFO\n')
        collect(.7, 'preflight')
        if result['controller_info'] != EXPECTED_INFO or 'OK,STOP' not in lines:
            raise RuntimeError('Controller identity and STOP not confirmed')
        if not resting(rows):
            raise RuntimeError('Controller not stationary/stopped before test')
        baseline = rows[-1]
        # This query runs while stopped, before the pulse clock/first PWM.
        # Allow the same 200ms setup budget as initial/final checks; retain
        # the 50ms timeout for every power query while the pulse is active.
        check_power(timeout=.2)
        if armed:
            pulse_started = time.monotonic()
            guard = FeedbackGuard(baseline, pulse_started)
            deadline = pulse_started+REQUESTED_SECONDS-STOP_MARGIN
            next_refresh = pulse_started
            next_power = pulse_started+.15
            try:
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    guard.check(now)
                    if now >= next_refresh:
                        send(pulse_command())
                        result['pulse_sent'] = True
                        next_refresh = time.monotonic()+REFRESH_SECONDS
                    if now >= next_power and deadline-time.monotonic() > .06:
                        check_power()
                        next_power = time.monotonic()+.15
                    if time.monotonic() >= deadline:
                        break
                    read_one('pulse', guard)
            except Exception as error:
                result['reason'] = str(error)
            finally:
                send(b'STOP\n')
                result['pulse_to_stop_elapsed_s'] = time.monotonic()-pulse_started
        else:
            send(b'STOP\n')
        stop_start = len(lines)
        collect(.9, 'after_stop')
        result['stop_confirmed'] = ('OK,STOP' in lines[stop_start:] and resting(rows))
        if not result['stop_confirmed']:
            raise RuntimeError('Post-test STOP/stationary state not confirmed')
        result['encoder_delta'] = [counter_delta(rows[-1][i], baseline[i]) for i in (1, 2)]
        check_power(timeout=.2)
        result['final_adc'] = command(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'])
        return 0 if result['reason'] is None else 2
    except Exception as error:
        result['reason'] = str(error)
        return 2
    finally:
        if connection is not None:
            try:
                send(b'STOP\n')
            except Exception as error:
                result['cleanup_stop_error'] = str(error)
            connection.close()
            result['serial_closed'] = True
        result['elapsed_s'] = time.monotonic()-started
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    raise SystemExit(main())



