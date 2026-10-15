#!/usr/bin/env python3
"""Explicitly authorized single selected-channel pulse, never a drive launch.

Requires --pulse-channel and exactly one explicit setup/arm flag.
Historical floor/forward behavior is retained. Reverse requires either
--arm-supervised-lifted-one-second or the separate, explicitly authorized
--arm-supervised-floor-reverse-one-second. A floor pulse pivots the robot;
the unpowered wheel can roll passively. Floor reverse also stops on loss
of encoder progress after an initial response.
Existing PWM100 ceiling retained;
no firmware/PID/FF/VEL/configuration changes. Nominal one-second episode with
a 30ms STOP scheduling margin, 80ms refreshes, and early feedback/power abort.
Operator must keep clear with a reachable physical motor-power disconnect.
Neither software deadlines nor the published watchdog are safety certification.
"""
import argparse
import json
import re
import signal
import subprocess
import time

from check_r19_lifted_wheel import (
    PORT, EXPECTED_INFO, command, counter_delta, parse_telemetry, resting,
)

PWM = 100
REQUESTED_SECONDS = 1.0
STOP_MARGIN = .03
REFRESH_SECONDS = .08
STALE_SECONDS = .15
NO_RESPONSE_SECONDS = .30
FLOOR_NO_PROGRESS_SECONDS = .25


def pulse_command(channel, direction='forward'):
    if channel not in ('left', 'right'):
        raise ValueError('Exactly one left/right channel must be selected')
    if direction not in ('forward', 'reverse'):
        raise ValueError('Exactly one forward/reverse direction must be selected')
    pwm = PWM if direction == 'forward' else -PWM
    return (f'PWM,{pwm},0\n' if channel == 'left' else
            f'PWM,0,{pwm}\n').encode('ascii')


class FeedbackGuard:
    def __init__(self, baseline, started, channel='left', direction='forward',
                 setup='lifted'):
        pulse_command(channel, direction)  # Validate before any hardware action.
        if setup not in ('floor', 'lifted'):
            raise ValueError('Explicit floor/lifted setup required')
        self.setup = setup
        self.direction = direction
        self.expected_pwm = PWM if direction == 'forward' else -PWM
        self.pwm_index = 3 if channel == 'left' else 4
        self.other_pwm_index = 7-self.pwm_index
        self.encoder_index = 1 if channel == 'left' else 2
        self.other_encoder_index = 3-self.encoder_index
        self.baseline = baseline
        self.previous = baseline
        self.started = started
        self.last_received = started
        self.last_progress = started
        self.responded = False

    def observe(self, row, now):
        clock_delta = counter_delta(row[0], self.previous[0])
        if clock_delta <= 0 or clock_delta > 250:
            raise RuntimeError('Controller clock reset, duplicated or discontinuous')
        if (row[self.other_pwm_index] != 0 or
                row[self.pwm_index] not in (0, self.expected_pwm) or
                row[5:7] != [0, 0]):
            raise RuntimeError('Unexpected motor output or velocity target')
        selected_delta = counter_delta(row[self.encoder_index],
                                       self.baseline[self.encoder_index])
        # Historical forward checks retain their behavior. Reverse signs are
        # checked in both setups; passive movement of the unpowered wheel is
        # possible on the floor, but its commanded PWM must always remain zero.
        if self.direction == 'reverse':
            if selected_delta > 0:
                raise RuntimeError('Encoder count increased during reverse pulse')
            if (self.setup == 'lifted' and
                    counter_delta(row[self.other_encoder_index],
                                  self.baseline[self.other_encoder_index]) != 0):
                raise RuntimeError('Unselected encoder moved during reverse pulse')
        if counter_delta(row[self.encoder_index],
                         self.previous[self.encoder_index]) != 0:
            self.last_progress = now
        self.responded |= (selected_delta != 0)
        self.previous = row
        self.last_received = now

    def check(self, now):
        if now - self.last_received > STALE_SECONDS:
            raise RuntimeError('Stale controller telemetry; stopping pulse')
        if now - self.started >= NO_RESPONSE_SECONDS and not self.responded:
            raise RuntimeError('No encoder response at PWM100; stopping pulse early')
        if (self.setup == 'floor' and self.direction == 'reverse' and
                self.responded and now-self.last_progress > FLOOR_NO_PROGRESS_SECONDS):
            raise RuntimeError('Encoder progress stopped during floor reverse; stopping pulse')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pulse-channel', choices=['left', 'right'], required=True)
    parser.add_argument('--pulse-direction', choices=['forward', 'reverse'],
                        default='forward')
    arm = parser.add_mutually_exclusive_group(required=True)
    arm.add_argument('--arm-supervised-floor-one-second', action='store_true')
    arm.add_argument('--arm-supervised-lifted-one-second', action='store_true')
    arm.add_argument('--arm-supervised-floor-reverse-one-second', action='store_true')
    args = parser.parse_args()
    if args.arm_supervised_floor_reverse_one_second and args.pulse_direction != 'reverse':
        parser.error('Floor-reverse arm requires --pulse-direction reverse')
    if (args.pulse_direction == 'reverse' and
            not (args.arm_supervised_lifted_one_second or
                 args.arm_supervised_floor_reverse_one_second)):
        parser.error('Reverse requires an explicit lifted or floor-reverse arm')
    setup = 'lifted' if args.arm_supervised_lifted_one_second else 'floor'
    selected_pwm = PWM if args.pulse_direction == 'forward' else -PWM
    result = {
        'port': PORT, 'requested_channel': args.pulse_channel, 'pwm': selected_pwm,
        'requested_direction': args.pulse_direction,
        'requested_setup': setup,
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
    if args.arm_supervised_floor_reverse_one_second:
        result['floor_no_progress_seconds'] = FLOOR_NO_PROGRESS_SECONDS
        result['limits'].extend([
            'A selected-wheel floor pulse pivots the robot; the unpowered wheel may roll.',
            'Floor loading is different from the earlier lifted checks.',
            'Encoder progress does not measure motor current or certify stall protection.',
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
        pulse_started = time.monotonic()
        guard = FeedbackGuard(baseline, pulse_started, args.pulse_channel,
                              args.pulse_direction, setup)
        deadline = pulse_started+REQUESTED_SECONDS-STOP_MARGIN
        next_refresh = pulse_started
        next_power = pulse_started+.15
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                guard.check(now)
                if now >= next_refresh:
                    send(pulse_command(args.pulse_channel, args.pulse_direction))
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
