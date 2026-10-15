#!/usr/bin/env python3
"""One explicitly armed, supervised one-wheel FLOOR velocity test.

Fixed target 200 ticks/s forward, PWM ceiling 100. Nominal one second, or
1.5 seconds only with its separate explicit arm. Progress guards are unchanged.
Three seconds is available ONLY for left FF85/Kp0.16 with a separate explicit
--arm-supervised-floor-velocity-three-seconds arm; it does not relax any guard.
The other wheel is commanded zero; it may roll passively as the robot pivots.
Default uses the already-flashed controller without PID/FF changes. The separate
--candidate-left-forward-ff85 option permits ONLY a nominal one-second left
episode, temporarily sets FF,85,60,75,60 while STOPPED, then explicitly restores
the known FF,75,60,75,60 baseline while STOPPED. It is not a measured calibration
or a firmware flash. Each profile change requires an acknowledgement.
--candidate-left-forward-ff85-kp160 adds only Kp 0.080 -> 0.160 compared
with FF85 alone. PID first resets all static values to 75, so a stopped-only
FF command must follow it. Cleanup restores PID,80,200,0,75 THEN the complete
FF,75,60,75,60 baseline. Neither candidate permits right/legacy1.5s episodes;
only the Kp0.16 candidate permits the explicitly armed three-second check.
No firmware upload, ROS drive launch, geometry writes, or automatic retries.
Human supervision, a clear floor and reachable motor-power disconnect required.
Software STOP/watchdog and encoder guards do not certify physical safety.
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

TARGET_TICKS_PER_SECOND = 200
PWM_CEILING = 100
REQUESTED_SECONDS = 1.0
EXTENDED_REQUESTED_SECONDS = 1.5
SETTLING_REQUESTED_SECONDS = 3.0
STOP_MARGIN = .03
REFRESH_SECONDS = .08
STALE_SECONDS = .15
ACK_SECONDS = .15
STARTUP_TARGET_GRACE_SECONDS = .10
NO_RESPONSE_SECONDS = .30
NO_PROGRESS_SECONDS = .25
MAX_FEEDBACK_TICKS_PER_SECOND = 1000
BASELINE_FEEDFORWARD = (75, 60, 75, 60)
CANDIDATE_FEEDFORWARD = (85, 60, 75, 60)
BASELINE_PID = (80, 200, 0, 75)
CANDIDATE_PID = (160, 200, 0, 75)
CONFIGURATION_PHASES = {
    'pid_candidate', 'pid_restore', 'feedforward_candidate',
    'feedforward_restore', 'cleanup_stop',
}


def velocity_command(channel):
    if channel not in ('left', 'right'):
        raise ValueError('Exactly one left/right channel must be selected')
    return (b'VEL,200,0\n' if channel == 'left' else b'VEL,0,200\n')


class FeedbackGuard:
    def __init__(self, baseline, started, channel):
        velocity_command(channel)
        self.pwm_index = 3 if channel == 'left' else 4
        self.other_pwm_index = 7-self.pwm_index
        self.encoder_index = 1 if channel == 'left' else 2
        self.measured_index = self.encoder_index+6
        self.targets = [TARGET_TICKS_PER_SECOND, 0] if channel == 'left' else [0, TARGET_TICKS_PER_SECOND]
        self.previous = baseline
        self.started = started
        self.last_received = started
        self.last_progress = started
        self.responded = False
        self.acknowledged = False

    def observe(self, row, now):
        clock_delta = counter_delta(row[0], self.previous[0])
        if clock_delta <= 0 or clock_delta > 250:
            raise RuntimeError('Controller clock reset, duplicated or discontinuous')
        if row[self.other_pwm_index] != 0 or not 0 <= row[self.pwm_index] <= PWM_CEILING:
            raise RuntimeError('Unexpected motor output during forward velocity test')
        startup_idle = (now-self.started <= STARTUP_TARGET_GRACE_SECONDS and
                        row[3:7] == [0, 0, 0, 0])
        if row[5:7] != self.targets and not startup_idle:
            raise RuntimeError('Unexpected velocity target')
        step = counter_delta(row[self.encoder_index], self.previous[self.encoder_index])
        if step < 0:
            raise RuntimeError('Encoder moved opposite to forward velocity target')
        if (step*1000/clock_delta > MAX_FEEDBACK_TICKS_PER_SECOND or
                abs(row[self.measured_index]) > MAX_FEEDBACK_TICKS_PER_SECOND):
            raise RuntimeError('Encoder speed exceeded test feedback bound')
        # Passive counts on the unpowered wheel are allowed on the floor.
        if step > 0:
            self.last_progress = now
            self.responded = True
        self.previous = row
        self.last_received = now

    def check(self, now):
        if now-self.last_received > STALE_SECONDS:
            raise RuntimeError('Stale controller telemetry; stopping velocity test')
        if now-self.started > ACK_SECONDS and not self.acknowledged:
            raise RuntimeError('Velocity command acknowledgement missing')
        if now-self.started >= NO_RESPONSE_SECONDS and not self.responded:
            raise RuntimeError('No encoder response; stopping velocity test')
        if self.responded and now-self.last_progress > NO_PROGRESS_SECONDS:
            raise RuntimeError('Encoder progress stopped; stopping velocity test')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--velocity-channel', choices=['left', 'right'], required=True)
    arm = parser.add_mutually_exclusive_group(required=True)
    arm.add_argument('--arm-supervised-floor-velocity-one-second', action='store_true')
    arm.add_argument('--arm-supervised-floor-velocity-one-point-five-seconds', action='store_true')
    arm.add_argument('--arm-supervised-floor-velocity-three-seconds', action='store_true')
    candidate = parser.add_mutually_exclusive_group()
    candidate.add_argument('--candidate-left-forward-ff85', action='store_true')
    candidate.add_argument('--candidate-left-forward-ff85-kp160', action='store_true')
    args = parser.parse_args()
    pid_candidate = args.candidate_left_forward_ff85_kp160
    candidate_requested = args.candidate_left_forward_ff85 or pid_candidate
    settling = args.arm_supervised_floor_velocity_three_seconds
    if candidate_requested and (
            args.velocity_channel != 'left' or
            not (args.arm_supervised_floor_velocity_one_second or
                 (pid_candidate and settling))):
        parser.error('Candidate requires left/one-second, or Kp0.16/three-second scope')
    if settling and not pid_candidate:
        parser.error('Three-second arm requires the left FF85/Kp0.16 candidate')
    requested_seconds = (SETTLING_REQUESTED_SECONDS if settling else
                         EXTENDED_REQUESTED_SECONDS
                         if args.arm_supervised_floor_velocity_one_point_five_seconds
                         else REQUESTED_SECONDS)
    expected_command = velocity_command(args.velocity_channel)
    expected_ack = 'OK,'+expected_command.decode('ascii').strip()
    result = {
        'port': PORT, 'requested_channel': args.velocity_channel,
        'requested_mode': 'VEL', 'requested_direction': 'forward',
        'requested_setup': 'floor', 'target_ticks_per_second': TARGET_TICKS_PER_SECOND,
        'pwm_ceiling': PWM_CEILING, 'requested_seconds': requested_seconds,
        'stop_scheduling_margin_s': STOP_MARGIN, 'refresh_seconds': REFRESH_SECONDS,
        'no_progress_seconds': NO_PROGRESS_SECONDS,
        'no_response_seconds': NO_RESPONSE_SECONDS,
        'stale_seconds': STALE_SECONDS, 'ack_seconds': ACK_SECONDS,
        'startup_target_grace_seconds': STARTUP_TARGET_GRACE_SECONDS,
        'max_feedback_ticks_per_second': MAX_FEEDBACK_TICKS_PER_SECOND,
        'controller_info': None, 'velocity_sent': False, 'velocity_acknowledgements': 0,
        'commands': [], 'telemetry': [], 'power_samples': [],
        'stop_confirmed': False, 'serial_closed': False,
        'reason': None, 'physical_wheel_observation': None,
        'feedforward_candidate': (list(CANDIDATE_FEEDFORWARD)
                                 if candidate_requested else None),
        'feedforward_applied': False, 'feedforward_restored': None,
        'pid_candidate': list(CANDIDATE_PID) if pid_candidate else None,
        'pid_applied': False, 'pid_restored': None,
        'profile_restored': None, 'configuration_acknowledgements': [],
        'limits': [
            'Arm flag is not an independent physical readiness check.',
            'One powered wheel pivots the robot; the other may roll passively.',
            'Encoder sign does not independently verify physical direction.',
            'Pi flags do not measure motor-branch voltage/current.',
            'A short transient response is not PID tuning or geometry acceptance.',
            'Encoder guards do not measure current or certify stall protection.',
            'Software timing/watchdog is not a physical emergency disconnect.',
            'PID gains affect both channels; only left is powered in the Kp trial.',
            'Restoration uses known baseline values, not unread runtime settings.',
        ],
    }
    connection = None
    started = time.monotonic()
    rows, lines = [], []
    baseline = None
    velocity_started = None
    initial_flags = None
    profile_may_have_changed = False
    pid_may_have_changed = False
    expected_config_ack = None
    known_candidate_acks = {'OK,FF,'+','.join(str(v) for v in CANDIDATE_FEEDFORWARD)}
    if pid_candidate:
        known_candidate_acks.add('OK,PID,'+','.join(str(v) for v in CANDIDATE_PID))
    exit_code = 2

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
        if line.startswith('OK,VEL,'):
            if line != expected_ack:
                raise RuntimeError('Unexpected velocity acknowledgement')
            result['velocity_acknowledgements'] += 1
            if guard is not None:
                guard.acknowledged = True
        if line.startswith(('OK,FF,', 'OK,PID,')):
            delayed = (phase == 'cleanup_stop' and profile_may_have_changed and
                       line in known_candidate_acks)
            if line != expected_config_ack and not delayed:
                raise RuntimeError('Unexpected configuration acknowledgement')
            # A delayed candidate ack in cleanup cannot confirm restoration.
            result['configuration_acknowledgements'].append({
                'elapsed_s': time.monotonic()-started, 'phase': phase,
                'acknowledgement': line, 'expected': line == expected_config_ack,
            })
        row = parse_telemetry(line)
        if row is None:
            return
        now = time.monotonic()
        if rows and counter_delta(row[0], rows[-1][0]) <= 0:
            raise RuntimeError('Controller clock restarted or repeated')
        rows.append(row)
        result['telemetry'].append({'elapsed_s': now-started,
                                    'phase': phase, 'values': row})
        if phase in CONFIGURATION_PHASES:
            if row[3:7] != [0, 0, 0, 0]:
                raise RuntimeError('Controller not stopped during profile change/cleanup')
        if guard is not None:
            guard.observe(row, now)

    def collect(seconds, phase):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            read_one(phase)

    def configure(prefix, values, phase):
        nonlocal expected_config_ack
        line = prefix+','+','.join(str(value) for value in values)
        expected_config_ack = 'OK,'+line
        first_line = len(lines)
        try:
            send((line+'\n').encode('ascii'))
            collect(.3, phase)
            if expected_config_ack not in lines[first_line:]:
                raise RuntimeError(prefix+' acknowledgement missing')
            if not resting(rows):
                raise RuntimeError('Controller not stationary after profile change')
        finally:
            expected_config_ack = None

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
        if candidate_requested:
            # Even a partial serial write may have changed the controller;
            # cleanup must attempt a stopped-only baseline restoration.
            profile_may_have_changed = True
            result['feedforward_restored'] = False
            result['profile_restored'] = False
            if pid_candidate:
                # PID also overwrites all FF values; restoration of both is
                # required even if the subsequent candidate FF step fails.
                pid_may_have_changed = True
                result['pid_restored'] = False
                configure('PID', CANDIDATE_PID, 'pid_candidate')
                result['pid_applied'] = True
            configure('FF', CANDIDATE_FEEDFORWARD, 'feedforward_candidate')
            result['feedforward_applied'] = True
        baseline = rows[-1]
        # This query runs while stopped, before the velocity clock/first VEL.
        # Allow the same 200ms setup budget as initial/final checks; retain
        # the 50ms timeout for every power query while VEL is active.
        check_power(timeout=.2)
        velocity_started = time.monotonic()
        guard = FeedbackGuard(baseline, velocity_started, args.velocity_channel)
        deadline = velocity_started+requested_seconds-STOP_MARGIN
        next_refresh = velocity_started
        next_power = velocity_started+.15
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                guard.check(now)
                if now >= next_refresh:
                    send(expected_command)
                    result['velocity_sent'] = True
                    next_refresh = time.monotonic()+REFRESH_SECONDS
                if now >= next_power and deadline-time.monotonic() > .06:
                    check_power()
                    next_power = time.monotonic()+.15
                if time.monotonic() >= deadline:
                    break
                read_one('velocity', guard)
        except Exception as error:
            result['reason'] = str(error)
        finally:
            send(b'STOP\n')
            result['velocity_to_stop_elapsed_s'] = time.monotonic()-velocity_started
        stop_start = len(lines)
        collect(.9, 'after_stop')
        result['stop_confirmed'] = ('OK,STOP' in lines[stop_start:] and resting(rows))
        if not result['stop_confirmed']:
            raise RuntimeError('Post-test STOP/stationary state not confirmed')
        result['encoder_delta'] = [counter_delta(rows[-1][i], baseline[i]) for i in (1, 2)]
        check_power(timeout=.2)
        result['final_adc'] = command(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'])
        exit_code = 0 if result['reason'] is None else 2
    except Exception as error:
        result['reason'] = str(error)
        exit_code = 2
    finally:
        if connection is not None:
            try:
                first_line = len(lines)
                send(b'STOP\n')
                if profile_may_have_changed:
                    collect(.3, 'cleanup_stop')
                    if 'OK,STOP' not in lines[first_line:] or not resting(rows):
                        raise RuntimeError('STOP/stationary state not confirmed for restoration')
                    if pid_may_have_changed:
                        configure('PID', BASELINE_PID, 'pid_restore')
                        result['pid_restored'] = True
                    configure('FF', BASELINE_FEEDFORWARD, 'feedforward_restore')
                    result['feedforward_restored'] = True
                    result['profile_restored'] = True
                    send(b'STOP\n')
            except Exception as error:
                result['cleanup_stop_error'] = str(error)
                if profile_may_have_changed:
                    result['feedforward_restore_error'] = str(error)
                    result['profile_restore_error'] = str(error)
                if pid_may_have_changed and not result['pid_restored']:
                    result['pid_restore_error'] = str(error)
                if result['reason'] is None:
                    result['reason'] = 'Cleanup failed: '+str(error)
                exit_code = 2
                try:
                    send(b'STOP\n')
                except Exception as last_error:
                    result['final_stop_error'] = str(last_error)
            connection.close()
            result['serial_closed'] = True
        result['elapsed_s'] = time.monotonic()-started
        print(json.dumps(result, indent=2))
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())

