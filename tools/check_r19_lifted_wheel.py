#!/usr/bin/env python3
"""One explicitly armed, supervised lifted-wheel pulse; never a drive launch.

Default: STOP/INFO/telemetry only. Motion requires --pulse-channel and
--arm-lifted-wheels after human confirmation of a secure support and reachable
motor-power disconnect. One fixed PWM=40 command for 0.3 s, never refreshed.
Firmware's published 350 ms watchdog is a fallback, not hardware isolation.
No VEL/PID/FF/ZERO, firmware flash, ROS publications or calibration writes.
"""
import argparse
import json
import re
import signal
import subprocess
import time

PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
EXPECTED_INFO = 'INFO,2,748,100,2000'
PULSE_PWM = 40
PULSE_SECONDS = 0.3


def pulse_command(channel, armed):
    if not armed or channel not in ('left', 'right'):
        raise ValueError('Explicit lifted-wheel arm and one channel required')
    return b'PWM,40,0\n' if channel == 'left' else b'PWM,0,40\n'


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
    if any(not -100 <= x <= 100 for x in values[3:5]):
        raise ValueError('Unexpected PWM outside published ceiling')
    return values


def resting(rows):
    # PWM + velocity targets zero; stationary encoder counts over >=0.2 s.
    return (len(rows) >= 5 and all(r[3:7] == [0, 0, 0, 0] for r in rows[-5:])
            and (rows[-1][0] - rows[-5][0]) % 2**32 >= 200
            and all(r[1:3] == rows[-1][1:3] for r in rows[-5:]))


def counter_delta(current, previous):
    return (current - previous + 2**31) % 2**32 - 2**31


def command(args, timeout=2):
    return subprocess.run(args, text=True, capture_output=True,
                          timeout=timeout, check=True).stdout.strip()


def power_status():
    flags = command(['vcgencmd', 'get_throttled'], timeout=.2)
    if not re.fullmatch(r'throttled=0x[0-9a-fA-F]+', flags):
        raise RuntimeError('Unrecognised Pi power flags')
    return int(flags.split('=')[1], 16)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pulse-channel', choices=['left', 'right'])
    parser.add_argument('--arm-lifted-wheels', action='store_true')
    args = parser.parse_args()
    if args.pulse_channel and not args.arm_lifted_wheels:
        parser.error('Motion requires --arm-lifted-wheels and human readiness')
    if args.arm_lifted_wheels and not args.pulse_channel:
        parser.error('Arm requires exactly one selected channel')
    result = {'port': PORT, 'controller_info': None, 'pulse_sent': False,
              'requested_channel': args.pulse_channel, 'pwm': PULSE_PWM,
              'requested_pulse_seconds': PULSE_SECONDS, 'commands': [],
              'power_samples': [], 'telemetry': [], 'reason': None,
              'stop_confirmed': False, 'physical_wheel_observation': None,
              'limits': ['Only firmware channel identity, not physical side/direction.',
                         'INFO does not independently verify flashed watchdog code.',
                         'Software STOP is not a physical emergency disconnect.']}
    connection = None
    started = time.monotonic()
    lines = []
    rows = []

    def check_power():
        flags = power_status()
        result['power_samples'].append({'elapsed_s': time.monotonic()-started,
                                        'throttled': hex(flags)})
        if flags & 0x5:
            raise RuntimeError('Active Pi undervoltage/throttling; refusing/aborting pulse')
        return flags

    def send(data):
        written = connection.write(data)
        if written != len(data):
            raise RuntimeError('Incomplete serial write')
        result['commands'].append({'elapsed_s': time.monotonic()-started,
                                   'command': data.decode('ascii').strip()})

    def collect(seconds, phase, monitor=False):
        until = time.monotonic()+seconds
        next_power = time.monotonic()
        while time.monotonic() < until:
            if monitor and time.monotonic() >= next_power:
                check_power()
                next_power = time.monotonic()+.05
                if time.monotonic() >= until:
                    break
            raw = connection.readline(128)
            if not raw:
                continue
            if not raw.endswith(b'\n'):
                raise RuntimeError('Partial/overlong serial line')
            line = raw.decode('ascii').strip()
            lines.append(line)
            if line.startswith('ERR,') or line.startswith('EVENT,'):
                raise RuntimeError('Controller event: '+line)
            if line.startswith('INFO,'):
                result['controller_info'] = line
            row = parse_telemetry(line)
            if row is not None:
                if rows and row[0] < rows[-1][0] and rows[-1][0] < 0xF0000000:
                    raise RuntimeError('Controller restarted during check')
                rows.append(row)
                result['telemetry'].append({'elapsed_s': time.monotonic()-started,
                                            'phase': phase, 'values': row})

    def interrupted(signum, frame):
        raise RuntimeError('Interrupted; stopping controller')

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    try:
        identity = command(['udevadm', 'info', '--query=property', '--name='+PORT])
        if 'ID_VENDOR_ID=1a86\n' not in identity+'\n' or 'ID_MODEL_ID=7523\n' not in identity+'\n':
            raise RuntimeError('Unexpected serial USB identity')
        owner = subprocess.run(['fuser', PORT], text=True, capture_output=True, timeout=2)
        if owner.returncode != 1 or owner.stdout.strip() or owner.stderr.strip():
            raise RuntimeError('Port occupied or occupancy check failed')
        initial_flags = check_power()
        result['initial_adc'] = command(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'])
        import serial
        connection = serial.Serial(PORT, 115200, timeout=.02,
                                   write_timeout=.1, exclusive=True)
        # Opening Nano USB can reset it; let its published stopped boot settle.
        time.sleep(2)
        connection.reset_input_buffer()
        send(b'STOP\n')
        send(b'INFO\n')
        collect(.7, 'preflight')
        if result['controller_info'] != EXPECTED_INFO or 'OK,STOP' not in lines:
            raise RuntimeError('Controller INFO/STOP not verified')
        if not resting(rows):
            raise RuntimeError('Controller not stationary and stopped before pulse')
        baseline = rows[-1]
        if args.pulse_channel:
            check_power()
            send(pulse_command(args.pulse_channel, args.arm_lifted_wheels))
            result['pulse_sent'] = True
            try:
                collect(PULSE_SECONDS, 'pulse', monitor=True)
            finally:
                send(b'STOP\n')
        else:
            send(b'STOP\n')
        stop_line_start = len(lines)
        collect(.9, 'after_stop', monitor=True)
        result['stop_confirmed'] = 'OK,STOP' in lines[stop_line_start:] and resting(rows)
        if not result['stop_confirmed']:
            raise RuntimeError('Post-test STOP/stationary telemetry not confirmed')
        result['encoder_delta'] = [counter_delta(rows[-1][i], baseline[i]) for i in (1,2)]
        final_flags = check_power()
        result['final_adc'] = command(['vcgencmd', 'pmic_read_adc', 'EXT5V_V'])
        if (final_flags & ~initial_flags) & 0x50000:
            raise RuntimeError('New historical Pi undervoltage/throttling flag during check')
        return 0
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
        result['elapsed_s'] = time.monotonic()-started
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    raise SystemExit(main())
