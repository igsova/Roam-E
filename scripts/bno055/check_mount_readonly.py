#!/usr/bin/env python3
"""Sample BNO055 mounting evidence without writes, ROS or motor serial access.

Hold the robot still in a named pose before starting. This observes the device;
it does not physically isolate motor power, restore offsets or change axis maps.
SMBus register-pointer transactions are used only to read sensor output.
"""
import argparse
import json
import math
import statistics
import time


def signed16(low, high):
    value = low | (high << 8)
    return value - 65536 if value & 32768 else value


def vector(data, offset, scale):
    return [signed16(data[i], data[i + 1]) * scale
            for i in range(offset, offset + 6, 2)]


def summarize(rows):
    if len(rows) < 2:
        raise RuntimeError('At least two valid samples are required')
    return {
        'mean': [statistics.mean(row[i] for row in rows) for i in range(len(rows[0]))],
        'stddev': [statistics.stdev(row[i] for row in rows) for i in range(len(rows[0]))],
    }


def check_registers(registers):
    expected = {'0x00': '0xa0', '0x3b': '0x00', '0x3d': '0x08',
                '0x41': '0x24', '0x42': '0x00'}
    if any(registers[key] != value for key, value in expected.items()):
        raise RuntimeError('Unexpected chip, units, mode or axis map: ' + str(registers))


def capture(bus, address, duration, pose):
    def registers():
        return {f'0x{reg:02x}': f'0x{bus.read_byte_data(address, reg):02x}'
                for reg in (0x00, 0x35, 0x39, 0x3a, 0x3b, 0x3d, 0x41, 0x42)}

    before = registers()
    check_registers(before)
    gyro, acceleration, gravity, raw_acceleration, quaternion = [], [], [], [], []
    errors = 0
    started = time.monotonic()
    while time.monotonic() - started < duration:
        try:
            raw = bus.read_i2c_block_data(address, 0x08, 6)
            data = bus.read_i2c_block_data(address, 0x14, 32)
            if len(raw) != 6 or len(data) != 32:
                raise OSError('Incomplete sample')
            raw_acceleration.append(vector(raw, 0, .01))
            quaternion.append([signed16(data[i], data[i + 1]) / 16384
                               for i in range(12, 20, 2)])
            gyro.append(vector(data, 0, math.pi / (180 * 16)))
            acceleration.append(vector(data, 20, .01))
            gravity.append(vector(data, 26, .01))
        except OSError:
            errors += 1
        time.sleep(.05)
    after = registers()
    check_registers(after)
    return {'pose': pose, 'coordinate_frame': 'native_sensor',
            'samples': len(gravity), 'duration_s': time.monotonic() - started,
            'i2c_errors': errors, 'registers_before': before,
            'registers_after': after, 'gyro_rad_s': summarize(gyro),
            'raw_acceleration_m_s2': summarize(raw_acceleration),
            'quaternion_wxyz': summarize(quaternion),
            'linear_acceleration_m_s2': summarize(acceleration),
            'gravity_m_s2': summarize(gravity)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose', required=True,
                        choices=('level', 'caster-up', 'left-up'))
    parser.add_argument('--duration', type=float, default=5)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 30:
        parser.error('Duration must be 1–30 seconds')
    from smbus2 import SMBus
    with SMBus(3) as bus:
        print(json.dumps(capture(bus, 0x29, args.duration, args.pose), indent=2))


if __name__ == '__main__':
    main()
