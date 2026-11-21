#!/usr/bin/env python3
"""Check the IMU axis convention through ROS while you move the robot by hand.

Subscribes to /imu/data and guides you through three moves: nose up, left side
up, and a counter-clockwise turn. It prints the roll, pitch and yaw change for
each, to compare with REP-103 (nose up is negative pitch, left side up is
positive roll, counter-clockwise is positive yaw). Run it with the sensors
launch running; it never commands motion.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def quaternion_to_euler(x, y, z, w):
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_term = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_term)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def angle_difference(final, initial):
    return (final - initial + 180.0) % 360.0 - 180.0


class AxisTestNode(Node):
    def __init__(self):
        super().__init__('imu_axis_test')
        self.lock = threading.Lock()
        self.samples = []
        self.collecting = False
        self.received_message = threading.Event()
        self.create_subscription(
            Imu,
            '/imu/data',
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message):
        self.received_message.set()

        with self.lock:
            if not self.collecting:
                return

            self.samples.append(
                {
                    'gyro': (
                        message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z,
                    ),
                    'quaternion': (
                        message.orientation.x,
                        message.orientation.y,
                        message.orientation.z,
                        message.orientation.w,
                    ),
                }
            )

    def collect(self, name, instruction, duration=4.0):
        print()
        print(f'=== {name} ===')
        print(instruction)
        input('Place your hands ready, then press Enter for the countdown. ')

        for value in (3, 2, 1):
            print(value, flush=True)
            time.sleep(1.0)

        with self.lock:
            self.samples = []
            self.collecting = True

        print('GO - move smoothly now, then HOLD that position.', flush=True)
        time.sleep(duration)

        with self.lock:
            self.collecting = False
            samples = list(self.samples)

        if len(samples) < 10:
            raise RuntimeError(
                f'Only {len(samples)} messages were received during {name}'
            )

        axis_names = ('X', 'Y', 'Z')
        peaks = []

        for axis in range(3):
            values = [sample['gyro'][axis] for sample in samples]
            peaks.append(max(values, key=abs))

        dominant_axis = max(range(3), key=lambda axis: abs(peaks[axis]))

        start_q = samples[0]['quaternion']
        end_q = samples[-1]['quaternion']
        start_euler = quaternion_to_euler(*start_q)
        end_euler = quaternion_to_euler(*end_q)
        delta_euler = tuple(
            angle_difference(end_euler[index], start_euler[index])
            for index in range(3)
        )

        print(f'Samples: {len(samples)}')
        print(
            'Signed gyro peaks (rad/s): '
            f'X={peaks[0]:+.4f}  '
            f'Y={peaks[1]:+.4f}  '
            f'Z={peaks[2]:+.4f}'
        )
        print(
            f'Dominant sensor gyro axis: {axis_names[dominant_axis]} '
            f'({peaks[dominant_axis]:+.4f} rad/s)'
        )
        print(
            'Native quaternion at start: '
            f'x={start_q[0]:+.5f} y={start_q[1]:+.5f} '
            f'z={start_q[2]:+.5f} w={start_q[3]:+.5f}'
        )
        print(
            'Native quaternion at end:   '
            f'x={end_q[0]:+.5f} y={end_q[1]:+.5f} '
            f'z={end_q[2]:+.5f} w={end_q[3]:+.5f}'
        )
        print(
            'Standard Euler change from native quaternion: '
            f'roll={delta_euler[0]:+.2f} deg  '
            f'pitch={delta_euler[1]:+.2f} deg  '
            f'yaw={delta_euler[2]:+.2f} deg'
        )


def main():
    rclpy.init()
    node = AxisTestNode()

    def spin_node():
        try:
            rclpy.spin(node)
        except Exception as error:
            if rclpy.ok():
                print(f'ROS spin failed: {error}')

    spin_thread = threading.Thread(
        target=spin_node,
        daemon=True,
    )
    spin_thread.start()

    try:
        print('Waiting for /imu/data...')

        if not node.received_message.wait(timeout=5.0):
            raise RuntimeError('No /imu/data messages received')

        print('IMU stream detected.')
        print('Keep the motors and LiDAR stopped during this test.')

        node.collect(
            'NOSE UP',
            'Smoothly raise the FRONT of the robot by about 20-30 degrees '
            'and hold it there. Do not lower it until collection finishes.',
        )
        input(
            '\nReturn the robot to its original level position, let it '
            'settle, then press Enter. '
        )

        node.collect(
            'LEFT SIDE UP',
            'Smoothly raise the LEFT side of the robot by about 20-30 '
            'degrees and hold it there.',
        )
        input(
            '\nReturn the robot to its original level position, let it '
            'settle, then press Enter. '
        )

        node.collect(
            'COUNTER-CLOCKWISE YAW',
            'Viewed from above, smoothly rotate the whole robot '
            'COUNTER-CLOCKWISE by about 60 degrees and hold it there.',
        )

        print()
        print('AXIS TEST COMPLETE')
    except KeyboardInterrupt:
        print('\nCancelled.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()

