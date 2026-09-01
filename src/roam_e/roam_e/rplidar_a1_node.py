#!/usr/bin/env python3
import math, time, threading
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar, RPLidarException

from .lidar_scan import bin_measurements, scan_geometry

class RPLidarA1Node(Node):
    def __init__(self):
        super().__init__('rplidar_a1_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('angle_increment_deg', 1.0)

        self.port = str(self.get_parameter('serial_port').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.angle_increment = math.radians(float(self.get_parameter('angle_increment_deg').value))

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_scan = self.create_publisher(LaserScan, 'scan', qos)

        self.num_bins, self.angle_min, self.angle_max = scan_geometry(self.angle_increment)

        self.lidar = None
        self._previous_scan_time_ns = None
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _connect(self):
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception:
                pass

        try:
            self.lidar = RPLidar(self.port, baudrate=115200, timeout=3)
            self.lidar.start_motor()
            self._previous_scan_time_ns = None
        except RPLidarException as e:
            self.get_logger().error(f"Error connecting to RPLidar: {e}")
            self.lidar = None

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                if not self.lidar:
                    self._connect()
                for scan in self.lidar.iter_scans(max_buf_meas=2000):
                    if self._stop.is_set():
                        break
                    now = self.get_clock().now()
                    scan_time = 0.0
                    if self._previous_scan_time_ns is not None:
                        scan_time = (now.nanoseconds - self._previous_scan_time_ns) / 1e9
                    self._previous_scan_time_ns = now.nanoseconds

                    ranges, intensities = bin_measurements(
                        scan,
                        num_bins=self.num_bins,
                        angle_increment=self.angle_increment,
                        range_min=self.range_min,
                        range_max=self.range_max,
                    )
                    msg = LaserScan()
                    msg.header.stamp = now.to_msg()
                    msg.header.frame_id = self.frame_id
                    msg.angle_min = self.angle_min
                    msg.angle_max = self.angle_max
                    msg.angle_increment = self.angle_increment
                    msg.time_increment = scan_time / max(self.num_bins - 1, 1)
                    msg.scan_time = scan_time
                    msg.range_min = self.range_min
                    msg.range_max = self.range_max
                    msg.ranges = ranges
                    msg.intensities = intensities
                    self.pub_scan.publish(msg)
            except RPLidarException as e:
                self.get_logger().warn(f'LiDAR error: {e}; reconnecting...')
                time.sleep(0.5)
                try:
                    self._connect()
                except Exception as ee:
                    self.get_logger().warn(f'Reconnect failed: {ee}')
                    time.sleep(1.0)
            except Exception as e:
                self.get_logger().warn(f'LiDAR unexpected error: {e}')
                time.sleep(0.2)

    def destroy_node(self):
        self._stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=2.0)
        if self.lidar:
            try:
                self.lidar.stop()
            except Exception:
                pass
            try:
                self.lidar.stop_motor()
            except Exception:
                pass
            try:
                self.lidar.disconnect()
            except Exception:
                pass
        super().destroy_node()

def main():
    rclpy.init()
    node = RPLidarA1Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
