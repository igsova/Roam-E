"""Light depth-image to PointCloud2 converter for Nav2 obstacle layers.

Publishes a decimated, range-limited XYZ cloud from the registered Astra depth
image at a capped rate, in the image's own optical frame. Nav2's voxel layer
transforms it, filters by height and marks/clears 3D obstacles. Work happens
only while someone subscribes. Read-only: no camera parameters are changed.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def depth_to_points(
    depth_mm: np.ndarray,
    k: list,
    stride: int,
    min_range_m: float,
    max_range_m: float,
) -> np.ndarray:
    """Back-project every `stride`-th pixel to optical-frame XYZ (metres).

    Pixels outside [min_range_m, max_range_m] or zero (invalid) are dropped.
    Returns an (N, 3) float32 array.
    """
    fx, cx, fy, cy = float(k[0]), float(k[2]), float(k[4]), float(k[5])
    sub = depth_mm[::stride, ::stride].astype(np.float32) / 1000.0
    rows, cols = np.nonzero((sub >= min_range_m) & (sub <= max_range_m))
    z = sub[rows, cols]
    u = cols.astype(np.float32) * stride
    v = rows.astype(np.float32) * stride
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.column_stack((x, y, z)).astype(np.float32)


def depth_array(image: Image) -> np.ndarray:
    if image.encoding not in ('16UC1', 'mono16'):
        raise ValueError(f'unsupported depth encoding {image.encoding!r}')
    dtype = np.dtype('>u2' if image.is_bigendian else '<u2')
    row_width = image.step // dtype.itemsize
    depth = np.frombuffer(image.data, dtype=dtype, count=image.height * row_width)
    return depth.reshape(image.height, row_width)[:, :image.width]


def make_cloud(points: np.ndarray, header) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    msg.data = points.astype('<f4').tobytes()
    return msg


class DepthObstacleCloudNode(Node):
    def __init__(self):
        super().__init__('depth_obstacle_cloud')
        self.declare_parameter('depth_topic', '/camera/depth_raw/image')
        self.declare_parameter('info_topic', '/camera/depth_raw/camera_info')
        self.declare_parameter('output_topic', '/camera/obstacle_points')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('stride', 4)
        self.declare_parameter('min_range_m', 0.45)
        self.declare_parameter('max_range_m', 3.0)
        self._period = 1.0 / float(self.get_parameter('publish_rate').value)
        self._stride = int(self.get_parameter('stride').value)
        self._min = float(self.get_parameter('min_range_m').value)
        self._max = float(self.get_parameter('max_range_m').value)
        self._k = None
        self._last = 0.0
        self._pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_topic').value), 2)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('info_topic').value),
            self._on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self._on_depth, qos_profile_sensor_data)

    def _on_info(self, msg: CameraInfo):
        if msg.k[0] > 0:
            self._k = list(msg.k)

    def _on_depth(self, msg: Image):
        if self._k is None or self._pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now
        try:
            points = depth_to_points(
                depth_array(msg), self._k, self._stride, self._min, self._max)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=10.0)
            return
        self._pub.publish(make_cloud(points, msg.header))


def main():
    rclpy.init()
    node = DepthObstacleCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
