"""Shift the Astra's registered depth image onto the UVC colour image.

The Astra registers depth to its own model of the RGB sensor, but the UVC
colour stream we use lands about 7 px to the left of it (measured 16 Oct
2026 with tools/calibration/measure_depth_color_offset.py, roughly constant from
0.7 to 2.4 m). RTAB-Map takes each colour feature's 3D position from the
depth pixel under it, so this offset breaks its visual matching. Pixels
shifted in from outside the image are zero (invalid). Work happens only
while someone subscribes.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def shift_image(image: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """Move content by (dx, dy) pixels; uncovered pixels become 0."""
    out = np.zeros_like(image)
    h, w = image.shape[:2]
    src_y = slice(max(0, -dy), min(h, h - dy))
    dst_y = slice(max(0, dy), min(h, h + dy))
    src_x = slice(max(0, -dx), min(w, w - dx))
    dst_x = slice(max(0, dx), min(w, w + dx))
    out[dst_y, dst_x] = image[src_y, src_x]
    return out


class DepthAlignNode(Node):
    def __init__(self):
        super().__init__('depth_align')
        self.declare_parameter('input_topic', '/camera/depth_raw/image')
        self.declare_parameter('output_topic', '/camera/depth_aligned/image')
        self.declare_parameter('shift_x_px', -7)
        self.declare_parameter('shift_y_px', -1)
        self._dx = int(self.get_parameter('shift_x_px').value)
        self._dy = int(self.get_parameter('shift_y_px').value)
        self._pub = self.create_publisher(
            Image, str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('input_topic').value),
            self._on_depth, qos_profile_sensor_data)

    def _on_depth(self, msg: Image):
        if self._pub.get_subscription_count() == 0:
            return
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().warning(
                f'unsupported depth encoding {msg.encoding!r}',
                throttle_duration_sec=10.0)
            return
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(
            msg.height, msg.step // 2)[:, :msg.width]
        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding, out.is_bigendian = msg.encoding, msg.is_bigendian
        out.step = msg.width * 2
        out.data = shift_image(depth, self._dx, self._dy).tobytes()
        self._pub.publish(out)


def main():
    rclpy.init()
    node = DepthAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
