"""Colourized, compressed preview of the depth image for Foxglove.

Maps depth in millimetres to a colour ramp between min_depth_m and max_depth_m,
optionally downsamples it, and publishes JPEG frames at a capped rate so the
preview can be watched over Wi-Fi without streaming raw depth.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def colorize_depth(
    image: Image,
    min_depth_mm: int,
    max_depth_mm: int,
    downsample: int,
) -> np.ndarray:
    """Convert a 16-bit millimetre depth image to a compact BGR preview."""
    if image.encoding not in ('16UC1', 'mono16'):
        raise ValueError(
            f'unsupported depth encoding {image.encoding!r}; expected 16UC1'
        )
    if image.step % 2:
        raise ValueError(f'invalid 16-bit image step: {image.step}')

    dtype = np.dtype('>u2' if image.is_bigendian else '<u2')
    row_width = image.step // dtype.itemsize
    expected_values = image.height * row_width
    depth = np.frombuffer(image.data, dtype=dtype, count=expected_values)
    depth = depth.reshape(image.height, row_width)[:, :image.width]
    depth = depth[::downsample, ::downsample]

    valid = (depth >= min_depth_mm) & (depth <= max_depth_mm)
    clipped = np.clip(depth, min_depth_mm, max_depth_mm)
    scale = 255.0 / float(max_depth_mm - min_depth_mm)
    intensity = ((max_depth_mm - clipped) * scale).astype(np.uint8)
    intensity[~valid] = 0

    color = cv2.applyColorMap(intensity, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


class DepthPreviewNode(Node):
    def __init__(self):
        super().__init__('depth_preview')
        self.declare_parameter('input_topic', '/camera/depth_raw/image')
        self.declare_parameter(
            'output_topic', '/camera/depth_preview/compressed'
        )
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('downsample', 2)
        self.declare_parameter('jpeg_quality', 70)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._min_depth_mm = int(
            float(self.get_parameter('min_depth_m').value) * 1000.0
        )
        self._max_depth_mm = int(
            float(self.get_parameter('max_depth_m').value) * 1000.0
        )
        self._publish_rate = float(self.get_parameter('publish_rate').value)
        self._downsample = int(self.get_parameter('downsample').value)
        self._jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        if self._max_depth_mm <= self._min_depth_mm:
            raise ValueError('max_depth_m must be greater than min_depth_m')
        if self._publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if self._downsample < 1:
            raise ValueError('downsample must be at least one')
        if not 1 <= self._jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 1 and 100')

        self._minimum_period = 1.0 / self._publish_rate
        self._last_publish_time = 0.0
        self._publisher = self.create_publisher(
            CompressedImage, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            Image,
            input_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Publishing {output_topic} at up to {self._publish_rate:.1f} Hz '
            f'from {input_topic} (1/{self._downsample} scale)'
        )

    def _on_depth(self, image: Image) -> None:
        now = time.monotonic()
        if now - self._last_publish_time < self._minimum_period:
            return

        try:
            preview = colorize_depth(
                image,
                self._min_depth_mm,
                self._max_depth_mm,
                self._downsample,
            )
            ok, encoded = cv2.imencode(
                '.jpg',
                preview,
                [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
            )
            if not ok:
                raise ValueError('OpenCV failed to encode the depth preview')
        except (ValueError, cv2.error) as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return

        message = CompressedImage()
        message.header = image.header
        message.format = 'jpeg'
        message.data = encoded.tobytes()
        self._publisher.publish(message)
        self._last_publish_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DepthPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
