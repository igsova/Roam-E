"""Astra Pro colour camera: native UVC MJPEG passthrough with CameraInfo.

Streams the camera's own JPEG frames without re-encoding (compressed topic),
optionally decodes a reduced-rate raw image for consumers such as RTAB-Map, and
publishes the calibrated CameraInfo from config/astra_rgb_640x480.yaml.
Timestamps are host receive times, not hardware exposure times.
"""

import threading
import time
import math

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from roam_e.rgb_calibration import load_rgb_calibration, camera_info_for_frame
from roam_e.uvc_capture import UvcCapture


def jpeg_bytes(frame):
    data = frame if isinstance(frame, bytes) else frame.tobytes()
    if not data.startswith(b'\xff\xd8') or b'\xff\xd9' not in data:
        raise ValueError('Camera returned an incomplete or non-JPEG frame')
    return data


def decode_rgb(data):
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Invalid JPEG frame')
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


class MjpegCamera(Node):
    def __init__(self):
        super().__init__('color_camera')
        for key, value in [
            ('width', 640),
            ('height', 480),
            ('capture_fps', 30.0),
            ('preview_fps', 5.0),
            ('decode_fps', 2.0),
            ('usb_payload_bytes', 800),
            ('camera_frame_id', 'camera_rgb_optical_frame'),
            ('calibration_file', ''),
        ]:
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))

        def p(name):
            return self.get_parameter(name).value

        self.preview_fps, self.decode_fps = p('preview_fps'), p('decode_fps')
        if (
            not all(math.isfinite(v) for v in [self.preview_fps, self.decode_fps, p('capture_fps')])
            or self.preview_fps <= 0
            or self.decode_fps < 0
            or p('capture_fps') <= 0
        ):
            raise ValueError('Rates must be positive; decode_fps may be zero')
        self.frame_id = p('camera_frame_id')
        self.width, self.height = p('width'), p('height')
        self.calibration = (
            load_rgb_calibration(p('calibration_file'), (self.width, self.height))
            if p('calibration_file')
            else None
        )
        self.info = (
            self.create_publisher(CameraInfo, 'camera_info', qos_profile_sensor_data)
            if self.calibration is not None
            else None
        )
        self.compressed = self.create_publisher(
            CompressedImage, 'image_raw/compressed', qos_profile_sensor_data
        )
        self.raw = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        self.cap = UvcCapture(p('width'), p('height'), p('capture_fps'), p('usb_payload_bytes'))
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.stop = threading.Event()
        self.last_preview = self.last_decode = 0.0
        self.captured = self.invalid = 0
        self.worker = threading.Thread(target=self.capture, daemon=True)
        self.worker.start()
        self.create_timer(0.01, self.publish_latest)
        self.create_timer(30.0, self.report_capture)
        if self.calibration is not None:
            self.get_logger().info(
                f'RGB calibration loaded: {p("calibration_file")}; '
                'CameraInfo shares image stamps; depth alignment is unverified'
            )
        else:
            self.get_logger().warning('No RGB calibration file; CameraInfo will not be published')
        self.get_logger().info(
            'Native MJPEG capture; host receive timestamps, not exposure synchronization'
        )

    def capture(self):
        last_frame = time.monotonic()
        try:
            while not self.stop.is_set():
                frame = self.cap.read()
                if frame is None:
                    if time.monotonic() - last_frame > 5:
                        raise RuntimeError('No RGB frame for 5 seconds; check USB connection')
                    continue
                if self.stop.is_set():
                    break
                stamp = self.get_clock().now().to_msg()
                try:
                    data = jpeg_bytes(frame)
                except ValueError:
                    self.invalid += 1
                    continue
                self.captured += 1
                last_frame = time.monotonic()
                with self.lock:
                    self.latest = (stamp, data)
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.cap.release()

    def publish_latest(self):
        if self.error:
            self.get_logger().error(self.error)
            raise RuntimeError(self.error)
        with self.lock:
            item, self.latest = self.latest, None
        if item is None:
            return
        stamp, data = item
        now = time.monotonic()
        published = False
        if now - self.last_preview >= 1.0 / self.preview_fps:
            msg = CompressedImage()
            msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
            msg.format, msg.data = 'jpeg', data
            self.compressed.publish(msg)
            self.last_preview = now
            published = True
        if (
            self.decode_fps > 0
            and self.raw.get_subscription_count() > 0
            and now - self.last_decode >= 1.0 / self.decode_fps
        ):
            try:
                rgb = decode_rgb(data)
            except ValueError:
                self.invalid += 1
            else:
                if rgb.shape[:2] != (self.height, self.width):
                    raise RuntimeError('Decoded RGB dimensions do not match capture/calibration')
                msg = Image()
                msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
                msg.height, msg.width = rgb.shape[:2]
                msg.encoding, msg.step = 'rgb8', rgb.shape[1] * 3
                msg.data = rgb.tobytes()
                self.raw.publish(msg)
                self.last_decode = now
                published = True
        # Raw decoding and JPEG preview may have different schedules. Publish
        # once for their union, so neither stream misses matching metadata.
        if published and self.info is not None:
            self.info.publish(camera_info_for_frame(self.calibration, stamp, self.frame_id))

    def report_capture(self):
        self.get_logger().info(
            f'Capture totals: {self.captured} JPEG frames, {self.invalid} invalid frames skipped'
        )

    def close(self):
        self.stop.set()
        self.worker.join(timeout=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MjpegCamera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
