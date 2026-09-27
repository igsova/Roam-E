#!/usr/bin/env python3
"""Isolated RGB-D baseline adapter. No hardware commands or TF publication.

Consumes existing JPEG RGB + firmware-registered metric depth. Does NOT open USB,
change camera parameters, or start motors/navigation. See RGBD_FIRMWARE_TEST.md.
"""
from collections import Counter, OrderedDict, deque
import copy
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import String

from firmware_rgbd_math import (camera_model, cloud_array, depth_array, nearest_depth,
                               rectification_maps, rectify_pair, stamp_ns)


class FirmwareRgbdTest(Node):
    def __init__(self):
        super().__init__('firmware_adapter', namespace='rgbd_test')
        cv2.setNumThreads(1)
        self.depths = deque(maxlen=32)
        self.colors = deque(maxlen=5)
        self.infos = OrderedDict()
        self.depth_info = None
        self.registration_ok = False
        self.param_future = None
        self.param_client = self.create_client(GetParameters, '/camera/driver/get_parameters')
        self.counts = Counter()
        self.deltas = deque(maxlen=150)
        self.ages = deque(maxlen=150)
        self.published_at = deque(maxlen=150)
        self.k = self.d = self.maps = None
        self.last_cloud = 0.0
        self.last_source_stamp = 0
        self.last_pair = {}
        self.started = time.monotonic()
        qos = qos_profile_sensor_data
        self.subs = [
            self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.color, qos),
            self.create_subscription(Image, '/camera/depth_raw/image', self.depth, qos),
            self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info, qos),
            self.create_subscription(CameraInfo, '/camera/depth_raw/camera_info', self.depth_metadata, qos),
        ]
        self.rgb_pub = self.create_publisher(Image, 'rgb/image_rect', qos)
        self.depth_pub = self.create_publisher(Image, 'depth/image_rect', qos)
        self.rgb_info_pub = self.create_publisher(CameraInfo, 'rgb/camera_info', qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, 'depth/camera_info', qos)
        self.cloud_pub = self.create_publisher(PointCloud2, 'points', qos)
        self.status_pub = self.create_publisher(String, 'status', 1)
        self.create_timer(0.02, self.process)
        self.create_timer(5.0, self.report)
        self.create_timer(2.0, self.check_registration)
        self.get_logger().warning(
            'EXPERIMENTAL: assumes firmware-registered depth shares raw RGB pixels. '
            'Joint RGB/depth rectification uses measured RGB K/D, not a new depth calibration. '
            'No TF, motors, USB configuration, or navigation changes.')

    def check_registration(self):
        if self.param_future is not None and not self.param_future.done():
            return
        if not self.param_client.service_is_ready():
            self.registration_ok = False
            return
        request = GetParameters.Request(names=['depth_registration'])
        self.param_future = self.param_client.call_async(request)
        self.param_future.add_done_callback(self.registration_response)

    def registration_response(self, future):
        try:
            values = future.result().values
            self.registration_ok = len(values) == 1 and values[0].type == 1 and values[0].bool_value
        except Exception as error:
            self.registration_ok = False
            self.get_logger().error(f'Registration check failed: {error}')

    def color(self, msg):
        if len(self.colors) == self.colors.maxlen:
            self.counts['rgb_queue_overflow'] += 1
        self.colors.append((stamp_ns(msg.header), msg, time.monotonic()))
        self.counts['rgb_received'] += 1

    def depth(self, msg):
        self.depths.append((stamp_ns(msg.header), msg))
        self.counts['depth_received'] += 1

    def info(self, msg):
        self.infos[stamp_ns(msg.header)] = msg
        while len(self.infos) > 32:
            self.infos.popitem(last=False)

    def depth_metadata(self, msg):
        self.depth_info = msg

    @staticmethod
    def image_msg(array, encoding, header):
        msg = Image()
        msg.header = copy.deepcopy(header)
        msg.height, msg.width = array.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = array.shape[1] * (3 if encoding == 'rgb8' else 2)
        msg.data = np.ascontiguousarray(array).tobytes()
        return msg

    def process(self):
        if not self.colors:
            return
        stamp, color, arrived = self.colors[0]
        now = self.get_clock().now().nanoseconds
        age = (now - stamp) / 1e9
        if age > .6 or age < -.05 or stamp <= self.last_source_stamp:
            self.colors.popleft()
            self.counts['stale_future_or_repeated_rgb'] += 1
            return
        # Wait briefly for exact-stamp RGB metadata and the next depth frame,
        # then choose the nearest header instead of always an older frame.
        waited = time.monotonic() - arrived
        if waited < .18 and (stamp not in self.infos or not self.depths
                            or max(s for s, _ in self.depths) < stamp + 25_000_000):
            return
        self.colors.popleft()
        if not self.registration_ok:
            self.counts['registration_not_confirmed'] += 1
            return
        info = self.infos.get(stamp)
        pair = nearest_depth(stamp, self.depths, 25_000_000)
        if info is None or pair is None:
            self.counts['missing_exact_rgb_info_or_depth_pair'] += 1
            return
        depth_stamp, depth = pair
        if not -.05 <= (now - depth_stamp)/1e9 <= .6:
            self.counts['stale_depth'] += 1
            return
        try:
            k, d = camera_model(info)
            frame = info.header.frame_id
            di = self.depth_info
            if (color.header.frame_id != frame or depth.header.frame_id != frame
                    or di is None or di.header.frame_id != frame
                    or (di.width, di.height) != (640, 480)
                    or abs(now - stamp_ns(di.header)) > 600_000_000):
                raise ValueError('Missing/stale/incompatible source frames or depth metadata')
            if self.k is None:
                self.k, self.d = k, d
                self.maps = rectification_maps(k, d)
            if not np.allclose(k, self.k) or not np.allclose(d, self.d):
                raise ValueError('RGB calibration changed during experiment; restart test')
            bgr = cv2.imdecode(np.frombuffer(color.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is None or bgr.shape != (480, 640, 3):
                raise ValueError('Invalid RGB JPEG/dimensions')
            rgb, depth_mm = rectify_pair(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), depth_array(depth), self.maps)
        except (ValueError, TypeError, cv2.error) as error:
            self.counts['invalid_pair'] += 1
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return
        # Derived pair uses RGB stamp for exact downstream message sync. The
        # ORIGINAL depth delta is retained in status. This is NOT hardware sync.
        header = color.header
        rect_info = copy.deepcopy(info)
        rect_info.header = copy.deepcopy(header)
        rect_info.d = [0.0] * 5
        rect_info.r = np.eye(3).ravel().tolist()
        rect_info.p = np.column_stack((k, np.zeros(3))).ravel().tolist()
        self.rgb_pub.publish(self.image_msg(rgb, 'rgb8', header))
        self.depth_pub.publish(self.image_msg(depth_mm.astype('<u2'), '16UC1', header))
        self.rgb_info_pub.publish(rect_info)
        self.depth_info_pub.publish(rect_info)
        self.last_source_stamp = stamp
        self.counts['pairs_published'] += 1
        self.deltas.append((depth_stamp - stamp) / 1e6)
        self.ages.append(age * 1000)
        self.published_at.append(time.monotonic())
        self.last_pair = {'rgb_source_stamp_ns': stamp, 'depth_source_stamp_ns': depth_stamp,
                          'valid_depth_percent': float(np.count_nonzero(depth_mm)/depth_mm.size*100)}
        if time.monotonic() - self.last_cloud >= 1.0:
            points = cloud_array(rgb, depth_mm, k)
            cloud = PointCloud2()
            cloud.header = copy.deepcopy(header)
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                            for name, offset in [('x', 0), ('y', 4), ('z', 8), ('rgb', 12)]]
            cloud.is_bigendian, cloud.is_dense = False, True
            cloud.point_step, cloud.row_step = 16, 16 * len(points)
            cloud.data = points.tobytes()
            self.cloud_pub.publish(cloud)
            self.last_cloud = time.monotonic()
            self.counts['clouds_published'] += 1
            self.last_pair.update(cloud_points=len(points), cloud_bytes=len(cloud.data))

    def report(self):
        def stats(values):
            return {'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
                    'min': float(min(values)), 'max': float(max(values))} if values else None
        ts = self.published_at
        result = {
            'experimental': True, 'registration_parameter_confirmed': self.registration_ok,
            'assumption': 'firmware depth and raw RGB coindexed; jointly rectified with RGB K/D',
            'hardware_synchronized': False, 'counts': dict(self.counts),
            'pair_hz': (len(ts)-1)/(ts[-1]-ts[0]) if len(ts) > 1 else 0,
            'last_pair_age_s': time.monotonic()-ts[-1] if ts else None,
            'source_depth_minus_rgb_ms': stats(self.deltas),
            'rgb_header_age_at_pair_ms': stats(self.ages), 'last_pair': self.last_pair,
        }
        text = json.dumps(result, allow_nan=False)
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = FirmwareRgbdTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
