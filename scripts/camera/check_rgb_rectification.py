#!/usr/bin/env python3
"""Bounded, read-only ROS check of RGB rectification with concurrent depth.

Does not open USB, alter parameters, publish commands, or launch motors. An
optional new output directory receives one raw/rectified pair plus metrics.
This verifies the processing pipeline, not physical RGB-depth registration.
"""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def stamp_key(message):
    stamp = message.header.stamp
    return stamp.sec, stamp.nanosec


def rgb_array(message):
    if message.encoding != 'rgb8' or (message.width, message.height) != (640, 480):
        raise ValueError('Expected rgb8 640x480')
    if message.step < message.width * 3 or len(message.data) != message.step * message.height:
        raise ValueError('Invalid image row stride/data length')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    return rows[:, :message.width*3].reshape(message.height, message.width, 3)


class RectificationCheck(Node):
    def __init__(self, output_dir):
        super().__init__('roam_rgb_rectification_check')
        self.output_dir = output_dir
        self.saved_pair = False
        self.buffers = {name: OrderedDict() for name in ('raw', 'rect', 'info')}
        self.done = set()
        self.errors = set()
        self.times = {name: [] for name in ('raw', 'rect', 'info', 'jpeg', 'depth')}
        self.ages = []
        self.comparisons = []
        self.maps = None
        self.intrinsics = None
        self.subscriptions_kept = []
        for name, topic, message_type in (
                ('raw', '/camera/color/image_raw', Image),
                ('rect', '/camera/color/image_rect', Image),
                ('info', '/camera/color/camera_info', CameraInfo),
                ('jpeg', '/camera/color/image_rect/compressed', CompressedImage),
                ('depth', '/camera/depth_raw/image', Image)):
            self.subscriptions_kept.append(self.create_subscription(
                message_type, topic, lambda msg, name=name: self.accept(name, msg),
                qos_profile_sensor_data))

    def accept(self, name, message):
        self.times[name].append(time.monotonic())
        try:
            if name == 'depth':
                if (message.width, message.height, message.encoding) != (640, 480, '16UC1'):
                    raise ValueError('Depth is not 640x480 16UC1')
                return
            if name == 'jpeg':
                jpeg = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
                if jpeg is None or jpeg.shape != (480, 640, 3):
                    raise ValueError('Rectified JPEG is not decodable 640x480')
                return
            key = stamp_key(message)
            self.buffers[name][key] = message
            if len(self.buffers[name]) > 64:
                self.buffers[name].popitem(last=False)
            if name == 'info':
                if ((message.width, message.height) != (640, 480)
                        or message.distortion_model != 'plumb_bob'
                        or message.header.frame_id != 'camera_rgb_optical_frame'):
                    raise ValueError('Unexpected RGB CameraInfo dimensions/model/frame')
                signature = tuple(message.k) + tuple(message.d) + tuple(message.r) + tuple(message.p)
                if self.intrinsics is None:
                    self.intrinsics = signature
                elif self.intrinsics != signature:
                    raise ValueError('Calibration changed during the check')
            if key in self.done or not all(key in b for b in self.buffers.values()):
                return
            raw, rect, info = (self.buffers[n][key] for n in ('raw', 'rect', 'info'))
            if not (raw.header == rect.header == info.header):
                raise ValueError('Raw/rectified/CameraInfo headers do not match exactly')
            if self.maps is None:
                self.maps = cv2.initUndistortRectifyMap(
                    np.array(info.k).reshape(3, 3), np.array(info.d),
                    np.array(info.r).reshape(3, 3), np.array(info.p).reshape(3, 4)[:, :3],
                    (640, 480), cv2.CV_32FC1)
            source, actual = rgb_array(raw), rgb_array(rect)
            expected = cv2.remap(source, *self.maps, cv2.INTER_LINEAR)
            error = np.abs(actual.astype(np.int16) - expected.astype(np.int16))
            changed = np.mean(np.abs(actual.astype(np.int16) - source.astype(np.int16)))
            self.comparisons.append({'mean_error': float(error.mean()),
                                     'max_error': int(error.max()),
                                     'mean_change_from_raw': float(changed)})
            self.ages.append(self.get_clock().now().nanoseconds/1e9 - key[0] - key[1]/1e9)
            self.done.add(key)
            if self.output_dir and not self.saved_pair:
                for label, array in (('raw', source), ('rectified', actual)):
                    if not cv2.imwrite(str(self.output_dir / f'{label}.png'),
                                       cv2.cvtColor(array, cv2.COLOR_RGB2BGR)):
                        raise ValueError('Failed to save inspection image')
                self.saved_pair = True
        except (ValueError, cv2.error) as error:
            self.errors.add(str(error))

    def report(self):
        rates = {name: ((len(t)-1)/(t[-1]-t[0]) if len(t)>1 else 0.)
                 for name, t in self.times.items()}
        # Discovery and delivery order may leave a boundary frame unmatched.
        matched = len(self.done)
        passed = (not self.errors and matched >= 8
                  and matched >= 0.9*len(self.times['rect'])
                  and len(self.times['jpeg']) >= 8 and rates['depth'] > 15
                  and max((c['mean_error'] for c in self.comparisons), default=999) < 0.1
                  and max((c['mean_change_from_raw'] for c in self.comparisons), default=0) > 0.01)
        return {'passed': bool(passed), 'scope': 'RGB processing only; RGB-depth alignment unverified',
                'counts': {n: len(t) for n, t in self.times.items()}, 'rates_hz': rates,
                'matched_raw_rect_info': matched, 'comparisons': self.comparisons,
                'rectified_age_seconds_median': float(np.median(self.ages)) if self.ages else None,
                'rectified_age_seconds_max': max(self.ages, default=None),
                'errors': sorted(self.errors)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=30)
    parser.add_argument('--output-dir', type=Path, help='New directory outside the repository')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 90:
        parser.error('--seconds must be between 10 and 90')
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    rclpy.init()
    node = RectificationCheck(args.output_dir)
    try:
        end = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = node.report()
        text = json.dumps(report, indent=2)
        if args.output_dir:
            (args.output_dir / 'report.json').write_text(text + '\n')
        print(text, flush=True)
        return 0 if report['passed'] else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
