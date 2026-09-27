#!/usr/bin/env python3
"""Observe RGB-D inputs for a bounded time; never publish or command hardware.

Subscribing to raw RGB activates the existing driver's on-demand decode path.
No image pixels are stored. Arrival/header ages measure the host pipeline, not
exposure synchronization. A numerical CameraInfo check is NOT calibration proof.
The driver may intentionally offset depth stamps (currently -33 ms). A high
nearest-pair fraction at 30 FPS therefore does not establish synchronized images.
"""
import argparse
from bisect import bisect_left
import json
import math
import statistics
import time


def distribution(values):
    values = sorted(values)
    if not values:
        return None
    return {'min': values[0], 'median': statistics.median(values),
            'p95': values[min(len(values)-1, math.ceil(.95*len(values))-1)],
            'max': values[-1]}


def nearest_deltas_ms(rgb_stamps, depth_stamps):
    """Signed nearest depth-minus-RGB header deltas; excludes overlap edges."""
    depth = sorted(depth_stamps)
    result = []
    for stamp in rgb_stamps:
        if not depth or stamp < depth[0] or stamp > depth[-1]:
            continue
        index = bisect_left(depth, stamp)
        candidates = depth[max(0, index-1):index+1]
        result.append((min(candidates, key=lambda t: abs(t-stamp)) - stamp)*1000)
    return result


def camera_info_summary(msg):
    numbers = list(msg.k) + list(msg.d) + list(msg.r) + list(msg.p)
    finite = all(math.isfinite(v) for v in numbers)
    return {'width': msg.width, 'height': msg.height,
            'frame_id': msg.header.frame_id, 'distortion_model': msg.distortion_model,
            'k': list(msg.k), 'd': list(msg.d), 'r': list(msg.r), 'p': list(msg.p),
            'finite_positive_focal_lengths': bool(finite and len(msg.k) == 9
                                                  and msg.k[0] > 0 and msg.k[4] > 0),
            'calibration_provenance': 'NOT VERIFIED by this check'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('--seconds must be between 1 and 120')
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    rclpy.init(args=[])
    node = Node('roam_rgbd_input_check')
    stats = {}
    infos = {}
    subscriptions = []

    def receive(topic, msg):
        entry = stats.setdefault(topic, {'arrivals': [], 'stamps': [], 'ages_ms': [],
                                        'non_increasing_stamps': 0, 'last': {}})
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        now = node.get_clock().now().nanoseconds*1e-9
        if entry['stamps'] and stamp <= entry['stamps'][-1]:
            entry['non_increasing_stamps'] += 1
        entry['arrivals'].append(time.monotonic())
        entry['stamps'].append(stamp)
        entry['ages_ms'].append((now-stamp)*1000)
        entry['last'] = {'frame_id': msg.header.frame_id, 'bytes': len(msg.data)}
        if isinstance(msg, Image):
            entry['last'].update(width=msg.width, height=msg.height,
                                 encoding=msg.encoding, step=msg.step)
            # Sample at low rate, respecting endian and row padding.
            if msg.encoding == '16UC1' and len(entry['arrivals']) % 30 == 1:
                try:
                    image = np.ndarray((msg.height, msg.width),
                        dtype=np.dtype('>u2' if msg.is_bigendian else '<u2'),
                        buffer=msg.data, strides=(msg.step, 2))[::8, ::8]
                    valid = image[image > 0]
                    entry['depth_sample'] = {'nonzero_percent': float(valid.size/image.size*100),
                        'median_nonzero_mm': float(np.median(valid)) if valid.size else None}
                except (ValueError, TypeError, ZeroDivisionError) as error:
                    entry['depth_sample'] = {'decode_error': str(error)}

    image_topics = {
        '/camera/color/image_raw': Image,
        '/camera/color/image_raw/compressed': CompressedImage,
        '/camera/depth_raw/image': Image,
    }
    for topic, kind in image_topics.items():
        subscriptions.append(node.create_subscription(kind, topic,
            lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data))
    for topic in ['/camera/color/camera_info', '/camera/depth_raw/camera_info']:
        subscriptions.append(node.create_subscription(CameraInfo, topic,
            lambda msg, t=topic: infos.update({t: camera_info_summary(msg)}), qos_profile_sensor_data))
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        finished = time.monotonic()
        result = {'duration_seconds': finished-started, 'images': {}, 'camera_info': infos,
                  'slam_ready': False,
                  'limitations': ['Calibration and registration need independent validation.',
                                  'Header matching does not measure exposure synchronization.',
                                  'No TF or motor commands are published.']}
        for topic in image_topics:
            entry = stats.get(topic)
            if not entry:
                result['images'][topic] = {'count': 0}
                continue
            times = entry['arrivals']
            result['images'][topic] = {
                'count': len(times),
                'received_hz': (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0,
                'last_receipt_age_s': finished-times[-1],
                'max_receipt_gap_s': max((b-a for a,b in zip(times,times[1:])), default=0),
                'header_age_ms': distribution(entry['ages_ms']),
                'non_increasing_stamps': entry['non_increasing_stamps'],
                **entry['last']}
            if 'depth_sample' in entry:
                result['images'][topic]['depth_sample'] = entry['depth_sample']
        rgb = stats.get('/camera/color/image_raw', {}).get('stamps', [])
        depth = stats.get('/camera/depth_raw/image', {}).get('stamps', [])
        deltas = nearest_deltas_ms(rgb, depth)
        result['nearest_depth_minus_rgb_header_ms'] = distribution(deltas)
        result['paired_within_20ms_fraction'] = sum(abs(d)<=20 for d in deltas)/len(deltas) if deltas else None
        result['nearest_pair_count'] = len(deltas)
        print(json.dumps(result, indent=2, allow_nan=False))
        missing = [topic for topic in image_topics if topic not in stats]
        return 2 if missing else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
