#!/usr/bin/env python3
"""Bounded metadata/cloud/optional odometry observer. Never commands hardware.

Writes test outputs only to an explicitly supplied new directory.
Cloud screenshots are projections of measured XYZ/RGB.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from firmware_rgbd_math import stamp_ns


def distribution(values):
    return {'min': float(np.min(values)), 'median': float(np.median(values)),
            'p95': float(np.percentile(values, 95)), 'max': float(np.max(values))} if values else None


def render_cloud(points):
    """Two independently fitted orthographic views, front and 30deg side."""
    xyz = np.column_stack([points[n] for n in ('x', 'y', 'z')]).astype(float)
    packed = points['rgb']
    colors = np.column_stack([packed & 255, (packed >> 8) & 255, (packed >> 16) & 255]).astype(np.uint8)
    canvas = np.full((560, 1280, 3), 20, np.uint8)
    if not len(xyz):
        return canvas
    for panel, angle in enumerate([0, 30]):
        rad = np.deg2rad(angle)
        x = xyz[:, 0]*np.cos(rad) + xyz[:, 2]*np.sin(rad)
        y = xyz[:, 1]
        z = -xyz[:, 0]*np.sin(rad) + xyz[:, 2]*np.cos(rad)
        low = np.percentile(np.column_stack([x, y]), 1, axis=0)
        high = np.percentile(np.column_stack([x, y]), 99, axis=0)
        scale = min(600/max(high[0]-low[0], .01), 460/max(high[1]-low[1], .01))
        u = ((x-low[0])*scale + 20 + panel*640).astype(int)
        v = ((y-low[1])*scale + 60).astype(int)
        for i in np.argsort(z)[::-1]:
            if panel*640+10 <= u[i] < panel*640+630 and 50 <= v[i] < 540:
                cv2.circle(canvas, (int(u[i]), int(v[i])), 2, tuple(int(c) for c in colors[i]), -1)
        cv2.putText(canvas, f'Colored XYZ: {angle} deg view', (panel*640+20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (240, 240, 240), 1, cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tracking', action='store_true')
    args = parser.parse_args()
    if not 2 <= args.seconds <= 120:
        parser.error('--seconds must be 2..120')
    args.output.mkdir(parents=False, exist_ok=False)
    rclpy.init(args=[])
    node = Node('rgbd_test_observer', namespace='rgbd_test')
    qos = qos_profile_sensor_data
    times, ages = defaultdict(list), defaultdict(list)
    latest, bundles = {}, {}
    matched = []
    poses, odom_info = [], []
    subs = []

    def receive(key, msg):
        now = node.get_clock().now().nanoseconds
        times[key].append(time.monotonic())
        latest[key] = msg
        if hasattr(msg, 'header'):
            stamp = stamp_ns(msg.header)
            ages[key].append((now-stamp)/1e6)
            if key in ('rgb', 'depth', 'rgb_info', 'depth_info'):
                bundle = bundles.setdefault(stamp, {})
                bundle[key] = msg
                if len(bundle) == 4:
                    rgb, depth, ri, di = [bundle[k] for k in ('rgb', 'depth', 'rgb_info', 'depth_info')]
                    consistent = (len({m.header.frame_id for m in bundle.values()}) == 1
                                  and all((m.width, m.height) == (640, 480) for m in bundle.values())
                                  and np.array_equal(ri.k, di.k) and np.array_equal(ri.p, di.p)
                                  and np.allclose(ri.d, 0) and np.allclose(di.d, 0)
                                  and rgb.encoding == 'rgb8' and depth.encoding == '16UC1')
                    matched.append(bool(consistent))
                    latest['matched_bundle'] = bundle
                while len(bundles) > 8:
                    del bundles[next(iter(bundles))]
        if key == 'odom':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            poses.append({'stamp_ns': stamp_ns(msg.header),
                          'xyz': [p.x, p.y, p.z], 'xyzw': [q.x, q.y, q.z, q.w],
                          'position_covariance_diagonal': [msg.pose.covariance[i] for i in (0, 7, 14)],
                          'parent': msg.header.frame_id, 'child': msg.child_frame_id})
        if key == 'odom_info':
            odom_info.append({'lost': msg.lost, 'features': msg.features, 'inliers': msg.inliers,
                              'time_estimation_s': msg.time_estimation, 'matches': msg.matches})

    for key, kind, topic in [
        ('rgb', Image, 'rgb/image_rect'), ('depth', Image, 'depth/image_rect'),
        ('rgb_info', CameraInfo, 'rgb/camera_info'), ('depth_info', CameraInfo, 'depth/camera_info'),
        ('cloud', PointCloud2, 'points'), ('status', String, 'status'),
    ]:
        subs.append(node.create_subscription(kind, topic, lambda msg, k=key: receive(k, msg), qos))
    if args.tracking:
        from rtabmap_msgs.msg import OdomInfo
        for key, kind, topic in [('odom', Odometry, 'odom'), ('odom_info', OdomInfo, 'odom_info_lite')]:
            subs.append(node.create_subscription(kind, topic, lambda msg, k=key: receive(k, msg), qos))
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        result = {
            'experimental_baseline_only': True,
            'topics': {k: {'count': len(t), 'hz': (len(t)-1)/(t[-1]-t[0]) if len(t)>1 else 0,
                           'age_ms': distribution(ages[k])} for k, t in times.items()},
            'complete_exact_stamp_bundles': len(matched),
            'consistent_bundles': sum(matched),
            'adapter_status': json.loads(latest['status'].data) if 'status' in latest else None,
            'poses': poses, 'odom_info': odom_info,
            'lost_count': sum(i['lost'] for i in odom_info),
            'inliers': distribution([i['inliers'] for i in odom_info]),
            'tracking_time_ms': distribution([i['time_estimation_s']*1000 for i in odom_info]),
        }
        result['tracking_smoke_check_passed'] = (bool(
            len(poses) >= 2 and len(odom_info) >= 2 and not any(i['lost'] for i in odom_info)
            and all(np.isfinite(p['xyz'] + p['xyzw']).all()
                    and .99 < np.linalg.norm(p['xyzw']) < 1.01 for p in poses)
        ) if args.tracking else None)
        if len(poses) > 1:
            xyz = np.array([p['xyz'] for p in poses])
            result['observed_position_extent_m'] = np.ptp(xyz, axis=0).tolist()
            result['first_to_last_distance_m'] = float(np.linalg.norm(xyz[-1]-xyz[0]))
            result['motion_ground_truth_available'] = False
        if 'cloud' in latest:
            msg = latest['cloud']
            if msg.point_step != 16 or msg.is_bigendian:
                raise ValueError('Unexpected cloud layout')
            p = np.frombuffer(msg.data, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
            xyz = np.column_stack([p[n] for n in ('x', 'y', 'z')])
            result['cloud'] = {'points': len(p), 'bytes': len(msg.data), 'frame': msg.header.frame_id,
                               'all_xyz_finite': bool(np.isfinite(xyz).all()),
                               'xyz_min_m': xyz.min(axis=0).tolist() if len(p) else [],
                               'xyz_max_m': xyz.max(axis=0).tolist() if len(p) else [],
                               'distinct_colors': int(len(np.unique(p['rgb'])))}
            np.savez_compressed(args.output/'colored_cloud.npz', points=p)
            cv2.imwrite(str(args.output/'colored_cloud_views.png'), render_cloud(p))
        if 'matched_bundle' in latest:
            b = latest['matched_bundle']
            rgb = np.frombuffer(b['rgb'].data, np.uint8).reshape(480, 640, 3)
            depth = np.frombuffer(b['depth'].data, '<u2').reshape(480, 640)
            np.savez_compressed(args.output/'matched_pair.npz', rgb=rgb, depth_mm=depth, k=b['rgb_info'].k)
            cv2.imwrite(str(args.output/'rectified_rgb.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps({k: v for k, v in result.items() if k not in ('poses', 'odom_info')}, indent=2))
        good = matched and all(matched) and 'cloud' in latest
        if args.tracking:
            good = good and result['tracking_smoke_check_passed']
        return 0 if good else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
