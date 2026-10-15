#!/usr/bin/env python3
"""Read-only camera-vs-lidar wall check for the R19 Astra mount and depth scale.

The robot faces a flat wall (or a large flat board) with the motors off. This
script collects registered depth frames and lidar scans for a few seconds, fits
the wall as a plane in the depth image and as a line in the scan, expresses both
in base_link via TF, and reports:

- depth distance vs lidar distance to the wall (along the wall normal),
- wall yaw seen by the camera vs by the lidar (camera yaw error vs lidar),
- wall tilt seen by the camera (camera pitch/roll error if the wall is vertical),
- optional comparison with a tape-measured distance from the robot front.

It only subscribes; it never publishes commands or changes parameters.
Usage (on the Pi, workspace sourced, camera + description/lidar running):
  python3 measure_r19_camera_wall.py --seconds 5 --tape-m 1.00 --label 1m
"""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np

# Camera ROI (fraction of width/height) assumed to see only the wall.
ROI_X = (0.30, 0.70)
ROI_Y = (0.25, 0.65)
LIDAR_HALF_ANGLE_DEG = 20.0
FRONT_X_M = 0.06965  # R19 CAD hull front, base_footprint/base_link X


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def depth_points(depth_mm: np.ndarray, k: list[float], stride: int = 4) -> np.ndarray:
    """Back-project the ROI of a 16UC1 millimetre image to optical-frame metres."""
    h, w = depth_mm.shape
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    y0, y1 = int(ROI_Y[0] * h), int(ROI_Y[1] * h)
    x0, x1 = int(ROI_X[0] * w), int(ROI_X[1] * w)
    ys, xs = np.mgrid[y0:y1:stride, x0:x1:stride]
    z = depth_mm[ys, xs].astype(float) / 1000.0
    valid = z > 0
    xs, ys, z = xs[valid], ys[valid], z[valid]
    return np.column_stack(((xs - cx) * z / fx, (ys - cy) * z / fy, z))


def fit_plane(points: np.ndarray, iterations: int = 3, keep_m: float = 0.02):
    """Least-squares plane with residual trimming. Returns (unit normal, d, rms, n)
    for n·p = d, with the normal oriented away from the sensor (d > 0)."""
    pts = points
    for _ in range(iterations):
        centroid = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - centroid)
        normal = vt[-1]
        d = float(normal @ centroid)
        if d < 0:
            normal, d = -normal, -d
        residual = points @ normal - d
        pts = points[np.abs(residual) < keep_m]
        if len(pts) < 50:
            break
    rms = float(np.sqrt(np.mean((pts @ normal - d) ** 2)))
    return normal, d, rms, len(pts)


def fit_line(points: np.ndarray, iterations: int = 3, keep_m: float = 0.02):
    """2D least-squares line n·p = d (unit n, d > 0) with trimming."""
    pts = points
    for _ in range(iterations):
        centroid = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - centroid)
        normal = vt[-1]
        d = float(normal @ centroid)
        if d < 0:
            normal, d = -normal, -d
        residual = points @ normal - d
        pts = points[np.abs(residual) < keep_m]
        if len(pts) < 10:
            break
    rms = float(np.sqrt(np.mean((pts @ normal - d) ** 2)))
    return normal, d, rms, len(pts)


def plane_in_base(normal_c: np.ndarray, d_c: float, rotation: np.ndarray,
                  translation: np.ndarray):
    """Transform plane n·p = d from camera frame into base frame."""
    n_b = rotation @ normal_c
    d_b = d_c + float(n_b @ translation)
    return n_b, d_b


def summarize(camera, lidar, tape_m: float | None) -> dict:
    n_cam, d_cam = camera
    n_lid, d_lid = lidar
    cam_yaw = math.degrees(math.atan2(n_cam[1], n_cam[0]))
    lid_yaw = math.degrees(math.atan2(n_lid[1], n_lid[0]))
    horizontal = math.hypot(n_cam[0], n_cam[1])
    tilt = math.degrees(math.atan2(n_cam[2], horizontal))
    # Distances from base origin along each wall normal; the camera normal is
    # projected to the horizontal so both compare the same wall offset.
    out = {
        'camera_wall_distance_m': round(d_cam / horizontal if horizontal else float('nan'), 4),
        'lidar_wall_distance_m': round(d_lid, 4),
        'camera_minus_lidar_m': round(d_cam / horizontal - d_lid, 4),
        'camera_wall_yaw_deg': round(cam_yaw, 2),
        'lidar_wall_yaw_deg': round(lid_yaw, 2),
        'camera_minus_lidar_yaw_deg': round(cam_yaw - lid_yaw, 2),
        'camera_wall_tilt_deg': round(tilt, 2),
    }
    if tape_m is not None:
        out['tape_front_to_wall_m'] = tape_m
        out['lidar_front_to_wall_m'] = round(d_lid - FRONT_X_M, 4)
        out['lidar_minus_tape_m'] = round(d_lid - FRONT_X_M - tape_m, 4)
        out['camera_minus_tape_m'] = round(d_cam / horizontal - FRONT_X_M - tape_m, 4)
    return out


def main() -> int:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    import tf2_ros

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--tape-m', type=float, default=None,
                        help='tape distance from the robot front face to the wall')
    parser.add_argument('--label', default='')
    parser.add_argument('--base-frame', default='base_link')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('camera_wall_check')
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    depths, scans, info = [], [], {}
    node.create_subscription(Image, '/camera/depth_raw/image', depths.append, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/depth_raw/camera_info',
                             lambda m: info.setdefault('k', (list(m.k), m.header.frame_id)),
                             qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    result = {'label': args.label, 'depth_frames': len(depths), 'scans': len(scans)}
    try:
        if not depths or not scans or 'k' not in info:
            raise RuntimeError('Missing depth, camera_info or scan data')
        k, cam_frame = info['k']
        stack = []
        for m in depths:
            if m.encoding not in ('16UC1', 'mono16'):
                raise RuntimeError('Unexpected depth encoding ' + m.encoding)
            arr = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
            stack.append(arr)
        stack = np.stack(stack).astype(float)
        stack[stack == 0] = np.nan
        with np.errstate(all='ignore'):
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                median = np.nan_to_num(np.nanmedian(stack, axis=0), nan=0).astype(np.uint16)
        pts = depth_points(median, k)
        n_c, d_c, rms_c, used_c = fit_plane(pts)
        tf_c = buffer.lookup_transform(args.base_frame, cam_frame, Time())
        q = tf_c.transform.rotation
        t = tf_c.transform.translation
        n_b, d_b = plane_in_base(n_c, d_c, quaternion_matrix(q.x, q.y, q.z, q.w),
                                 np.array([t.x, t.y, t.z]))
        laser_frame = scans[-1].header.frame_id
        tf_l = buffer.lookup_transform(args.base_frame, laser_frame, Time())
        ql, tl = tf_l.transform.rotation, tf_l.transform.translation
        rot_l = quaternion_matrix(ql.x, ql.y, ql.z, ql.w)
        lidar_pts = []
        for s in scans:
            angles = s.angle_min + np.arange(len(s.ranges)) * s.angle_increment
            r = np.array(s.ranges)
            ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
            p = np.column_stack((r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok]), np.zeros(ok.sum())))
            pb = (rot_l @ p.T).T + np.array([tl.x, tl.y, tl.z])
            bearing = np.degrees(np.arctan2(pb[:, 1], pb[:, 0]))
            lidar_pts.append(pb[np.abs(bearing) < LIDAR_HALF_ANGLE_DEG][:, :2])
        lidar_pts = np.vstack(lidar_pts)
        n_l, d_l, rms_l, used_l = fit_line(lidar_pts)
        result.update({
            'camera_frame': cam_frame, 'laser_frame': laser_frame,
            'camera_points_used': used_c, 'camera_plane_rms_m': round(rms_c, 4),
            'lidar_points_used': used_l, 'lidar_line_rms_m': round(rms_l, 4),
            'camera_normal_base': [round(float(x), 4) for x in n_b],
            'lidar_normal_base': [round(float(x), 4) for x in n_l],
        })
        result.update(summarize((n_b, d_b), (np.array([n_l[0], n_l[1], 0.0]), d_l), args.tape_m))
        # Quality gate: the ROI must see one flat surface in both sensors.
        problems = []
        if used_c < 1000 or rms_c > 0.01:
            problems.append('camera ROI is not one flat wall (points %d, rms %.3f m)' % (used_c, rms_c))
        if used_l < 100 or rms_l > 0.01:
            problems.append('lidar front sector is not one straight wall (points %d, rms %.3f m)' % (used_l, rms_l))
        result['problems'] = problems
        result['ok'] = not problems
    except Exception as error:  # Report and exit; nothing to clean up.
        result['ok'] = False
        result['error'] = str(error)
    print(json.dumps(result, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
