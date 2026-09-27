#!/usr/bin/env python3
"""Save one RGB/depth pair for registration inspection, never apply a transform.

Subscribes to the existing ROS camera topics only. Output is private and must be
outside the repository. Same-pixel edge overlays are deliberately NOT evidence
of calibration by themselves; use identifiable stationary objects at several
distances. Header proximity is not exposure synchronization.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def stamp_ns(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def image_array(message):
    if (message.width, message.height) != (640, 480):
        raise ValueError('Expected 640x480 images')
    if message.encoding == 'rgb8':
        if message.step < message.width*3 or len(message.data) != message.step*message.height:
            raise ValueError('Invalid RGB stride/data length')
        rows = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        return rows[:, :message.width*3].reshape(message.height, message.width, 3).copy()
    if message.encoding == '16UC1':
        if message.step < message.width*2 or len(message.data) != message.step*message.height:
            raise ValueError('Invalid depth stride/data length')
        return np.ndarray((message.height, message.width),
            dtype='>u2' if message.is_bigendian else '<u2', buffer=message.data,
            strides=(message.step, 2)).astype(np.uint16)
    raise ValueError(f'Unsupported encoding: {message.encoding}')


def depth_edges(depth, threshold_mm=80):
    """Separate metric jumps between valid pixels from invalid-depth boundaries."""
    valid = depth > 0
    jumps = np.zeros(depth.shape, np.uint8)
    invalid_edges = np.zeros(depth.shape, np.uint8)
    for axis in (0, 1):
        a = [slice(None), slice(None)]
        b = [slice(None), slice(None)]
        a[axis], b[axis] = slice(None, -1), slice(1, None)
        a, b = tuple(a), tuple(b)
        both = valid[a] & valid[b]
        jump = both & (np.abs(depth[a].astype(np.int32)-depth[b].astype(np.int32)) > threshold_mm)
        jumps[a] |= jump.astype(np.uint8)
        jumps[b] |= jump.astype(np.uint8)
        boundary = valid[a] != valid[b]
        invalid_edges[a] |= boundary.astype(np.uint8)
        invalid_edges[b] |= boundary.astype(np.uint8)
    return jumps.astype(bool), invalid_edges.astype(bool)


def info_dict(info):
    return {'stamp_ns': stamp_ns(info), 'frame_id': info.header.frame_id,
            'width': info.width, 'height': info.height, 'distortion_model': info.distortion_model,
            'k': list(info.k), 'd': list(info.d), 'r': list(info.r), 'p': list(info.p)}


def write_pair(output, rgb_msg, depth_msg, rgb_info, depth_info):
    rgb, depth = image_array(rgb_msg), image_array(depth_msg)
    k, d = np.array(rgb_info.k).reshape(3, 3), np.array(rgb_info.d)
    r, p = np.array(rgb_info.r).reshape(3, 3), np.array(rgb_info.p).reshape(3, 4)
    maps = cv2.initUndistortRectifyMap(k, d, r, p[:, :3], (640, 480), cv2.CV_32FC1)
    rect = cv2.remap(rgb, *maps, cv2.INTER_LINEAR)
    jumps, invalid_edges = depth_edges(depth)
    scaled = np.clip((depth.astype(np.float32)-300.)/(4000.-300.)*255., 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    colored[depth == 0] = 0
    images = {'rgb_raw.png': cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
              'rgb_rectified.png': cv2.cvtColor(rect, cv2.COLOR_RGB2BGR),
              'depth_mm.png': depth, 'depth_preview.png': colored}
    for label, source in (('raw', rgb), ('rectified', rect)):
        overlay = cv2.cvtColor(source, cv2.COLOR_RGB2BGR)
        overlay[invalid_edges] = (0, 255, 255)  # yellow: invalid boundary, NOT a measured obstacle
        overlay[jumps] = (255, 255, 0)  # cyan: >80mm adjacent valid-depth jump
        cv2.putText(overlay, 'Same-pixel check ONLY - no alignment applied', (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
        images[f'overlay_{label}.png'] = overlay
    for name, array in images.items():
        if not cv2.imwrite(str(output / name), array):
            raise RuntimeError(f'Could not write {name}')
    report = {'scope': 'Inspection only, not calibrated RGB-D',
        'rgb_header_ns': stamp_ns(rgb_msg), 'depth_header_ns': stamp_ns(depth_msg),
        'depth_minus_rgb_header_ms': (stamp_ns(depth_msg)-stamp_ns(rgb_msg))/1e6,
        'rgb_camera_info_exact_stamp': stamp_ns(rgb_msg) == stamp_ns(rgb_info),
        'depth_camera_info_exact_stamp': stamp_ns(depth_msg) == stamp_ns(depth_info),
        'rgb_info': info_dict(rgb_info), 'depth_info': info_dict(depth_info),
        'nonzero_depth_fraction': float(np.mean(depth > 0)),
        'depth_units': 'millimetres, requires driver PIXEL_FORMAT_DEPTH_1_MM mode',
        'overlay_cyan': 'Adjacent valid depths differ by >80mm',
        'overlay_yellow': 'Boundary of invalid depth; not a measured surface',
        'warning': 'Do not fit an arbitrary 2D shift from this overlay or treat equal frame IDs as alignment proof.'}
    (output / 'pair.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=15)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 30:
        parser.error('--seconds must be between 5 and 30')
    output = args.output_dir.expanduser().resolve()
    repo = Path(__file__).resolve().parents[2]
    if output.is_relative_to(repo):
        parser.error('Private camera images must be outside the repository')
    if output.exists():
        parser.error('Output directory must be new; existing data will not be overwritten')
    cv2.setNumThreads(1)
    rclpy.init(args=[])
    node = rclpy.create_node('roam_rgbd_pair_inspection')
    frames = {'rgb': deque(maxlen=8), 'depth': deque(maxlen=40)}
    infos = {'rgb': deque(maxlen=40), 'depth': deque(maxlen=80)}
    subs = []
    for key, topic in [('rgb', '/camera/color/image_raw'), ('depth', '/camera/depth_raw/image')]:
        subs.append(node.create_subscription(Image, topic,
                    lambda m, key=key: frames[key].append(m), qos_profile_sensor_data))
    for key, topic in [('rgb', '/camera/color/camera_info'), ('depth', '/camera/depth_raw/camera_info')]:
        subs.append(node.create_subscription(CameraInfo, topic,
                    lambda m, key=key: infos[key].append(m), qos_profile_sensor_data))
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic()-start < 3 or not all(frames.values()):
                continue
            for rgb in reversed(frames['rgb']):
                # Wait for depth stamps on both sides before choosing the nearest.
                if not stamp_ns(frames['depth'][0]) <= stamp_ns(rgb) <= stamp_ns(frames['depth'][-1]):
                    continue
                depth = min(frames['depth'], key=lambda m: abs(stamp_ns(m)-stamp_ns(rgb)))
                if abs(stamp_ns(depth)-stamp_ns(rgb)) > 20000000:
                    continue
                rgb_info = next((m for m in infos['rgb'] if stamp_ns(m) == stamp_ns(rgb)), None)
                depth_info = next((m for m in infos['depth'] if stamp_ns(m) == stamp_ns(depth)), None)
                if rgb_info is None or depth_info is None:
                    continue
                output.mkdir(parents=True, exist_ok=False)
                result = write_pair(output, rgb, depth, rgb_info, depth_info)
                print(json.dumps({'output_dir': str(output),
                    'depth_minus_rgb_header_ms': result['depth_minus_rgb_header_ms'],
                    'nonzero_depth_fraction': result['nonzero_depth_fraction']}, indent=2))
                return 0
        print('No complete RGB/depth/CameraInfo pair within 20ms in the bounded observation.')
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
