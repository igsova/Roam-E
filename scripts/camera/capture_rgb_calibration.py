#!/usr/bin/env python3
"""Record bounded, diverse chessboard views; does not solve/apply calibration.

Subscribes to the existing RGB stream. Saves grayscale PNGs plus detected corners
outside the repo; never opens USB, changes camera parameters or publishes to ROS.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from check_calibration_target import detect, image_to_gray

PATTERN = (7, 7)
SQUARE_M = 0.035
SIZE = (640, 480)


def grid_distance(a, b):
    """RMS pixel distance, independent of square board's 8 possible orderings."""
    target = np.asarray(a).reshape(7, 7, 2)
    source = np.asarray(b).reshape(7, 7, 2)
    variants = [np.rot90(source, k, axes=(0, 1)) for k in range(4)]
    variants += [variant[::-1] for variant in variants[:]]
    return min(float(np.sqrt(np.mean(np.sum((target - item) ** 2, axis=2))))
               for item in variants)


def board_quality(gray, corners):
    xy = np.asarray(corners).reshape(7, 7, 2)
    flat = xy.reshape(-1, 2)
    if not np.isfinite(flat).all():
        return 'invalid_corners', {}
    spacing = np.concatenate((np.linalg.norm(np.diff(xy, axis=0), axis=2).ravel(),
                              np.linalg.norm(np.diff(xy, axis=1), axis=2).ravel()))
    details = {'min_spacing_px': float(spacing.min()),
               'bounds_px': [flat.min(axis=0).tolist(), flat.max(axis=0).tolist()]}
    if spacing.min() < 10:
        return 'board_too_small_or_steep', details
    height, width = gray.shape
    if (flat < 8).any() or (flat[:, 0] > width - 9).any() or (flat[:, 1] > height - 9).any():
        return 'corners_too_close_to_edge', details
    widths = [float(cv2.estimateChessboardSharpness(gray, PATTERN, corners,
                vertical=vertical)[0][0]) for vertical in (False, True)]
    details['edge_transition_px'] = widths
    if any(not math.isfinite(value) or value <= 0 or value > 3.0 for value in widths):
        return 'blurred_board', details
    return None, details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, help='New directory; refuses existing paths')
    parser.add_argument('--seconds', type=int, default=180)
    parser.add_argument('--max-views', type=int, default=40)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 300 or not 1 <= args.max_views <= 60:
        parser.error('Use 10..300 seconds and 1..60 views')
    output = Path(args.output_dir).expanduser().resolve()
    output.mkdir(mode=0o700, exist_ok=False)
    manifest = {'format_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
        'topic': '/camera/color/image_raw', 'image_size': list(SIZE),
        'internal_corners': list(PATTERN), 'square_m': SQUARE_M,
        'calibration_applied': False, 'views': [], 'status': 'recording',
        'limitations': ['RGB intrinsics dataset only; no depth/IR calibration.',
                        'Symmetric board order may differ between images.',
                        'Diverse views and independent validation are still required.']}

    def persist():
        temporary = output / 'session.json.tmp'
        temporary.write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')
        temporary.replace(output / 'session.json')

    def emit(value):
        print(json.dumps(value, allow_nan=False), flush=True)

    persist()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    cv2.setNumThreads(1)
    rclpy.init(args=[])
    node = Node('roam_rgb_calibration_capture')
    latest = [None]
    received = [0]
    last_received_at = [time.monotonic()]

    def receive(msg):
        received[0] += 1
        last_received_at[0] = time.monotonic()
        latest[0] = (msg, last_received_at[0])

    subscription = node.create_subscription(Image, manifest['topic'], receive,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE))
    started = next_check = last_heartbeat = time.monotonic()
    previous = None
    previous_at = 0.0
    accepted_corners = []
    rejected = Counter()
    checked = 0
    last_stamp = None
    reason = 'waiting_for_images'
    emit({'event': 'recording_started', 'output_dir': str(output),
          'seconds': args.seconds, 'max_views': args.max_views, 'square_m': SQUARE_M})
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds and len(accepted_corners) < args.max_views:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_received_at[0] > 8:
                raise RuntimeError('No RGB images for 8 seconds; capture stopped. Check the camera connection.')
            if now - last_heartbeat >= 5:
                emit({'event': 'progress', 'elapsed_s': round(now-started),
                      'saved': len(accepted_corners), 'received': received[0], 'last_status': reason})
                last_heartbeat = now
            if latest[0] is None or now < next_check:
                continue
            msg, arrived = latest[0]
            latest[0] = None
            next_check = now + 1.0
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp == last_stamp or now - arrived > 1:
                reason = 'stale_image'
                rejected[reason] += 1
                continue
            last_stamp = stamp
            if (msg.width, msg.height) != SIZE:
                raise RuntimeError('Camera resolution changed; refusing mixed-resolution dataset')
            checked += 1
            try:
                gray = image_to_gray(msg)
                corners = detect(gray, PATTERN)
                reason, details = ('board_not_found', {}) if corners is None else board_quality(gray, corners)
            except (ValueError, cv2.error) as error:
                reason = 'image_or_detector_error'
                emit({'event': reason, 'error': str(error)})
                corners = None
            if reason:
                rejected[reason] += 1
                previous = None
                continue
            stable = previous is not None and now - previous_at < 3 and grid_distance(corners, previous) <= 2.0
            previous, previous_at = corners, now
            if not stable:
                reason = 'hold_still'
                rejected[reason] += 1
                continue
            if any(grid_distance(corners, saved) < 15.0 for saved in accepted_corners):
                reason = 'duplicate_view_move_to_new_pose'
                rejected[reason] += 1
                continue
            filename = 'view_%03d.png' % (len(accepted_corners) + 1)
            if not cv2.imwrite(str(output / filename), gray):
                raise RuntimeError('Could not save ' + filename)
            accepted_corners.append(corners.copy())
            manifest['views'].append({'file': filename,
                'stamp': {'sec': stamp[0], 'nanosec': stamp[1]},
                'frame_id': msg.header.frame_id, 'encoding': 'mono8',
                'corners_px': corners.reshape(-1, 2).tolist(), **details})
            persist()
            reason = 'saved'
            emit({'event': 'saved', 'index': len(accepted_corners), 'file': filename, **details})
        manifest['status'] = 'capture_finished'
    except KeyboardInterrupt:
        manifest['status'] = 'interrupted'
    except Exception as error:
        manifest['status'] = 'failed'
        manifest['error'] = str(error)
        raise
    finally:
        manifest.update(received=received[0], checked=checked,
                        rejection_counts=dict(rejected), elapsed_s=time.monotonic()-started)
        persist()
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        emit({'event': 'summary', 'output_dir': str(output), 'saved': len(accepted_corners),
              'status': manifest['status'], 'rejection_counts': dict(rejected),
              'calibration_applied': False})
    return 0 if accepted_corners else 2


if __name__ == '__main__':
    raise SystemExit(main())
