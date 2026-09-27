#!/usr/bin/env python3
"""Check a chessboard in the existing RGB stream; never calibrate or write images.

Defaults describe the 8x8 chessboard, measured at 35 mm per square.
This checks corner detection ONLY, not board accuracy, RGB/depth alignment,
infrared contrast, capture diversity, or readiness for SLAM.
"""
import argparse
import json
import math
import time

import cv2
import numpy as np


def object_points(columns, rows, square_m):
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_m
    return points


def image_to_gray(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError('Expected rgb8 or bgr8, got ' + msg.encoding)
    if msg.height < 1 or msg.width < 1 or msg.step < msg.width * 3:
        raise ValueError('Invalid RGB dimensions or row stride')
    if len(msg.data) < msg.height * msg.step:
        raise ValueError('Truncated image buffer')
    pixels = np.ndarray((msg.height, msg.width, 3), dtype=np.uint8,
                        buffer=msg.data, strides=(msg.step, 3, 1))
    return cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8'
                       else cv2.COLOR_BGR2GRAY)


def detect(gray, pattern):
    """Full-grid detection, with subpixel classic fallback for SB misses."""
    expected = math.prod(pattern)

    def complete(found, corners):
        return (found and corners is not None and len(corners) == expected
                and np.isfinite(corners).all())

    found, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if complete(found, corners):
        return corners
    found, corners = cv2.findChessboardCorners(
        gray, pattern,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not complete(found, corners):
        return None
    corners = cv2.cornerSubPix(
        gray, corners, (5, 5), (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    return corners if complete(True, corners) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/camera/color/image_raw')
    parser.add_argument('--columns', type=int, default=7, help='Internal corners, not squares')
    parser.add_argument('--rows', type=int, default=7)
    parser.add_argument('--square-m', type=float, default=0.035)
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--rate', type=float, default=1.0)
    args = parser.parse_args()
    if not 3 <= args.columns <= 20 or not 3 <= args.rows <= 20:
        parser.error('Internal-corner counts must be between 3 and 20')
    if not math.isfinite(args.square_m) or not 0 < args.square_m <= 1:
        parser.error('--square-m must be finite, positive, and at most 1 metre')
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 60:
        parser.error('--seconds must be between 1 and 60')
    if not math.isfinite(args.rate) or not 0 < args.rate <= 2:
        parser.error('--rate must be positive and no greater than 2 Hz')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    cv2.setNumThreads(1)
    rclpy.init(args=[])
    node = Node('roam_calibration_target_check')
    latest = [None]
    received = [0]

    def receive(msg):
        received[0] += 1
        latest[0] = (msg, time.monotonic())

    sub = node.create_subscription(Image, args.topic, receive,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE))
    pattern = (args.columns, args.rows)
    started = time.monotonic()
    next_check = started
    checked = found_count = errors = 0
    interrupted = False
    print(json.dumps({'event': 'started', 'topic': args.topic,
        'internal_corners': pattern, 'square_m': args.square_m,
        'calibration_applied': False}), flush=True)
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if latest[0] is None or now < next_check:
                continue
            msg, arrival = latest[0]
            latest[0] = None
            if now - arrival > 1.0:
                continue
            next_check = now + 1 / args.rate
            checked += 1
            tick = time.monotonic()
            try:
                corners = detect(image_to_gray(msg), pattern)
            except (ValueError, cv2.error) as error:
                errors += 1
                print(json.dumps({'event': 'image_error', 'error': str(error)}), flush=True)
                continue
            found_count += int(corners is not None)
            result = {'event': 'frame', 'width': msg.width, 'height': msg.height,
                'found': corners is not None, 'corners': 0 if corners is None else len(corners),
                'detection_ms': round((time.monotonic() - tick) * 1000, 1)}
            if corners is not None:
                xy = corners.reshape(-1, 2)
                result['internal_corner_bounds_px'] = [xy.min(axis=0).tolist(), xy.max(axis=0).tolist()]
            print(json.dumps(result), flush=True)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({'event': 'summary', 'received': received[0], 'checked': checked,
        'detections': found_count, 'decode_errors': errors, 'interrupted': interrupted,
        'calibration_applied': False, 'images_saved': False}), flush=True)
    return 130 if interrupted else (0 if found_count else 2)


if __name__ == '__main__':
    raise SystemExit(main())
