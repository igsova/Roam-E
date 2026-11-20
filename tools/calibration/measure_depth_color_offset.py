#!/usr/bin/env python3
"""Measure the pixel offset between the Astra's registered depth and the UVC
colour image, by matching depth edges to colour edges.

Read-only: it only subscribes. Place an object with clear outlines (a box)
in front of a farther background so the depth image has edges. The offset
is reported per depth band so a distance-dependent (parallax) error shows up.

Usage (Pi, ROS sourced, cameras running):
  python3 measure_depth_color_offset.py --seconds 6 --label box_1m
"""
from __future__ import annotations

import argparse
import json
import time

import numpy as np

SEARCH_PX = 20
MARGIN = 40
BANDS_M = ((0.4, 0.9), (0.9, 1.4), (1.4, 2.2), (2.2, 4.0))


def depth_edges(depth_m: np.ndarray, jump_m: float = 0.08) -> np.ndarray:
    """Pixels where depth jumps by more than jump_m to a neighbour, on the
    near (foreground) side, which is where the object outline lies."""
    import cv2
    d = depth_m.copy()
    valid = d > 0
    d[~valid] = np.inf
    edges = np.zeros(d.shape, bool)
    with np.errstate(invalid='ignore'):
        for axis in (0, 1):
            for shift in (1, -1):
                neighbour = np.roll(d, shift, axis)
                edges |= valid & (neighbour - d > jump_m)
    return cv2.dilate(edges.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0


def colour_edges(rgb: np.ndarray) -> np.ndarray:
    import cv2
    grey = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(grey, (3, 3), 0), 40, 120).astype(np.float32) / 255
    return cv2.GaussianBlur(edges, (5, 5), 0)


def best_shift(mask: np.ndarray, colour: np.ndarray):
    """(dx, dy) to add to depth pixel coordinates to land on colour edges."""
    h, w = mask.shape
    roi = (slice(MARGIN, h - MARGIN), slice(MARGIN, w - MARGIN))
    scores = np.zeros((2 * SEARCH_PX + 1, 2 * SEARCH_PX + 1))
    m = mask.astype(np.float32)
    for iy, dy in enumerate(range(-SEARCH_PX, SEARCH_PX + 1)):
        for ix, dx in enumerate(range(-SEARCH_PX, SEARCH_PX + 1)):
            shifted = np.roll(np.roll(m, dy, 0), dx, 1)
            scores[iy, ix] = float((shifted[roi] * colour[roi]).sum())
    iy, ix = np.unravel_index(np.argmax(scores), scores.shape)
    peak = scores[iy, ix]
    contrast = peak / max(float(np.median(scores)), 1e-6)
    return int(ix) - SEARCH_PX, int(iy) - SEARCH_PX, round(float(contrast), 2)


def main() -> int:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=6.0)
    parser.add_argument('--label', default='')
    parser.add_argument('--colour-topic', default='/camera/color/image_rect')
    parser.add_argument('--depth-topic', default='/camera/depth_raw/image')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('depth_colour_offset')
    pairs, last_depth = [], {}

    def on_depth(msg):
        last_depth['m'] = msg

    def on_colour(msg):
        if 'm' in last_depth:
            pairs.append((msg, last_depth['m']))

    node.create_subscription(Image, args.depth_topic, on_depth, qos_profile_sensor_data)
    node.create_subscription(Image, args.colour_topic, on_colour, qos_profile_sensor_data)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    result = {'label': args.label, 'colour_topic': args.colour_topic,
              'depth_topic': args.depth_topic, 'frames': len(pairs)}
    if not pairs:
        result['error'] = 'no colour/depth frames'
        print(json.dumps(result, indent=1))
        return 2
    colour_sum, edge_masks, depth_med = None, [], []
    for colour_msg, depth_msg in pairs:
        rgb = np.frombuffer(colour_msg.data, np.uint8).reshape(colour_msg.height, colour_msg.width, 3)
        depth = np.frombuffer(depth_msg.data, '<u2').reshape(
            depth_msg.height, depth_msg.step // 2)[:, :depth_msg.width].astype(np.float32) / 1000
        c = colour_edges(rgb)
        colour_sum = c if colour_sum is None else colour_sum + c
        edge_masks.append(depth_edges(depth))
        depth_med.append(depth)
    colour = colour_sum / len(pairs)
    edges = np.mean(edge_masks, axis=0) > 0.5
    depth = np.median(np.stack(depth_med), axis=0)
    dx, dy, contrast = best_shift(edges, colour)
    result['all_edges'] = {'pixels': int(edges.sum()), 'dx': dx, 'dy': dy, 'contrast': contrast}
    bands = []
    for lo, hi in BANDS_M:
        mask = edges & (depth >= lo) & (depth < hi)
        if mask.sum() < 300:
            continue
        bdx, bdy, bc = best_shift(mask, colour)
        bands.append({'depth_m': [lo, hi], 'median_m': round(float(np.median(depth[mask])), 3),
                      'pixels': int(mask.sum()), 'dx': bdx, 'dy': bdy, 'contrast': bc})
    result['bands'] = bands
    print(json.dumps(result, indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
