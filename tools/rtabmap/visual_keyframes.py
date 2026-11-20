#!/usr/bin/env python3
"""Build a learned-feature keyframe cache from an RTAB-Map export, for the
R19 visual relocalizer (tools/rtabmap/visual_relocalizer.py).

Export first (on a copy of the database, nothing running on it):
  rtabmap-export --images_id --poses_camera --poses_format 10 --output_dir OUT map.db
Then (needs torch; XFeat from ~/venvs/src/accelerated_features or XFEAT_DIR):
  python visual_keyframes.py OUT/map keyframes.npz --features xfeat

Each keyframe stores keypoints/descriptors and, for every keypoint with valid
depth, its 3D position in the map frame. Keypoints without depth are kept
(the matchers use them) but marked invalid for PnP.
"""
import argparse
import os

import cv2
import numpy as np

from visual_features import VisualFeatures


def quat_to_rotation(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def read_camera_matrix(path):
    """camera_matrix data from an RTAB-Map calibration YAML (ROS layout)."""
    text = open(path).read()
    block = text.split('camera_matrix:', 1)[1]
    data = block.split('data:', 1)[1].split('[', 1)[1].split(']', 1)[0]
    return np.array([float(v) for v in data.replace('\n', ' ').split(',')]).reshape(3, 3)


def keypoints_to_map(keypoints, depth_mm, k, camera_pose, min_m=0.3, max_m=4.0):
    """Back-project keypoints with the keyframe depth into the map frame.
    Returns (N, 3) float32 points and an (N,) validity mask."""
    h, w = depth_mm.shape
    u = np.round(keypoints[:, 0]).astype(int).clip(0, w - 1)
    v = np.round(keypoints[:, 1]).astype(int).clip(0, h - 1)
    z = depth_mm[v, u].astype(np.float64) / 1000.0
    valid = (z > min_m) & (z < max_m)
    x = (keypoints[:, 0] - k[0, 2]) * z / k[0, 0]
    y = (keypoints[:, 1] - k[1, 2]) * z / k[1, 1]
    camera_points = np.vstack([x, y, z, np.ones_like(z)])
    world = (camera_pose @ camera_points)[:3].T
    world[~valid] = 0.0
    return world.astype(np.float32), valid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('export_prefix', help='e.g. OUT/map (OUT/map_rgb, OUT/map_depth, ...)')
    parser.add_argument('output')
    parser.add_argument('--features', choices=('xfeat', 'superpoint'), default='xfeat')
    parser.add_argument('--max-keypoints', type=int, default=None)
    args = parser.parse_args()

    prefix = args.export_prefix
    ids = sorted(int(os.path.splitext(f)[0]) for f in os.listdir(prefix + '_rgb'))
    with open(prefix + '_camera_poses.txt') as stream:
        rows = [line.split() for line in stream if not line.startswith('#')]
    if len(rows) != len(ids):
        raise SystemExit(f'{len(ids)} images but {len(rows)} poses')

    features = VisualFeatures(args.features, args.max_keypoints)
    out = {'ids': np.array(ids), 'features': np.array(args.features)}
    for node_id, row in zip(ids, rows):
        values = list(map(float, row[1:]))
        pose = np.eye(4)
        pose[:3, :3] = quat_to_rotation(*values[3:7])
        pose[:3, 3] = values[:3]
        k = read_camera_matrix(f'{prefix}_calib/{node_id}.yaml')
        grey = cv2.imread(f'{prefix}_rgb/{node_id}.jpg', cv2.IMREAD_GRAYSCALE)
        depth = cv2.imread(f'{prefix}_depth/{node_id}.png', cv2.IMREAD_UNCHANGED)
        feats = features.extract(grey)
        keypoints = feats['keypoints'].numpy()
        points, valid = keypoints_to_map(keypoints, depth, k, pose)
        out[f'{node_id}_keypoints'] = keypoints.astype(np.float32)
        out[f'{node_id}_descriptors'] = feats['descriptors'].numpy().astype(np.float32)
        out[f'{node_id}_points'] = points
        out[f'{node_id}_valid'] = valid
        out['image_size'] = np.array(grey.shape[::-1])
        out['camera_matrix'] = k
        print(f'keyframe {node_id}: {len(keypoints)} keypoints, {int(valid.sum())} with depth')
    np.savez_compressed(args.output, **out)
    print(f'wrote {len(ids)} {args.features} keyframes to {args.output}')


if __name__ == '__main__':
    main()
