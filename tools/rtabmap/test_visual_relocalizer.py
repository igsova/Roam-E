"""Unit tests for the visual relocalizer helpers (no camera, no torch).

Run on the Pi with ROS sourced:  python3 -m pytest -q tools/rtabmap/test_visual_relocalizer.py
"""
import math

import numpy as np
import pytest

from visual_keyframes import keypoints_to_map, quat_to_rotation
from visual_relocalizer import planar_matrix, planar_pose, pose_difference


def test_keypoints_back_project_into_the_map_frame():
    k = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
    depth = np.zeros((480, 640), np.uint16)
    depth[240, 320] = 2000      # centre pixel, 2 m
    depth[240, 420] = 1000      # 100 px right, 1 m
    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 0.5]
    keypoints = np.array([[320.0, 240.0], [420.0, 240.0], [10.0, 10.0]])
    points, valid = keypoints_to_map(keypoints, depth, k, pose)
    assert valid.tolist() == [True, True, False]
    assert points[0] == pytest.approx([1.0, 2.0, 2.5])
    assert points[1] == pytest.approx([1.2, 2.0, 1.5])
    assert points[2].tolist() == [0.0, 0.0, 0.0]


def test_quaternion_rotation_matches_yaw():
    r = quat_to_rotation(0.0, 0.0, math.sin(0.5), math.cos(0.5))
    assert np.allclose(r[:2, :2], [[math.cos(1.0), -math.sin(1.0)],
                                   [math.sin(1.0), math.cos(1.0)]])


def test_planar_pose_round_trip_and_difference():
    pose = (0.4, -1.2, 2.5)
    assert planar_pose(planar_matrix(*pose)) == pytest.approx(pose)
    dist, d_yaw = pose_difference((0.0, 0.0, math.pi - 0.05), (0.3, 0.4, -math.pi + 0.05))
    assert dist == pytest.approx(0.5)
    assert d_yaw == pytest.approx(0.1)
