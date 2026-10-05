"""Offline safety/math tests; no Blender run, ROS or robot hardware access."""
import math
from pathlib import Path
import tempfile

import pytest

import prepare_r19_camera_forward as candidate
import check_r19_candidate_tf as tf_check


def test_native_axes_in_camera_forward_base():
    mount = candidate.validate_description.rotation([0, 0, math.pi])
    native = candidate.validate_description.rotation(candidate.IMU_RELATIVE_RPY)
    actual = candidate.compose(mount, native)
    expected = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
    assert max(abs(x-y) for aa,bb in zip(actual,expected) for x,y in zip(aa,bb)) < 1e-12


def test_existing_output_is_never_overwritten():
    with tempfile.TemporaryDirectory() as folder:
        with pytest.raises(ValueError, match='already exists'):
            candidate.prepare(Path(folder), Path('/nonexistent/blender'))


def test_source_tree_output_is_rejected():
    with pytest.raises(ValueError, match='source workspace src tree'):
        candidate.prepare(candidate.ROOT / 'src' / 'forbidden_candidate',
                          Path('/nonexistent/blender'))


def test_missing_r19_baseline_is_reported_before_creating_output(tmp_path):
    output = tmp_path / 'candidate'
    with pytest.raises(ValueError, match='--baseline-package'):
        candidate.prepare(output, Path('/nonexistent/blender'), tmp_path / 'legacy')
    assert not output.exists()


def test_global_half_turn_preserves_up_and_yaw_sign():
    half_turn = candidate.validate_description.rotation([0, 0, math.pi])
    assert half_turn[2] == [0, 0, 1]
    assert abs(half_turn[0][0] * half_turn[1][1]
               - half_turn[0][1] * half_turn[1][0] - 1) < 1e-12


def test_runtime_tf_composition():
    frames = {'imu_link': {'parent':'imu_mount_link',
                          'quaternion_xyzw':[0,0,math.sin(math.pi/4),math.cos(math.pi/4)]},
              'imu_mount_link': {'parent':'base_link',
                                 'quaternion_xyzw':[0,0,1,0]}}
    actual = tf_check.to_base(frames,'imu_link')
    expected = [[0,1,0],[-1,0,0],[0,0,1]]
    assert max(abs(x-y) for aa,bb in zip(actual,expected) for x,y in zip(aa,bb)) < 1e-12


def test_runtime_tf_rejects_cycle_and_zero_quaternion():
    with pytest.raises(ValueError,match='Zero quaternion'):
        tf_check.quaternion_rotation([0,0,0,0])
    with pytest.raises(ValueError,match='TF cycle'):
        tf_check.to_base({'imu_link': {'parent':'imu_link',
                                      'quaternion_xyzw':[0,0,0,1]}}, 'imu_link')
