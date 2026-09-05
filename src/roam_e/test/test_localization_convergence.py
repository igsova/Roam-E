import math

import pytest

from roam_e.localization_convergence import (
    ConvergenceTracker,
    normalize_angle,
    planar_distance,
    yaw_from_quaternion,
)


def covariance(x=0.01, y=0.02, yaw=0.03):
    values = [0.0] * 36
    values[0] = x
    values[7] = y
    values[35] = yaw
    return values


def tracker(required_samples=3):
    return ConvergenceTracker(
        max_x_variance=0.04,
        max_y_variance=0.04,
        max_yaw_variance=0.09,
        required_samples=required_samples,
        max_position_jump=0.15,
        max_yaw_jump=0.35,
    )


def test_planar_pose_helpers_handle_quaternion_and_angle_wrap():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(-math.pi)
    assert yaw_from_quaternion(
        math.sin(math.pi / 4.0),
        math.cos(math.pi / 4.0),
    ) == pytest.approx(math.pi / 2.0)
    assert planar_distance((0.0, 0.0), (0.3, 0.4)) == pytest.approx(0.5)


def test_convergence_requires_multiple_precise_consistent_samples():
    subject = tracker(required_samples=3)

    first = subject.update(
        x=1.0,
        y=2.0,
        yaw=3.10,
        covariance=covariance(),
    )
    second = subject.update(
        x=1.01,
        y=2.01,
        yaw=-3.12,
        covariance=covariance(),
    )
    third = subject.update(
        x=1.02,
        y=2.01,
        yaw=-3.10,
        covariance=covariance(),
    )

    assert not first.converged
    assert not second.converged
    assert third.converged
    assert third.stable_samples == 3


def test_imprecise_or_jumping_pose_resets_stability():
    subject = tracker(required_samples=2)
    subject.update(
        x=0.0,
        y=0.0,
        yaw=0.0,
        covariance=covariance(),
    )

    imprecise = subject.update(
        x=0.0,
        y=0.0,
        yaw=0.0,
        covariance=covariance(yaw=0.5),
    )
    jumped = subject.update(
        x=1.0,
        y=0.0,
        yaw=0.0,
        covariance=covariance(),
    )

    assert imprecise.stable_samples == 0
    assert not imprecise.converged
    assert jumped.stable_samples == 1
    assert not jumped.converged


def test_short_covariance_is_rejected():
    quality = tracker(required_samples=1).update(
        x=0.0,
        y=0.0,
        yaw=0.0,
        covariance=[0.0] * 10,
    )

    assert not quality.converged
    assert math.isinf(quality.x_variance)


def test_converged_quality_stays_latched_until_a_new_bad_estimate():
    subject = tracker(required_samples=2)
    subject.update(
        x=0.0,
        y=0.0,
        yaw=0.0,
        covariance=covariance(),
    )
    converged = subject.update(
        x=0.01,
        y=0.0,
        yaw=0.01,
        covariance=covariance(),
    )

    assert converged.converged
    assert subject.quality.converged

    revoked = subject.update(
        x=0.01,
        y=0.0,
        yaw=0.01,
        covariance=covariance(yaw=0.5),
    )
    assert not revoked.converged
