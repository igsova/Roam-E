"""Pure differential-drive command conversion and serial formatting helpers."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WheelVelocityTargets:
    requested_linear: float
    requested_angular: float
    limited_linear: float
    limited_angular: float
    left_ticks_per_second: int
    right_ticks_per_second: int
    limited: bool


def _clamp(value, lower, upper):
    return max(lower, min(value, upper))


def twist_to_wheel_targets(
    linear_velocity,
    angular_velocity,
    wheel_separation,
    metres_per_tick,
    max_linear_velocity,
    max_angular_velocity,
    max_wheel_ticks_per_second,
):
    """Convert a body twist to curvature-preserving integer wheel targets."""
    values = (
        linear_velocity,
        angular_velocity,
        wheel_separation,
        metres_per_tick,
        max_linear_velocity,
        max_angular_velocity,
        max_wheel_ticks_per_second,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('motor command parameters must be finite')
    if wheel_separation <= 0.0 or metres_per_tick <= 0.0:
        raise ValueError('wheel geometry must be positive')
    if (
        max_linear_velocity <= 0.0
        or max_angular_velocity <= 0.0
        or max_wheel_ticks_per_second <= 0.0
    ):
        raise ValueError('motor command limits must be positive')

    limited_linear = _clamp(
        linear_velocity,
        -max_linear_velocity,
        max_linear_velocity,
    )
    limited_angular = _clamp(
        angular_velocity,
        -max_angular_velocity,
        max_angular_velocity,
    )

    half_track = 0.5 * wheel_separation
    left = (limited_linear - limited_angular * half_track) / metres_per_tick
    right = (limited_linear + limited_angular * half_track) / metres_per_tick

    largest = max(abs(left), abs(right))
    if largest > max_wheel_ticks_per_second:
        scale = max_wheel_ticks_per_second / largest
        left *= scale
        right *= scale

    left_rounded = int(round(left))
    right_rounded = int(round(right))
    limited = (
        limited_linear != linear_velocity
        or limited_angular != angular_velocity
        or largest > max_wheel_ticks_per_second
    )

    return WheelVelocityTargets(
        requested_linear=linear_velocity,
        requested_angular=angular_velocity,
        limited_linear=limited_linear,
        limited_angular=limited_angular,
        left_ticks_per_second=left_rounded,
        right_ticks_per_second=right_rounded,
        limited=limited,
    )


def velocity_command(left_ticks_per_second, right_ticks_per_second):
    return (
        f'VEL,{int(left_ticks_per_second)},{int(right_ticks_per_second)}\n'
    ).encode('ascii')


def pid_command(kp, ki, kv, static_pwm):
    values = (kp, ki, kv, static_pwm)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('PID parameters must be finite')

    kp_milli = int(round(1000.0 * kp))
    ki_milli = int(round(1000.0 * ki))
    kv_milli = int(round(1000.0 * kv))
    static_pwm = int(static_pwm)

    if not 0 <= kp_milli <= 1000:
        raise ValueError('kp must be in [0, 1]')
    if not 0 <= ki_milli <= 2000:
        raise ValueError('ki must be in [0, 2]')
    if not 0 <= kv_milli <= 1000:
        raise ValueError('kv must be in [0, 1]')
    if not 0 <= static_pwm <= 160:
        raise ValueError('static_pwm must be in [0, 160]')

    return (
        f'PID,{kp_milli},{ki_milli},{kv_milli},{static_pwm}\n'
    ).encode('ascii')


def feedforward_command(
    left_forward_pwm,
    left_reverse_pwm,
    right_forward_pwm,
    right_reverse_pwm,
):
    values = (
        left_forward_pwm,
        left_reverse_pwm,
        right_forward_pwm,
        right_reverse_pwm,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('feed-forward values must be finite')

    integer_values = tuple(int(value) for value in values)
    if any(value < 0 or value > 160 for value in integer_values):
        raise ValueError('feed-forward PWM values must be in [0, 160]')

    return ('FF,{},{},{},{}\n'.format(*integer_values)).encode('ascii')
