"""Pure helpers for Roam-E motor telemetry and differential-drive odometry."""

from dataclasses import dataclass
import math


UINT32_MODULUS = 1 << 32
INT32_MIN = -(1 << 31)
INT32_MAX = (1 << 31) - 1


@dataclass(frozen=True)
class MotorTelemetry:
    controller_time_ms: int
    left_ticks: int
    right_ticks: int
    left_pwm: int
    right_pwm: int
    left_target_ticks_per_second: int = None
    right_target_ticks_per_second: int = None
    left_measured_ticks_per_second: int = None
    right_measured_ticks_per_second: int = None


@dataclass(frozen=True)
class OdometryUpdate:
    dt: float
    delta_left_ticks: int
    delta_right_ticks: int
    left_position: float
    right_position: float
    left_velocity: float
    right_velocity: float
    linear_velocity: float
    angular_velocity: float
    x: float
    y: float
    yaw: float


def parse_motor_telemetry(line):
    """Parse a firmware ``T,...`` line; ignore non-telemetry lines."""
    fields = line.strip().split(',')

    if not fields or fields[0] != 'T':
        return None
    if len(fields) not in (6, 10):
        raise ValueError(
            'telemetry must contain six (v1) or ten (v2) fields'
        )

    try:
        controller_time_ms, left_ticks, right_ticks, left_pwm, right_pwm = (
            int(value, 10) for value in fields[1:6]
        )
    except ValueError as error:
        raise ValueError('telemetry contains a non-integer field') from error

    if not 0 <= controller_time_ms < UINT32_MODULUS:
        raise ValueError('controller time is outside uint32 range')
    if not INT32_MIN <= left_ticks <= INT32_MAX:
        raise ValueError('left encoder count is outside int32 range')
    if not INT32_MIN <= right_ticks <= INT32_MAX:
        raise ValueError('right encoder count is outside int32 range')
    if not -255 <= left_pwm <= 255 or not -255 <= right_pwm <= 255:
        raise ValueError('PWM value is outside Arduino range')

    extended = [None, None, None, None]
    if len(fields) == 10:
        try:
            extended = [int(value, 10) for value in fields[6:10]]
        except ValueError as error:
            raise ValueError(
                'extended telemetry contains a non-integer field'
            ) from error

        if not all(INT32_MIN <= value <= INT32_MAX for value in extended):
            raise ValueError('wheel velocity is outside int32 range')

    return MotorTelemetry(
        controller_time_ms=controller_time_ms,
        left_ticks=left_ticks,
        right_ticks=right_ticks,
        left_pwm=left_pwm,
        right_pwm=right_pwm,
        left_target_ticks_per_second=extended[0],
        right_target_ticks_per_second=extended[1],
        left_measured_ticks_per_second=extended[2],
        right_measured_ticks_per_second=extended[3],
    )


def signed_counter_delta(current, previous):
    """Return an int32 counter delta while handling signed wraparound."""
    difference = (current - previous) % UINT32_MODULUS
    if difference >= (1 << 31):
        difference -= UINT32_MODULUS
    return difference


def unsigned_elapsed_ms(current, previous):
    """Return elapsed Arduino milliseconds while handling uint32 rollover."""
    return (current - previous) % UINT32_MODULUS


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DifferentialDriveTracker:
    """Integrate normalized forward-positive wheel encoder telemetry."""

    def __init__(
        self,
        ticks_per_revolution,
        wheel_radius,
        wheel_separation,
        max_tick_rate=10000.0,
    ):
        if ticks_per_revolution <= 0:
            raise ValueError('ticks_per_revolution must be positive')
        if wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be positive')
        if wheel_separation <= 0.0:
            raise ValueError('wheel_separation must be positive')
        if max_tick_rate <= 0.0:
            raise ValueError('max_tick_rate must be positive')

        self.ticks_per_revolution = float(ticks_per_revolution)
        self.wheel_radius = float(wheel_radius)
        self.wheel_separation = float(wheel_separation)
        self.max_tick_rate = float(max_tick_rate)
        self.radians_per_tick = 2.0 * math.pi / self.ticks_per_revolution
        self.metres_per_tick = (
            2.0 * math.pi * self.wheel_radius / self.ticks_per_revolution
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.left_position = 0.0
        self.right_position = 0.0
        self.last_rebase_reason = 'awaiting first telemetry sample'
        self.rebase_count = 0
        self._previous = None

    def _rebase(self, telemetry, reason):
        self._previous = telemetry
        self.last_rebase_reason = reason
        self.rebase_count += 1

    def update(self, telemetry):
        if self._previous is None:
            self._rebase(telemetry, 'initial telemetry sample')
            return None

        previous = self._previous

        # A lower millis value is a normal wrap only near the uint32 boundary.
        controller_restarted = (
            telemetry.controller_time_ms < previous.controller_time_ms
            and not (
                previous.controller_time_ms >= 0xF0000000
                and telemetry.controller_time_ms <= 0x0FFFFFFF
            )
        )
        if controller_restarted:
            self._rebase(telemetry, 'controller clock restarted')
            return None

        elapsed_ms = unsigned_elapsed_ms(
            telemetry.controller_time_ms,
            previous.controller_time_ms,
        )
        if elapsed_ms == 0:
            return None

        dt = elapsed_ms / 1000.0
        delta_left_ticks = signed_counter_delta(
            telemetry.left_ticks, previous.left_ticks
        )
        delta_right_ticks = signed_counter_delta(
            telemetry.right_ticks, previous.right_ticks
        )

        maximum_delta = self.max_tick_rate * dt + 2.0
        if (
            abs(delta_left_ticks) > maximum_delta
            or abs(delta_right_ticks) > maximum_delta
        ):
            self._rebase(telemetry, 'implausible encoder jump')
            return None

        self._previous = telemetry

        delta_left_angle = delta_left_ticks * self.radians_per_tick
        delta_right_angle = delta_right_ticks * self.radians_per_tick
        self.left_position += delta_left_angle
        self.right_position += delta_right_angle

        delta_left = delta_left_ticks * self.metres_per_tick
        delta_right = delta_right_ticks * self.metres_per_tick
        delta_distance = 0.5 * (delta_left + delta_right)
        delta_yaw = (delta_right - delta_left) / self.wheel_separation

        heading_midpoint = self.yaw + 0.5 * delta_yaw
        self.x += delta_distance * math.cos(heading_midpoint)
        self.y += delta_distance * math.sin(heading_midpoint)
        self.yaw = normalize_angle(self.yaw + delta_yaw)

        return OdometryUpdate(
            dt=dt,
            delta_left_ticks=delta_left_ticks,
            delta_right_ticks=delta_right_ticks,
            left_position=self.left_position,
            right_position=self.right_position,
            left_velocity=delta_left_angle / dt,
            right_velocity=delta_right_angle / dt,
            linear_velocity=delta_distance / dt,
            angular_velocity=delta_yaw / dt,
            x=self.x,
            y=self.y,
            yaw=self.yaw,
        )
