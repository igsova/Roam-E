from dataclasses import dataclass
import math


def normalize_angle(angle):
    """Return an angle in the half-open interval [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(z, w):
    """Extract planar yaw from a quaternion whose roll and pitch are zero."""
    return normalize_angle(2.0 * math.atan2(float(z), float(w)))


def planar_distance(first, second):
    """Return the Euclidean distance between two planar points."""
    return math.hypot(second[0] - first[0], second[1] - first[1])


@dataclass(frozen=True)
class LocalizationQuality:
    x_variance: float
    y_variance: float
    yaw_variance: float
    stable_samples: int
    converged: bool


class ConvergenceTracker:
    """Require several precise, mutually consistent AMCL estimates."""

    def __init__(
        self,
        *,
        max_x_variance,
        max_y_variance,
        max_yaw_variance,
        required_samples,
        max_position_jump,
        max_yaw_jump,
    ):
        if required_samples < 1:
            raise ValueError('required_samples must be at least one')

        self.max_x_variance = float(max_x_variance)
        self.max_y_variance = float(max_y_variance)
        self.max_yaw_variance = float(max_yaw_variance)
        self.required_samples = int(required_samples)
        self.max_position_jump = float(max_position_jump)
        self.max_yaw_jump = float(max_yaw_jump)
        self.reset()

    def reset(self):
        self.stable_samples = 0
        self.last_pose = None
        self.quality = LocalizationQuality(
            x_variance=math.inf,
            y_variance=math.inf,
            yaw_variance=math.inf,
            stable_samples=0,
            converged=False,
        )

    def update(self, *, x, y, yaw, covariance):
        if len(covariance) < 36:
            self.reset()
            return self.quality

        variances = (
            float(covariance[0]),
            float(covariance[7]),
            float(covariance[35]),
        )
        finite_nonnegative = all(
            math.isfinite(value) and value >= 0.0 for value in variances
        )
        precise = (
            finite_nonnegative
            and variances[0] <= self.max_x_variance
            and variances[1] <= self.max_y_variance
            and variances[2] <= self.max_yaw_variance
        )

        pose = (float(x), float(y), normalize_angle(yaw))
        consistent = True
        if self.last_pose is not None:
            consistent = (
                planar_distance(self.last_pose[:2], pose[:2])
                <= self.max_position_jump
                and abs(normalize_angle(pose[2] - self.last_pose[2]))
                <= self.max_yaw_jump
            )

        if precise and consistent:
            self.stable_samples += 1
        elif precise:
            self.stable_samples = 1
        else:
            self.stable_samples = 0

        self.last_pose = pose
        converged = self.stable_samples >= self.required_samples
        self.quality = LocalizationQuality(
            x_variance=variances[0],
            y_variance=variances[1],
            yaw_variance=variances[2],
            stable_samples=self.stable_samples,
            converged=converged,
        )
        return self.quality
