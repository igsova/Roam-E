import math


FULL_TURN_RADIANS = 2.0 * math.pi


def scan_geometry(angle_increment):
    """Return the bin count and inclusive angular bounds for a full scan."""
    if not math.isfinite(angle_increment) or angle_increment <= 0.0:
        raise ValueError('angle_increment must be a finite positive value')

    num_bins = int(round(FULL_TURN_RADIANS / angle_increment))
    if num_bins < 1:
        raise ValueError('angle_increment produces no scan bins')

    angle_min = 0.0
    angle_max = angle_min + (num_bins - 1) * angle_increment
    return num_bins, angle_min, angle_max


def bin_measurements(
    measurements,
    *,
    num_bins,
    angle_increment,
    range_min,
    range_max,
):
    """Convert RPLidar quality/angle/distance samples into fixed ROS bins."""
    no_return = range_max + 1.0
    ranges = [no_return] * num_bins
    intensities = [0.0] * num_bins

    for quality, angle_deg, distance_mm in measurements:
        if not math.isfinite(angle_deg):
            continue

        angle = math.radians(angle_deg) % FULL_TURN_RADIANS
        index = int(angle / angle_increment)
        if not 0 <= index < num_bins:
            continue

        if not distance_mm or not math.isfinite(distance_mm):
            continue

        distance = distance_mm / 1000.0
        if not range_min <= distance <= range_max:
            continue

        # Multiple native measurements can fall into one output bin. Preserve
        # the closest return and the quality value belonging to that return.
        if distance < ranges[index]:
            ranges[index] = distance
            intensities[index] = float(quality)

    return ranges, intensities
