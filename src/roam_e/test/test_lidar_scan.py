import math

from roam_e.lidar_scan import (
    bin_measurements,
    rplidar_angle_to_ros,
    scan_geometry,
)


def assert_angle_close(actual, expected):
    difference = math.atan2(
        math.sin(actual - expected),
        math.cos(actual - expected),
    )
    assert math.isclose(difference, 0.0, abs_tol=1e-12)


def test_rplidar_native_axes_are_converted_to_ros_axes():
    # Native 180 degrees is physically forward on this RPLidar A1, while
    # native angles 90 and 270 degrees are left and right respectively.
    assert_angle_close(rplidar_angle_to_ros(180.0), 0.0)
    assert_angle_close(rplidar_angle_to_ros(90.0), math.pi / 2.0)
    assert_angle_close(rplidar_angle_to_ros(0.0), math.pi)
    assert_angle_close(rplidar_angle_to_ros(270.0), 3.0 * math.pi / 2.0)


def test_bin_measurements_places_native_samples_in_ros_quadrants():
    angle_increment = math.pi / 2.0
    num_bins, _, _ = scan_geometry(angle_increment)

    ranges, intensities = bin_measurements(
        [
            (11, 180.0, 1000.0),
            (22, 90.0, 2000.0),
            (33, 0.0, 3000.0),
            (44, 270.0, 4000.0),
        ],
        num_bins=num_bins,
        angle_increment=angle_increment,
        range_min=0.15,
        range_max=12.0,
    )

    assert ranges == [1.0, 2.0, 3.0, 4.0]
    assert intensities == [11.0, 22.0, 33.0, 44.0]


def test_closest_return_wins_after_angle_conversion():
    ranges, intensities = bin_measurements(
        [(5, 180.0, 900.0), (9, 180.0, 700.0)],
        num_bins=360,
        angle_increment=math.radians(1.0),
        range_min=0.15,
        range_max=12.0,
    )

    assert ranges[0] == 0.7
    assert intensities[0] == 9.0
