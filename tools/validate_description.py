#!/usr/bin/env python3
"""Lightweight checks that do not require a ROS installation."""

from __future__ import annotations

import json
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


EXPECTED_MESHES = (
    "base_link.stl",
    "left_wheel_link.stl",
    "right_wheel_link.stl",
    "laser_frame.stl",
    "imu_mount_link.stl",
)


def binary_stl_stats(path):
    with path.open("rb") as stream:
        stream.read(80)
        triangle_count = struct.unpack("<I", stream.read(4))[0]
        minimum = [float("inf")] * 3
        maximum = [float("-inf")] * 3
        for _ in range(triangle_count):
            record = stream.read(50)
            if len(record) != 50:
                raise ValueError(f"Truncated binary STL: {path}")
            values = struct.unpack("<12fH", record)
            for vertex_start in (3, 6, 9):
                for axis in range(3):
                    coordinate = values[vertex_start + axis]
                    minimum[axis] = min(minimum[axis], coordinate)
                    maximum[axis] = max(maximum[axis], coordinate)

        if stream.read(1):
            raise ValueError(f"Unexpected trailing data in STL: {path}")

    return {
        "triangles": triangle_count,
        "min_mm": [round(value, 3) for value in minimum],
        "max_mm": [round(value, 3) for value in maximum],
    }


def main():
    workspace = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    package = workspace / "src" / "roam_e_description"

    ET.parse(package / "package.xml")
    ET.parse(package / "urdf" / "roam_e.urdf.xacro")
    urdf_tree = ET.parse(package / "urdf" / "roam_e.urdf")
    link_names = {
        element.attrib["name"] for element in urdf_tree.findall("link")
    }
    joint_names = {
        element.attrib["name"] for element in urdf_tree.findall("joint")
    }
    expected_links = {
        "base_footprint",
        "base_link",
        "left_wheel_link",
        "right_wheel_link",
        "laser_frame",
        "imu_mount_link",
        "imu_link",
    }
    if link_names != expected_links:
        raise ValueError(f"Unexpected URDF links: {sorted(link_names)}")
    if len(joint_names) != len(link_names) - 1:
        raise ValueError("URDF must be a single tree with links - 1 joints")

    meshes = {
        name: binary_stl_stats(package / "meshes" / name)
        for name in EXPECTED_MESHES
    }
    print(
        json.dumps(
            {
                "package": str(package),
                "links": sorted(link_names),
                "joints": sorted(joint_names),
                "meshes": meshes,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
