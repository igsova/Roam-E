#!/usr/bin/env python3
"""Export ROS-oriented meshes from the saved Roam-E FreeCAD assembly.

Run with FreeCAD's Python interpreter. FreeCAD geometry is in millimetres.
The output meshes remain in millimetres and the URDF supplies a 0.001 scale.
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

FREECAD_RESOURCES = Path("/Applications/FreeCAD.app/Contents/Resources")
sys.path.insert(0, str(FREECAD_RESOURCES / "lib"))
sys.path.insert(0, str(FREECAD_RESOURCES / "Mod"))

import FreeCAD as App  # noqa: E402
import MeshPart  # noqa: E402
import Part  # noqa: E402


ROOT_LABEL = "BumperBotLaser"
CATEGORY_LABELS = {
    "left_wheel_link": "left_wheel_link",
    "right_wheel_link": "right_wheel_link",
    "laser_frame": "RP-lidar-A1-1",
    "imu_mount_link": "MCU",
}


def ancestors(obj):
    """Return all parents exposed through FreeCAD's reverse dependency graph."""
    found = []
    pending = list(obj.InList)
    seen = set()
    while pending:
        parent = pending.pop()
        if parent.Name in seen:
            continue
        seen.add(parent.Name)
        found.append(parent)
        pending.extend(parent.InList)
    return found


def bbox_center(obj):
    center = obj.Shape.BoundBox.Center
    return (center.x, center.y, center.z)


def ros_transform(origin_cad_mm):
    """Map CAD (x forward, y up, z right) to ROS (x forward, y left, z up)."""
    ox, oy, oz = origin_cad_mm
    matrix = App.Matrix()
    matrix.A11, matrix.A12, matrix.A13, matrix.A14 = 1.0, 0.0, 0.0, -ox
    matrix.A21, matrix.A22, matrix.A23, matrix.A24 = 0.0, 0.0, -1.0, oz
    matrix.A31, matrix.A32, matrix.A33, matrix.A34 = 0.0, 1.0, 0.0, -oy
    matrix.A41, matrix.A42, matrix.A43, matrix.A44 = 0.0, 0.0, 0.0, 1.0
    return matrix


def to_ros_relative(point_cad_mm, base_cad_mm):
    px, py, pz = point_cad_mm
    bx, by, bz = base_cad_mm
    return (px - bx, -(pz - bz), py - by)


def export_mesh(shapes, origin_cad_mm, destination):
    if not shapes:
        raise RuntimeError(f"No shapes selected for {destination.name}")

    compound = Part.makeCompound([shape.copy() for shape in shapes])
    compound.transformShape(ros_transform(origin_cad_mm), True)
    mesh = MeshPart.meshFromShape(
        Shape=compound,
        # RViz visualization meshes should be light enough for the Pi 3. The
        # model is only ~220 mm tall, so these values retain its silhouette
        # while avoiding hundreds of thousands of tiny screw/thread facets.
        LinearDeflection=1.0,
        AngularDeflection=0.70,
        Relative=False,
    )
    mesh.write(str(destination))
    return {
        "source_shape_count": len(shapes),
        "facet_count": mesh.CountFacets,
        "bytes": destination.stat().st_size,
    }


def main():
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: export_freecad_meshes.py MODEL.FCStd ROS_PACKAGE_DIRECTORY"
        )

    model_path = Path(sys.argv[1]).expanduser().resolve()
    package_dir = Path(sys.argv[2]).expanduser().resolve()
    mesh_dir = package_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    doc = App.openDocument(str(model_path))
    try:
        by_label = {obj.Label: obj for obj in doc.Objects}
        missing = [
            label
            for label in [ROOT_LABEL, *CATEGORY_LABELS.values()]
            if label not in by_label
        ]
        if missing:
            raise RuntimeError(f"Missing required FreeCAD labels: {missing}")

        left_center = bbox_center(by_label["left_wheel_link"])
        right_center = bbox_center(by_label["right_wheel_link"])
        base_center = tuple(
            (left_center[index] + right_center[index]) / 2.0 for index in range(3)
        )

        # Use the circular head center for x/z and the laser solid for scan height.
        head = next(
            obj for obj in doc.Objects if "/head.ipt-1-solid1" in obj.Label
        )
        laser = next(
            obj for obj in doc.Objects if "/laser.ipt-1-solid1" in obj.Label
        )
        head_center = bbox_center(head)
        laser_center = bbox_center(laser)
        laser_origin = (head_center[0], laser_center[1], head_center[2])
        imu_origin = bbox_center(by_label["MCU"])

        categorized = {name: [] for name in CATEGORY_LABELS}
        base_shapes = []

        for obj in doc.Objects:
            if obj.TypeId != "Part::Feature":
                continue
            if not hasattr(obj, "Shape") or obj.Shape.isNull():
                continue

            labels = {parent.Label for parent in ancestors(obj)}
            if ROOT_LABEL not in labels:
                continue

            category = next(
                (
                    name
                    for name, label in CATEGORY_LABELS.items()
                    if label in labels
                ),
                None,
            )
            if category is None:
                base_shapes.append(obj.Shape)
            else:
                categorized[category].append(obj.Shape)

        # Named LinkGroups can be true instances (the right wheel reuses linked
        # source geometry), so export their resolved aggregate Shape rather
        # than assuming each group owns Part::Feature children.
        stats = {
            "base_link": export_mesh(
                base_shapes, base_center, mesh_dir / "base_link.stl"
            ),
            "left_wheel_link": export_mesh(
                [by_label["left_wheel_link"].Shape],
                left_center,
                mesh_dir / "left_wheel_link.stl",
            ),
            "right_wheel_link": export_mesh(
                [by_label["right_wheel_link"].Shape],
                right_center,
                mesh_dir / "right_wheel_link.stl",
            ),
            "laser_frame": export_mesh(
                [by_label["RP-lidar-A1-1"].Shape],
                laser_origin,
                mesh_dir / "laser_frame.stl",
            ),
            "imu_mount_link": export_mesh(
                [by_label["MCU"].Shape],
                imu_origin,
                mesh_dir / "imu_mount_link.stl",
            ),
        }

        metadata = {
            "source": str(model_path),
            "source_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
            "cad_to_ros": {
                "x_ros": "+x_cad",
                "y_ros": "-z_cad",
                "z_ros": "+y_cad",
            },
            "base_link_origin_cad_mm": base_center,
            "wheel_centers_cad_mm": {
                "left": left_center,
                "right": right_center,
            },
            "fixed_origins_ros_mm": {
                "laser_frame": to_ros_relative(laser_origin, base_center),
                "imu_mount_link": to_ros_relative(imu_origin, base_center),
            },
            "meshes": stats,
        }
        metadata_path = package_dir / "config" / "mesh_export.json"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
        print(json.dumps(metadata, indent=2))
    finally:
        App.closeDocument(doc.Name)


if __name__ == "__main__":
    main()
