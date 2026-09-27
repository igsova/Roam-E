"""Render the exported mesh assembly for visual QA with Blender 4.x."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector


def material(name, rgba, metallic=0.0, roughness=0.55):
    result = bpy.data.materials.new(name)
    result.diffuse_color = rgba
    result.metallic = metallic
    result.roughness = roughness
    return result


def import_stl(path, name, location, color):
    bpy.ops.wm.stl_import(filepath=str(path))
    obj = bpy.context.object
    obj.name = name
    obj.scale = (0.001, 0.001, 0.001)
    obj.location = location
    obj.data.materials.append(color)
    return obj


def point_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat("-Z", "Y").to_euler()


args = sys.argv[sys.argv.index("--") + 1 :]
package = Path(args[0]).resolve()
output = Path(args[1]).resolve()
mesh_dir = package / "meshes"

bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)

dark = material("chassis", (0.10, 0.12, 0.15, 1.0), metallic=0.25)
rubber = material("wheel", (0.018, 0.022, 0.026, 1.0), roughness=0.8)
lidar = material("lidar", (0.055, 0.065, 0.075, 1.0), roughness=0.35)
sensor = material("sensor", (0.03, 0.36, 0.12, 1.0), roughness=0.5)

import_stl(mesh_dir / "base_link.stl", "base_link", (0, 0, 0), dark)
import_stl(
    mesh_dir / "left_wheel_link.stl",
    "left_wheel_link",
    (0, 0.173630 / 2.0, 0),
    rubber,
)
import_stl(
    mesh_dir / "right_wheel_link.stl",
    "right_wheel_link",
    (0, -0.173630 / 2.0, 0),
    rubber,
)
import_stl(
    mesh_dir / "laser_frame.stl",
    "laser_frame",
    (0.017447, -0.000870, 0.176897),
    lidar,
)
import_stl(
    mesh_dir / "imu_mount_link.stl",
    "imu_mount_link",
    (0.024045, 0.0, 0.102297),
    sensor,
)

bpy.ops.mesh.primitive_plane_add(size=1.5, location=(0, 0, -0.033875))
ground = bpy.context.object
ground.data.materials.append(material("ground", (0.035, 0.04, 0.05, 1.0), roughness=0.95))

bpy.ops.object.light_add(type="AREA", location=(0.25, -0.25, 0.65))
bpy.context.object.data.energy = 45
bpy.context.object.data.shape = "DISK"
bpy.context.object.data.size = 0.5

bpy.ops.object.light_add(type="AREA", location=(-0.35, 0.35, 0.3))
bpy.context.object.data.energy = 25
bpy.context.object.data.size = 0.35

bpy.ops.object.camera_add(location=(0.48, -0.52, 0.34))
camera = bpy.context.object
point_at(camera, (0.005, 0.0, 0.08))
camera.data.lens = 58
bpy.context.scene.camera = camera

scene = bpy.context.scene
scene.render.engine = "BLENDER_EEVEE"
scene.render.resolution_x = 1100
scene.render.resolution_y = 800
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.filepath = str(output)
scene.render.film_transparent = False
scene.world.color = (0.012, 0.014, 0.02)
scene.render.image_settings.color_mode = "RGBA"
scene.view_settings.look = "AgX - Medium High Contrast"
scene.view_settings.exposure = -1.0

bpy.ops.render.render(write_still=True)
