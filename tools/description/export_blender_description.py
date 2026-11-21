"""Export the Roam-E visual meshes and CAD-nominal frames from the Blender model.

Run it inside Blender; script options go after ``--``:

    blender --background --python tools/description/export_blender_description.py
    blender --background --python tools/description/export_blender_description.py -- --help

The script opens mechanical/roam_e.blend, never saves it, and writes into
src/roam_e_description:

- meshes/visual/<link>_<colour id>.stl: one binary STL per link and colour, in metres
  and in the frame of that link;
- config/mesh_manifest.json: the CAD-nominal frames, the mesh list (triangle counts,
  bounds, hashes), the wheel data and the footprint hull. build_description.py turns
  it into the URDF.

CAD coordinates are millimetres with +Z up. The camera looks along CAD -Y and the
caster sits towards CAD +Y. Output follows REP-103 in base_link: +X forward, +Y left.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from collections import defaultdict
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[2]

# The .blend has a unit scale of 0.1, so one Blender unit is 0.1 m (100 mm).
BLENDER_UNIT_SCALE = 0.1
MM_PER_BLENDER_UNIT = 100
MM_PER_M = 1000

# base_link origin in CAD millimetres: the middle of the wheel axle.
BASE_LINK_CAD_MM = (0, -37, 32.5)

# CAD -> base_link rotations for each choice of forward direction. Both are proper
# rotations (determinant +1), so the meshes are never mirrored.
CAD_TO_BASE = {
    'camera': ((0, -1, 0), (1, 0, 0), (0, 0, 1)),  # +X = CAD -Y
    'caster': ((0, 1, 0), (-1, 0, 0), (0, 0, 1)),  # +X = CAD +Y
}

# Wheels in CAD: centres at x = +/-88.3 mm on the axle, tyre radius 32.5 mm.
CAD_WHEEL_X_MM = 88.3
CAD_WHEEL_RADIUS_M = 0.0325
# Measured on the robot. The tyre meshes are scaled radially to this radius and the
# wheel joints are placed at this separation (centre to centre).
PHYSICAL_WHEEL_RADIUS_M = 0.033875
PHYSICAL_WHEEL_SEPARATION_M = 0.173630
PHYSICAL_WHEEL_WIDTH_M = 0.029
# Floor-calibrated values used by the encoder odometry. Recorded for reference only;
# the URDF uses the physical values above.
ODOMETRY_WHEEL_RADIUS_M = 0.033975
ODOMETRY_WHEEL_SEPARATION_M = 0.175150
# Connected parts of a motor-wheel object that span more than this in CAD Y and Z
# are the tyre and hub, which rotate. The rest is the motor, fixed to base_link.
ROTATING_PART_MIN_EXTENT_MM = 50

# camera_link axes in CAD coordinates (matrix columns): +X along the optical axis
# (CAD -Y), +Y to the camera's left (CAD +X), +Z up. This is the OpenNI convention.
CAMERA_LINK_AXES_CAD = ((0, 1, 0), (-1, 0, 0), (0, 0, 1))
# Optical frame axes in camera_link axes (columns): +X right, +Y down, +Z forward.
OPTICAL_AXES = ((0, 0, 1), (-1, 0, 0), (0, -1, 0))

# The laser and IMU-mount axes are fixed to the hardware and were defined with the
# caster forward. With the camera forward, base_link is turned by pi about Z, so
# these frames get a yaw of pi relative to it.
NOMINAL_SENSOR_YAW = {'camera': math.pi, 'caster': 0.0}

# Blender object names. They come from the .blend and cannot be changed here.
STUDIO_COLLECTION = '06 Studio'
LIDAR_ROTOR = 'R15 LiDAR | Rotating body 70.26 mm'
IMU_BOARD = 'IMU | Adafruit BNO055 Rev C reference'
IMU_PACKAGE = 'IMU | BNO055 package'
DEPTH_LENS = 'Depth_Coated_Lens'
OPTICAL_FRAME_LENSES = (
    ('camera_rgb_optical_frame', 'RGB_Coated_Lens'),
    ('camera_depth_optical_frame', DEPTH_LENS),
    ('camera_ir_optical_frame', DEPTH_LENS),
)
WHEEL_OBJECTS = ('Component | Motor_Wheel', 'Component | Motor_Wheel_Left')
TOF_PCB = 'ToF {label} | Reference PCB 25x10.66 R09'
TOF_OPTIC = 'ToF {label} | VL53L0X optical package'
TOF_CAD_LABELS = ('Front_L', 'Front_C', 'Front_R', 'Rear_L', 'Rear_C', 'Rear_R', 'Left', 'Right')
# ToF link names in the order they are written, front to back and left to right.
TOF_LINKS = tuple(f'tof_{label.lower()}_link' for label in TOF_CAD_LABELS)

# Object name prefixes that select the link a mesh is attached to.
LIDAR_PREFIX = 'R15 LiDAR |'
TOF_PREFIX = 'ToF '
IMU_MOUNT_PREFIXES = ('IMU | Adafruit', 'IMU | BNO055', 'IMU | Header pad')
CAMERA_PREFIXES = ('RGB_', 'IR_Projector_', 'Depth_', 'Front_')
CAMERA_OBJECTS = (
    'Housing_Silver_Capsule',
    'Rear_Black_Inset',
    'Right_Small_Front_Aperture',
    'Left_Small_Front_Aperture',
)
# Body panels get their own links so they can be hidden in RViz without hiding the
# sensors. They are matched by part number, the text before ' |'.
PANEL_LINKS = {
    'P15B': 'hood_side_link',
    'P15C': 'hood_front_link',
    'P15D': 'hood_rear_link',
    'P16B': 'skirt_side_link',
    'P16C': 'skirt_front_link',
    'P16D': 'skirt_rear_link',
    'P05': 'battery_drawer_link',
}
LARGE_PART_PREFIXES = ('P15', 'P16')
LARGE_PART_OBJECTS = ('Component | Caster',)
LARGE_PART_MARKER = 'Rotating body'

# Visual level of detail. Coplanar faces are merged first, then each object is
# decimated to a triangle budget that depends on its size.
PLANAR_ANGLE_DEG = 0.5
LARGE_PART_TRIANGLES = 18000
SMALL_PART_TRIANGLES = 350
PART_TRIANGLES = 2000
SMALL_PART_MAX_EXTENT_MM = 15
# A decimated object must keep its bounding box within this distance of the source.
# If it does not, the budget is raised from LOD_RETRY_FIRST, doubling up to LOD_RETRY_LAST.
LOD_BOUNDS_TOLERANCE_M = 0.0005
LOD_RETRY_FIRST = 4000
LOD_RETRY_LAST = 64000

DEFAULT_RGBA = (0.12, 0.14, 0.16, 1.0)  # faces without a material
COLOUR_DECIMALS = 3
FRAME_DECIMALS = 9
FOOTPRINT_DECIMALS = 6
# Triangles with |AB x AC|^2 below this (m^4) have no area and are dropped.
MIN_CROSS_PRODUCT_SQ = 1e-24
STL_HEADER = b'Roam-E visual mesh; metres; not for manufacturing'.ljust(80)
TEMP_NAME = '_export temporary'
PROGRESS_INTERVAL = 100

CAD_NOMINAL = 'CAD nominal'
PANEL_NOTE = 'Body panel; shares the base_link origin'

LIMITS = [
    'Sensor frames are CAD nominal; calibration.yaml holds the measured values.',
    'Visual and TF model: no inertia, no simulated sensors, wheel collisions only.',
    'ToF names follow REP-103 (l and left are +Y); I2C channels are not modelled.',
]


class CadToBase:
    """Converts between CAD millimetres and base_link metres."""

    def __init__(self, forward: str):
        self.rotation = Matrix(CAD_TO_BASE[forward])
        self.origin = Vector(BASE_LINK_CAD_MM)

    def to_base(self, point_mm) -> Vector:
        return self.rotation @ (Vector(point_mm) - self.origin) / MM_PER_M

    def to_blender(self, point_m: Vector) -> Vector:
        """Map a base_link point back to Blender units in the CAD frame."""
        cad_mm = self.origin + self.rotation.transposed() @ (point_m * MM_PER_M)
        return cad_mm / MM_PER_BLENDER_UNIT


class Frames:
    """CAD-nominal frames: manifest records plus base_link poses of mesh-bearing links."""

    def __init__(self):
        self.records = {}
        self.poses = {'base_link': Matrix.Identity(4)}

    def add(self, name, parent, xyz, rpy, note):
        self.records[name] = {
            'parent': parent,
            'xyz': [round(v, FRAME_DECIMALS) for v in xyz],
            'rpy': [round(v, FRAME_DECIMALS) for v in rpy],
            'note': note,
        }

    def add_in_base(self, name, xyz, rotation, note=CAD_NOMINAL):
        """Add a child of base_link at xyz (metres) with a 3x3 rotation."""
        pose = rotation.to_4x4()
        pose.translation = xyz
        self.poses[name] = pose
        self.add(name, 'base_link', xyz, rotation.to_euler('XYZ'), note)


def parse_args() -> argparse.Namespace:
    argv = sys.argv[sys.argv.index('--') + 1 :] if '--' in sys.argv else []
    parser = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    parser.add_argument('--source', type=Path, default=ROOT / 'mechanical/roam_e.blend')
    parser.add_argument('--package', type=Path, default=ROOT / 'src/roam_e_description')
    parser.add_argument(
        '--forward',
        choices=sorted(CAD_TO_BASE),
        default='camera',
        help='Which end of the robot is +X in base_link (default: camera)',
    )
    return parser.parse_args(argv)


def bounds(points) -> list[list[float]]:
    """Axis-aligned bounds as [[xmin, xmax], [ymin, ymax], [zmin, zmax]]."""
    return [[min(p[i] for p in points), max(p[i] for p in points)] for i in range(3)]


def bounds_change(a, b) -> float:
    return max(abs(a[i][j] - b[i][j]) for i in range(3) for j in range(2))


def centre_mm(obj) -> Vector:
    """World-space centre of an object's bounding box, in CAD millimetres."""
    corners = [obj.matrix_world @ Vector(corner) * MM_PER_BLENDER_UNIT for corner in obj.bound_box]
    return Vector([(low + high) / 2 for low, high in bounds(corners)])


def tof_link_name(cad_label: str, forward: str) -> str:
    """Return the REP-103 link name of the ToF station with a CAD label like 'Front_L'.

    The CAD labels describe the robot as seen from in front of its camera: 'Front' is
    the camera end and 'L'/'Left' is that viewer's left, CAD -X. ROS names use the
    robot's own left (+Y), so with the camera forward the sides swap, and with the
    caster forward the ends swap.
    """
    if forward == 'camera':
        swap = {'l': 'r', 'r': 'l', 'left': 'right', 'right': 'left'}
    else:
        swap = {'front': 'rear', 'rear': 'front'}
    words = cad_label.lower().split('_')
    return 'tof_' + '_'.join(swap.get(word, word) for word in words) + '_link'


def visible_meshes(scene) -> list:
    """Mesh objects shown in renders, without the studio backdrop."""
    return [
        obj
        for obj in scene.objects
        if obj.type == 'MESH'
        and not obj.hide_render
        and not obj.hide_get()
        and not any(c.name == STUDIO_COLLECTION for c in obj.users_collection)
    ]


def panel_links(objects) -> dict[str, str]:
    """Map the object name of each body panel to its link."""
    links = {}
    for obj in objects:
        part = obj.name.split(' |')[0]
        if part in PANEL_LINKS:
            links[obj.name] = PANEL_LINKS[part]
    missing = set(PANEL_LINKS.values()) - set(links.values())
    if missing:
        raise RuntimeError(f'No object found for panel links {sorted(missing)}')
    return links


def add_sensor_frames(scene, cad: CadToBase, forward: str, frames: Frames) -> dict[str, str]:
    """Add the panel, LiDAR, IMU, camera and ToF frames.

    Returns the ToF link name for each CAD station label.
    """
    objects = scene.objects
    for link in PANEL_LINKS.values():
        frames.poses[link] = Matrix.Identity(4)
        frames.add(link, 'base_link', (0, 0, 0), (0, 0, 0), PANEL_NOTE)

    sensor_basis = Matrix.Rotation(NOMINAL_SENSOR_YAW[forward], 3, 'Z')
    frames.add_in_base(
        'laser_frame',
        cad.to_base(centre_mm(objects[LIDAR_ROTOR])),
        sensor_basis,
        'CAD midpoint of the LiDAR rotor; scan plane and yaw not measured',
    )
    frames.add_in_base('imu_mount_link', cad.to_base(centre_mm(objects[IMU_BOARD])), sensor_basis)
    imu_xyz = cad.to_base(centre_mm(objects[IMU_PACKAGE]))
    frames.add(
        'imu_link',
        'imu_mount_link',
        imu_xyz - frames.poses['imu_mount_link'].translation,
        (math.pi, 0, 0),
        'CAD centre of the BNO055 package; chip axes not measured',
    )

    camera_axes = cad.rotation @ Matrix(CAMERA_LINK_AXES_CAD)
    frames.add_in_base(
        'camera_link',
        cad.to_base(centre_mm(objects[DEPTH_LENS])),
        camera_axes,
        'CAD centre of the depth lens; mount angles not measured',
    )
    camera_origin = frames.poses['camera_link'].translation
    optical_rpy = Matrix(OPTICAL_AXES).to_euler('XYZ')
    for link, lens in OPTICAL_FRAME_LENSES:
        lens_xyz = cad.to_base(centre_mm(objects[lens]))
        frames.add(
            link,
            'camera_link',
            camera_axes.transposed() @ (lens_xyz - camera_origin),
            optical_rpy,
            'CAD lens centre with REP-103 optical axes',
        )

    tof_links = {label: tof_link_name(label, forward) for label in TOF_CAD_LABELS}
    labels = {link: label for label, link in tof_links.items()}
    up = Vector((0, 0, 1))
    for link in TOF_LINKS:
        label = labels[link]
        pcb = objects[TOF_PCB.format(label=label)]
        # The PCB's local +Z is the beam direction. Level it: +X out along the beam,
        # +Z up, +Y to the left of the beam.
        beam = (cad.rotation @ pcb.matrix_world.to_3x3().col[2]).normalized()
        left = up.cross(beam).normalized()
        beam = left.cross(up)
        rotation = Matrix((beam, left, up)).transposed()
        optic = objects[TOF_OPTIC.format(label=label)]
        frames.add_in_base(link, cad.to_base(centre_mm(optic)), rotation)
    return tof_links


def link_for(name: str, panels: dict[str, str], tof_links: dict[str, str]) -> str:
    """Return the link an object's mesh is attached to."""
    if name.startswith(LIDAR_PREFIX):
        return 'laser_frame'
    if name.startswith(IMU_MOUNT_PREFIXES):
        return 'imu_mount_link'
    if name.startswith(CAMERA_PREFIXES) or name in CAMERA_OBJECTS:
        return 'camera_link'
    if name.startswith(TOF_PREFIX):
        return tof_links[name.split(' |')[0][len(TOF_PREFIX) :]]
    return panels.get(name, 'base_link')


def is_large_part(name: str) -> bool:
    return (
        name.startswith(LARGE_PART_PREFIXES)
        or name in LARGE_PART_OBJECTS
        or LARGE_PART_MARKER in name
    )


def connected_components(mesh) -> list[set[int]]:
    """Vertex indices of each edge-connected part of a mesh."""
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.verts.ensure_lookup_table()
    pending = set(bm.verts)
    parts = []
    while pending:
        stack = [pending.pop()]
        part = set()
        while stack:
            vert = stack.pop()
            part.add(vert.index)
            for edge in vert.link_edges:
                other = edge.other_vert(vert)
                if other in pending:
                    pending.remove(other)
                    stack.append(other)
        parts.append(part)
    bm.free()
    return parts


def submesh(mesh, indices: set[int]):
    """New mesh with the given vertices and the faces that use only those vertices."""
    ids = sorted(indices)
    new_index = {old: new for new, old in enumerate(ids)}
    faces = [p for p in mesh.polygons if all(i in indices for i in p.vertices)]
    out = bpy.data.meshes.new(TEMP_NAME)
    out.from_pydata(
        [mesh.vertices[i].co for i in ids],
        [],
        [tuple(new_index[i] for i in p.vertices) for p in faces],
    )
    for material in mesh.materials:
        out.materials.append(material)
    for new_face, old_face in zip(out.polygons, faces, strict=True):
        new_face.material_index = old_face.material_index
    out.update()
    return out


def split_wheel(obj, mesh, cad: CadToBase, frames: Frames):
    """Split a motor-wheel object into the fixed motor and the rotating tyre and hub.

    Adds the wheel link frame at the measured separation and moves the tyre into that
    frame, scaled radially from the CAD radius to the measured radius.
    Returns ``(chunks, record)``.
    """
    rotating = set()
    for part in connected_components(mesh):
        extent = bounds([mesh.vertices[i].co * MM_PER_BLENDER_UNIT for i in part])
        if (
            extent[1][1] - extent[1][0] > ROTATING_PART_MIN_EXTENT_MM
            and extent[2][1] - extent[2][0] > ROTATING_PART_MIN_EXTENT_MM
        ):
            rotating.update(part)
    tyre = submesh(mesh, rotating)
    motor = submesh(mesh, set(range(len(mesh.vertices))) - rotating)

    wheel_x = -CAD_WHEEL_X_MM if 'Left' in obj.name else CAD_WHEEL_X_MM
    wheel_centre = Vector((wheel_x, BASE_LINK_CAD_MM[1], BASE_LINK_CAD_MM[2]))
    side = 'left' if cad.to_base(wheel_centre).y > 0 else 'right'
    link = f'{side}_wheel_link'
    sign = 1 if side == 'left' else -1
    joint_origin = Vector((0, PHYSICAL_WHEEL_SEPARATION_M / 2 * sign, 0))
    frames.poses[link] = Matrix.Translation(joint_origin)
    frames.add(
        link,
        'base_link',
        joint_origin,
        (0, 0, 0),
        'Measured wheel separation; CAD tyre scaled to the measured radius',
    )

    radial_scale = PHYSICAL_WHEEL_RADIUS_M / CAD_WHEEL_RADIUS_M
    centre_m = cad.to_base(wheel_centre)
    for vert in tyre.vertices:
        p = cad.to_base(vert.co * MM_PER_BLENDER_UNIT) - centre_m
        p.x *= radial_scale
        p.z *= radial_scale
        # Map back to the CAD frame so the common export path places it in the link.
        vert.co = cad.to_blender(p + joint_origin)

    record = {
        'object': obj.name,
        'wheel_link': link,
        'rotating_vertices': len(rotating),
        'cad_centre_mm': list(wheel_centre),
        'cad_radius_m': CAD_WHEEL_RADIUS_M,
        'physical_radius_m': PHYSICAL_WHEEL_RADIUS_M,
        'cad_separation_m': round(2 * CAD_WHEEL_X_MM / MM_PER_M, 6),
        'physical_separation_m': PHYSICAL_WHEEL_SEPARATION_M,
    }
    return [(motor, 'base_link', False), (tyre, link, True)], record


def decimated(mesh, large: bool, budget: int | None = None):
    """Visual LOD: planar dissolve, then decimate to a triangle budget."""
    obj = bpy.data.objects.new(TEMP_NAME, mesh)
    bpy.context.scene.collection.objects.link(obj)
    dissolve = obj.modifiers.new('Planar dissolve', 'DECIMATE')
    dissolve.decimate_type = 'DISSOLVE'
    dissolve.angle_limit = math.radians(PLANAR_ANGLE_DEG)
    dissolve.delimit = {'MATERIAL'}
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    flat = bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph))
    obj.modifiers.clear()
    obj.data = flat
    flat.calc_loop_triangles()

    if budget is None:
        extent = bounds([v.co * MM_PER_BLENDER_UNIT for v in mesh.vertices])
        if large:
            budget = LARGE_PART_TRIANGLES
        elif max(high - low for low, high in extent) < SMALL_PART_MAX_EXTENT_MM:
            budget = SMALL_PART_TRIANGLES
        else:
            budget = PART_TRIANGLES
    if len(flat.loop_triangles) > budget:
        collapse = obj.modifiers.new('Triangle budget', 'DECIMATE')
        collapse.ratio = budget / len(flat.loop_triangles)
    bpy.context.view_layer.update()
    result = bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph))
    result.calc_loop_triangles()
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.meshes.remove(flat)
    return result


def link_lod(name: str, chunk, large: bool, link_pose: Matrix, cad: CadToBase):
    """Decimate a mesh and express it in its link frame.

    Raises the triangle budget until the bounding box stays within
    LOD_BOUNDS_TOLERANCE_M of the source. Returns the LOD mesh, its vertices in link
    coordinates and in base_link coordinates, and the bounding-box change in metres.
    """
    to_link = link_pose.inverted()
    source_bounds = bounds([cad.to_base(v.co * MM_PER_BLENDER_UNIT) for v in chunk.vertices])
    budget = None
    next_budget = LOD_RETRY_FIRST
    while True:
        lod = decimated(chunk, large, budget)
        local = [to_link @ cad.to_base(v.co * MM_PER_BLENDER_UNIT) for v in lod.vertices]
        in_base = [link_pose @ p for p in local]
        error = bounds_change(bounds(in_base), source_bounds)
        if error <= LOD_BOUNDS_TOLERANCE_M or next_budget > LOD_RETRY_LAST:
            break
        bpy.data.meshes.remove(lod)
        budget = next_budget
        next_budget *= 2
    if error > LOD_BOUNDS_TOLERANCE_M:
        raise RuntimeError(f'LOD bounds of {name!r} moved by {error * MM_PER_M:.3f} mm')
    return lod, local, in_base, error


def material_rgba(material) -> tuple[float, ...]:
    """Base colour of a material, from its Principled BSDF when it has one."""
    if material is None:
        colour = DEFAULT_RGBA
    else:
        colour = material.diffuse_color
        if material.node_tree:
            shader = next(
                (n for n in material.node_tree.nodes if n.type == 'BSDF_PRINCIPLED'), None
            )
            if shader:
                colour = shader.inputs['Base Color'].default_value
    return tuple(round(float(v), COLOUR_DECIMALS) for v in colour)


def add_triangles(lod, points, link: str, buckets) -> None:
    """Sort the LOD triangles into (link, colour) buckets, dropping zero-area ones."""
    colours = [material_rgba(m) for m in lod.materials]
    default = material_rgba(None)
    for tri in lod.loop_triangles:
        rgba = colours[tri.material_index] if tri.material_index < len(colours) else default
        a, b, c = (points[i] for i in tri.vertices)
        if (b - a).cross(c - a).length_squared > MIN_CROSS_PRODUCT_SQ:
            buckets[(link, rgba)].append((a, b, c))


def export_objects(objects, panels, tof_links, cad: CadToBase, frames: Frames):
    """Decimate every visible object and sort its triangles by link and colour.

    Returns the triangle buckets, per-object records, wheel records and all exported
    vertices in base_link.
    """
    buckets = defaultdict(list)
    object_records = []
    wheel_records = []
    all_points = []
    depsgraph = bpy.context.evaluated_depsgraph_get()
    for index, obj in enumerate(objects):
        mesh = bpy.data.meshes.new_from_object(obj.evaluated_get(depsgraph))
        mesh.transform(obj.matrix_world)
        chunks = [(mesh, link_for(obj.name, panels, tof_links), False)]
        if obj.name in WHEEL_OBJECTS:
            chunks, wheel = split_wheel(obj, mesh, cad, frames)
            wheel_records.append(wheel)
        for chunk, link, is_wheel in chunks:
            large = is_wheel or is_large_part(obj.name)
            lod, local, in_base, error = link_lod(obj.name, chunk, large, frames.poses[link], cad)
            all_points.extend(in_base)
            object_records.append(
                {
                    'object': obj.name,
                    'link': link,
                    'triangles': len(lod.loop_triangles),
                    'bounds_error_m': error,
                }
            )
            add_triangles(lod, local, link, buckets)
            bpy.data.meshes.remove(lod)
        if index % PROGRESS_INTERVAL == 0:
            print(f'Exported {index}/{len(objects)} objects', flush=True)

    counts = {w['rotating_vertices'] for w in wheel_records}
    if len(wheel_records) != len(WHEEL_OBJECTS) or len(counts) != 1 or 0 in counts:
        raise RuntimeError(f'Expected two identical non-empty wheels: {wheel_records}')
    return buckets, object_records, wheel_records, all_points


def write_stl(path: Path, triangles) -> None:
    with path.open('wb') as f:
        f.write(STL_HEADER)
        f.write(struct.pack('<I', len(triangles)))
        for a, b, c in triangles:
            normal = (b - a).cross(c - a).normalized()
            f.write(struct.pack('<12fH', *normal, *a, *b, *c, 0))


def write_meshes(buckets, mesh_dir: Path, package: Path):
    """Write one STL per (link, colour). Returns the visuals per link and mesh stats."""
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for old in mesh_dir.glob('*.stl'):
        old.unlink()
    visuals = defaultdict(list)
    stats = {}
    for (link, rgba), triangles in sorted(buckets.items()):
        colour_id = hashlib.sha256(str(rgba).encode()).hexdigest()[:8]
        path = mesh_dir / f'{link}_{colour_id}.stl'
        write_stl(path, triangles)
        stats[path.name] = {
            'triangles': len(triangles),
            'bytes': path.stat().st_size,
            'bounds_m': bounds([p for tri in triangles for p in tri]),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        visuals[link].append({'mesh': path.relative_to(package).as_posix(), 'rgba': rgba})
    return dict(visuals), stats


def convex_hull_xy(points) -> list[tuple[float, float]]:
    """Counter-clockwise convex hull of the points projected onto the XY plane."""
    xy = sorted({(round(p.x, FOOTPRINT_DECIMALS), round(p.y, FOOTPRINT_DECIMALS)) for p in points})

    def turn(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    halves = []
    for sequence in (xy, xy[::-1]):
        half = []
        for p in sequence:
            while len(half) >= 2 and turn(half[-2], half[-1], p) <= 0:
                half.pop()
            half.append(p)
        halves.append(half[:-1])
    return halves[0] + halves[1]


def main() -> None:
    opts = parse_args()
    source = opts.source.resolve()
    package = opts.package.resolve()
    source_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()

    bpy.ops.wm.open_mainfile(filepath=str(source))
    bpy.context.view_layer.update()
    scene = bpy.context.scene
    if abs(scene.unit_settings.scale_length - BLENDER_UNIT_SCALE) > 1e-6:
        raise RuntimeError(f'Unexpected unit scale {scene.unit_settings.scale_length}')

    cad = CadToBase(opts.forward)
    frames = Frames()
    objects = visible_meshes(scene)
    panels = panel_links(objects)
    tof_links = add_sensor_frames(scene, cad, opts.forward, frames)
    buckets, object_records, wheel_records, points = export_objects(
        objects, panels, tof_links, cad, frames
    )
    visuals, mesh_stats = write_meshes(buckets, package / 'meshes/visual', package)

    exported = {obj.name for obj in objects}
    if source.is_relative_to(ROOT):
        source_name = source.relative_to(ROOT).as_posix()
    else:
        source_name = str(source)
    manifest = {
        'source': source_name,
        'source_sha256': source_sha256,
        'forward': opts.forward,
        'mesh_units': 'metres',
        'mesh_scale': [1, 1, 1],
        'cad_to_ros_rotation': [list(row) for row in cad.rotation],
        'base_link_origin_cad_mm': list(cad.origin),
        'physical_wheel_radius_m': PHYSICAL_WHEEL_RADIUS_M,
        'physical_wheel_separation_m': PHYSICAL_WHEEL_SEPARATION_M,
        'physical_wheel_width_m': PHYSICAL_WHEEL_WIDTH_M,
        'effective_odometry_radius_m': ODOMETRY_WHEEL_RADIUS_M,
        'effective_odometry_separation_m': ODOMETRY_WHEEL_SEPARATION_M,
        'frames': frames.records,
        'visuals': visuals,
        'mesh_stats': mesh_stats,
        'source_objects': object_records,
        'wheels': wheel_records,
        'excluded_objects': [
            obj.name
            for obj in scene.objects
            if obj.type == 'MESH' and obj.name not in exported and not obj.name.startswith('_')
        ],
        'assembled_bounds_m': bounds(points),
        'footprint': convex_hull_xy(points),
        'limits': LIMITS,
    }
    (package / 'config/mesh_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

    if hashlib.sha256(source.read_bytes()).hexdigest() != source_sha256:
        raise RuntimeError(f'{source} changed during the export')
    total = sum(s['triangles'] for s in mesh_stats.values())
    print(f'Exported {len(objects)} objects to {len(mesh_stats)} meshes, {total} triangles')


if __name__ == '__main__':
    main()
