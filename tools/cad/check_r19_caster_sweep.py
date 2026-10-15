"""Read-only caster envelope audit; run with Blender --disable-autoexec.

The STL has no kinematic joint. Infer the nominal swivel axis from the
mounting plate, cross-check it against the four spacer centres, and rotate a
conservative XY bounding-box envelope through a full circle. Rotating the
fixed plate too deliberately overestimates the movable caster envelope.
No source mesh/scene is edited or saved. Output goes to stdout only.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import bmesh
import bpy
from mathutils import Vector


def bounds(points):
    return [[min(p[i] for p in points), max(p[i] for p in points)] for i in range(3)]


def inspect(source):
    source = source.resolve()
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(source))
    bpy.context.view_layer.update()
    if abs(bpy.context.scene.unit_settings.scale_length - 0.1) > 1e-6:
        raise ValueError('Unexpected CAD scene units')
    ob = bpy.data.objects['Component | Caster']
    mesh = bpy.data.meshes.new_from_object(
        ob.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    bm = bmesh.new()
    bm.from_mesh(mesh)
    pending = set(bm.verts)
    groups = []
    while pending:
        stack = [pending.pop()]
        points = []
        while stack:
            vertex = stack.pop()
            points.append(ob.matrix_world @ vertex.co * 100)
            for edge in vertex.link_edges:
                other = edge.other_vert(vertex)
                if other in pending:
                    pending.remove(other)
                    stack.append(other)
        groups.append({'vertices': len(points), 'bounds_cad_mm': bounds(points)})
    bm.free()
    all_bounds = bounds([ob.matrix_world @ v.co * 100 for v in mesh.vertices])
    # In this imported STL the component reaching the top plane is the plate.
    plate = max(groups, key=lambda group: group['bounds_cad_mm'][2][1])
    pb = plate['bounds_cad_mm']
    if pb[2][1] - pb[2][0] > 5 or pb[0][1] - pb[0][0] < 30:
        raise ValueError('Caster mounting-plate identification changed; review CAD')
    pivot = [(pb[i][0] + pb[i][1]) / 2 for i in (0, 1)]
    spacer_centres = []
    for index in range(1, 5):
        spacer = bpy.data.objects[f'P02.{index} | Caster spacer 13.501 mm']
        bb = bounds([spacer.matrix_world @ Vector(p) * 100 for p in spacer.bound_box])
        spacer_centres.append([(bb[i][0] + bb[i][1]) / 2 for i in (0, 1)])
    spacer_pivot = [sum(p[i] for p in spacer_centres) / 4 for i in (0, 1)]
    disagreement = math.dist(pivot, spacer_pivot)
    if disagreement > 0.01:
        raise ValueError('Plate and spacer inferred axes disagree by >0.01 mm')
    # The disc encloses every bounding-box corner at every rotation; no
    # angular sampling or inscribed-circle approximation can miss an angle.
    radius_mm = max(math.hypot(x - pivot[0], y - pivot[1])
                    for x in all_bounds[0] for y in all_bounds[1])
    radius_m = math.ceil(radius_mm - 1e-9) / 1000  # outward to whole millimetres
    bpy.data.meshes.remove(mesh)
    if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
        raise ValueError('Source Blender file changed')
    return {
        'status': 'CAD_ONLY', 'source_sha256': digest,
        'scene_scale_length': 0.1, 'object': ob.name,
        'component_bounds_cad_mm': all_bounds,
        'mesh_components': sorted(groups, key=lambda g: -g['vertices']),
        'nominal_pivot_cad_xy_mm': pivot,
        'spacer_centres_cad_xy_mm': spacer_centres,
        'plate_spacer_axis_disagreement_mm': disagreement,
        'sweep_radius_from_bbox_m': radius_mm / 1000,
        'sweep_radius_outward_rounded_m': radius_m,
        'method': 'Full XY bounding-box rotation, including fixed plate; analytic disc',
        'source_unchanged': True, 'physical_swivel_axis_verified': False,
        'limits': ['Swivel axis inferred from mounting-plate symmetry, not a CAD joint.',
                   'Physical caster/assembly and external harness protrusions unverified.'],
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    print('R19_CASTER_REPORT ' + json.dumps(inspect(args.source), sort_keys=True))
