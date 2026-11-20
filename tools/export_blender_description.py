"""Export R19 assembled, link-relative ROS meshes; run in Blender, not Python.

Blender --background --disable-autoexec --python tools/export_blender_description.py
No source geometry or .blend is saved. Output meshes are metres (scale 1).
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

import bpy
import bmesh
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
TAGS = ('Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right')


def bounds(points):
    return [[min(p[i] for p in points), max(p[i] for p in points)] for i in range(3)]


def center(o):
    ps = [o.matrix_world @ Vector(p) * 100 for p in o.bound_box]
    return Vector([(a+b)/2 for a,b in bounds(ps)])


def components(mesh):
    bm = bmesh.new(); bm.from_mesh(mesh)
    bm.verts.ensure_lookup_table()
    pending = set(bm.verts); result = []
    while pending:
        stack = [pending.pop()]; indices = set()
        while stack:
            v = stack.pop(); indices.add(v.index)
            for e in v.link_edges:
                other = e.other_vert(v)
                if other in pending:
                    pending.remove(other); stack.append(other)
        result.append(indices)
    bm.free()
    return result


def subset(mesh, indices):
    ids = sorted(indices); lut = {old:new for new,old in enumerate(ids)}
    faces = [p for p in mesh.polygons if all(i in indices for i in p.vertices)]
    out = bpy.data.meshes.new('_ROS wheel component')
    out.from_pydata([mesh.vertices[i].co for i in ids], [], [tuple(lut[i] for i in p.vertices) for p in faces])
    for mat in mesh.materials: out.materials.append(mat)
    for a,b in zip(out.polygons, faces): a.material_index = b.material_index
    out.update()
    return out


def optimized(mesh, large=False, collapse=True, budget=None):
    """Visual LOD only: planar dissolve then a per-object triangle budget."""
    ob = bpy.data.objects.new('_ROS visual LOD', mesh)
    bpy.context.scene.collection.objects.link(ob)
    mod = ob.modifiers.new('Planar visual simplification', 'DECIMATE')
    mod.decimate_type = 'DISSOLVE'; mod.angle_limit = math.radians(0.5)
    mod.delimit = {'MATERIAL'}
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    flat = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    ob.modifiers.clear(); ob.data = flat
    flat.calc_loop_triangles()
    bb=bounds([v.co*100 for v in mesh.vertices])
    limit = budget or (18000 if large else 350 if max(b-a for a,b in bb)<15 else 2000)
    if collapse and len(flat.loop_triangles) > limit:
        mod = ob.modifiers.new('Bounded visual LOD', 'DECIMATE')
        mod.ratio = limit / len(flat.loop_triangles)
    bpy.context.view_layer.update()
    result = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    result.calc_loop_triangles()
    bpy.data.objects.remove(ob, do_unlink=True)
    return result


def stl(path, triangles):
    with path.open('wb') as f:
        f.write(b'Roam-E R19 ROS visual; metres; not for manufacturing'.ljust(80,b' '))
        f.write(struct.pack('<I',len(triangles)))
        for a,b,c in triangles:
            n = (b-a).cross(c-a).normalized()
            f.write(struct.pack('<12fH',*n,*a,*b,*c,0))


def hull(points):
    ps = sorted(set((round(p.x,6),round(p.y,6)) for p in points))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    halves=[]
    for seq in [ps,list(reversed(ps))]:
        half=[]
        for p in seq:
            while len(half)>=2 and cross(half[-2],half[-1],p)<=0:half.pop()
            half.append(p)
        halves.append(half[:-1])
    return halves[0]+halves[1]


def main():
    args = sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,default=ROOT/'mechanical/roam_e.blend')
    parser.add_argument('--package',type=Path,default=ROOT/'src/roam_e_description')
    parser.add_argument('--forward',choices=['caster','camera'],default='caster')
    opts=parser.parse_args(args)
    source=opts.source.resolve(); package=opts.package.resolve()
    source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(source)); bpy.context.view_layer.update()
    assert abs(bpy.context.scene.unit_settings.scale_length-.1)<1e-6
    # Proper rotation, determinant +1. Caster faces CAD +Y, camera CAD -Y.
    rot=Matrix(((0,1,0),(-1,0,0),(0,0,1)))
    if opts.forward=='camera':rot=Matrix(((0,-1,0),(1,0,0),(0,0,1)))
    # Preserve the existing nominal sensor-frame basis when re-expressing the
    # base axes. Identity in both conventions would silently turn laser/board
    # axes relative to the hardware, rather than just changing base coordinates.
    nominal_basis = (Matrix.Rotation(math.pi, 3, 'Z')
                     if opts.forward == 'camera' else Matrix.Identity(3))
    base_cad=Vector((0,-37,32.5))
    def ros(p):return rot @ (Vector(p)-base_cad) / 1000
    physical_radius=.033875; physical_track=.173630
    frames={}; matrices={'base_link':Matrix.Identity(4)}
    def frame(name,parent,xyz,rpy,note):
        frames[name]={'parent':parent,'xyz':[round(v,9) for v in xyz],
            'rpy':[round(v,9) for v in rpy],'status':note}
    def world_frame(name,p,rotation=None,note='CAD nominal; bench verification required'):
        if rotation is None:rotation=nominal_basis
        xyz=ros(p); m=rotation.to_4x4();m.translation=xyz; matrices[name]=m
        frame(name,'base_link',xyz,rotation.to_euler('XYZ'),note)

    # Independent shells can be hidden in RViz without hiding the sensors.
    # Match revised names by part ID, since R18/R16 descriptive labels vary.
    visible=[o for o in bpy.context.scene.objects if o.type=='MESH' and not o.hide_render and not o.hide_get()
        and not any(c.name=='06 Studio' for c in o.users_collection)]
    ids={'P15B':'hood_side_link','P15C':'hood_front_link','P15D':'hood_rear_link',
         'P16B':'skirt_side_link','P16C':'skirt_front_link','P16D':'skirt_rear_link','P05':'battery_drawer_link'}
    part_map={o.name:ids[o.name.split(' |')[0]] for o in visible if o.name.split(' |')[0] in ids}
    assert set(part_map.values())==set(ids.values()),part_map
    for link in ids.values():
        matrices[link]=Matrix.Identity(4);frame(link,'base_link',(0,0,0),(0,0,0),'Rigid assembled panel; origin shared with base_link')
    # Rotor-envelope midpoint is only a nominal scan plane, not metrology.
    rotor=bpy.context.scene.objects['R15 LiDAR | Rotating body 70.26 mm']
    world_frame('laser_frame',center(rotor),note='CAD rotor-envelope midpoint; actual scan plane and yaw must be measured')
    world_frame('imu_mount_link',center(bpy.context.scene.objects['IMU | Adafruit BNO055 Rev C reference']))
    imu_origin=ros(center(bpy.context.scene.objects['IMU | BNO055 package']))
    frame('imu_link','imu_mount_link',imu_origin-matrices['imu_mount_link'].translation,(math.pi,0,0),
          'Retained software roll=pi axis convention; drawing is not proof of physical chip axes')
    # Match the OpenNI camera_link body convention at the depth optic.
    camrot=rot @ Matrix(((0,1,0),(-1,0,0),(0,0,1)))
    world_frame('camera_link',center(bpy.context.scene.objects['Depth_Coated_Lens']),camrot,
        'Provisional depth lens visual centre; optical detail in Blender is unmeasured')
    optical=Matrix(((0,0,1),(-1,0,0),(0,-1,0)))
    for link,objname in [('camera_rgb_optical_frame','RGB_Coated_Lens'),
                         ('camera_depth_optical_frame','Depth_Coated_Lens'),
                         ('camera_ir_optical_frame','Depth_Coated_Lens')]:
        xyz=camrot.transposed() @ (ros(center(bpy.context.scene.objects[objname]))-matrices['camera_link'].translation)
        frame(link,'camera_link',xyz,optical.to_euler('XYZ'),
              'Preview only: nominal CAD centre and REP-103 axes; not stereo calibration')
    for tag in TAGS:
        pcb=bpy.context.scene.objects[f'ToF {tag} | Reference PCB 25x10.66 R09']
        optic=bpy.context.scene.objects[f'ToF {tag} | VL53L0X optical package']
        # Range +X outward, +Z up, +Y left. PCB local +Z is beam direction.
        forward=(rot @ pcb.matrix_world.to_3x3().col[2]).normalized()
        up=Vector((0,0,1)); left=up.cross(forward).normalized(); forward=left.cross(up)
        r=Matrix((forward,left,up)).transposed()
        world_frame('tof_'+tag.lower()+'_link',center(optic),r)

    meshdir=package/'meshes/r19';meshdir.mkdir(parents=True,exist_ok=True)
    buckets=defaultdict(list); members=defaultdict(list); source_records=[]
    full_points=[]; wheel_checks=[]; dg=bpy.context.evaluated_depsgraph_get()
    for index,o in enumerate(visible):
        mesh=bpy.data.meshes.new_from_object(o.evaluated_get(dg))
        mesh.transform(o.matrix_world)
        link='base_link'
        if o.name.startswith('R15 LiDAR |'):link='laser_frame'
        elif o.name.startswith(('IMU | Adafruit','IMU | BNO055','IMU | Header pad')):link='imu_mount_link'
        elif o.name.startswith(('RGB_','IR_Projector_','Depth_','Front_')) or o.name in ['Housing_Silver_Capsule','Rear_Black_Inset','Right_Small_Front_Aperture','Left_Small_Front_Aperture']:link='camera_link'
        elif o.name.startswith('ToF '):
            tag=o.name.split(' |')[0][4:];link='tof_'+tag.lower()+'_link'
        elif o.name in part_map:link=part_map[o.name]
        chunks=[(mesh,link,False)]
        if o.name in ['Component | Motor_Wheel','Component | Motor_Wheel_Left']:
            groups=components(mesh)
            moving=set()
            for g in groups:
                bb=bounds([mesh.vertices[i].co*100 for i in g])
                if bb[1][1]-bb[1][0]>50 and bb[2][1]-bb[2][0]>50:moving.update(g)
            assert len(moving)==35630,(o.name,len(moving))
            wheelmesh=subset(mesh,moving);motor=subset(mesh,set(range(len(mesh.vertices)))-moving)
            wheelcentre=center(o); wheelcentre=Vector(((-88.3 if 'Left' in o.name else 88.3),-37,32.5))
            side='left' if ros(wheelcentre).y>0 else 'right';wl=side+'_wheel_link'
            jointorigin=Vector((0,physical_track/2*(1 if side=='left' else -1),0))
            matrices[wl]=Matrix.Translation(jointorigin)
            frame(wl,'base_link',jointorigin,(0,0,0),'Measured physical track; CAD tyre/hub scaled radially to measured radius')
            for v in wheelmesh.vertices:
                p=ros(v.co*100)-ros(wheelcentre)
                p.x*=physical_radius/.0325;p.z*=physical_radius/.0325
                # Inverse map, so the normal common export path produces local wheel mesh.
                v.co=(base_cad+rot.transposed() @ ((p+jointorigin)*1000))/100
            chunks=[(motor,'base_link',False),(wheelmesh,wl,True)]
            wheel_checks.append({'object':o.name,'wheel_link':wl,'rotating_vertices':len(moving),
                'cad_center_mm':list(wheelcentre),'cad_radius_m':.0325,'physical_radius_m':physical_radius,
                'cad_track_m':.1766,'physical_track_m':physical_track})
        for chunk,link,is_wheel in chunks:
            large=is_wheel or o.name.startswith(('P15','P16')) or o.name in ['Component | Caster'] or 'Rotating body' in o.name
            lod=optimized(chunk,large)
            local=matrices[link].inverted()
            points=[local @ ros(v.co*100) for v in lod.vertices]
            assembled=[matrices[link] @ p for p in points]
            cb=bounds(assembled)
            original=[ros(v.co*100) for v in chunk.vertices];ob=bounds(original)
            error=max(abs(cb[i][j]-ob[i][j]) for i in range(3) for j in range(2))
            # Silhouette check; tighter mesh quality is not a manufacturing guarantee.
            budget=4000
            while error>.0005 and budget<=64000:
                bpy.data.meshes.remove(lod)
                lod=optimized(chunk,large,budget=budget)
                points=[local @ ros(v.co*100) for v in lod.vertices]
                assembled=[matrices[link] @ p for p in points]
                cb=bounds(assembled)
                error=max(abs(cb[i][j]-ob[i][j]) for i in range(3) for j in range(2))
                budget*=2
            if error>.0005:raise RuntimeError(f'LOD bounds changed >0.5mm: {o.name} {error}')
            full_points.extend(assembled)
            source_records.append({'object':o.name,'link':link,'triangles':len(lod.loop_triangles),'bounds_error_m':error})
            members[link].append(o.name)
            for tri in lod.loop_triangles:
                mat=lod.materials[tri.material_index] if tri.material_index<len(lod.materials) else None
                color=mat.diffuse_color if mat else (.12,.14,.16,1.)
                if mat and mat.use_nodes and mat.node_tree:
                    shader=next((n for n in mat.node_tree.nodes if n.type=='BSDF_PRINCIPLED'),None)
                    if shader:color=shader.inputs['Base Color'].default_value
                rgba=tuple(round(float(v),3) for v in color)
                ps=tuple(points[i] for i in tri.vertices)
                if (ps[1]-ps[0]).cross(ps[2]-ps[0]).length_squared>1e-24:buckets[(link,rgba)].append(ps)
            bpy.data.meshes.remove(lod)
        if index%100==0:print('ROS_EXPORT',index,len(visible),flush=True)
    visuals=defaultdict(list);meshstats={}
    for (link,rgba),tris in sorted(buckets.items()):
        colorid=hashlib.sha256(str(rgba).encode()).hexdigest()[:8]
        name=f'{link}_{colorid}.stl';path=meshdir/name;stl(path,tris)
        meshstats[name]={'triangles':len(tris),'bytes':path.stat().st_size,
            'bounds_m':bounds([p for t in tris for p in t]),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        visuals[link].append({'mesh':f'meshes/r19/{name}','rgba':rgba})
    metadata={'revision':'R19','source':str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) else str(source),
        'source_sha256':source_hash,'forward':opts.forward,'mesh_units':'metres','mesh_scale':[1,1,1],
        'cad_to_ros_rotation':[list(row) for row in rot],'base_link_origin_cad_mm':list(base_cad),
        'physical_wheel_radius_m':physical_radius,'physical_wheel_separation_m':physical_track,
        'effective_odometry_radius_m':.033975,'effective_odometry_separation_m':.175150,
        'frames':frames,'visuals':dict(visuals),'mesh_stats':meshstats,'source_objects':source_records,
        'wheel_separation_checks':wheel_checks,'excluded_objects':[o.name for o in bpy.context.scene.objects if o.type=='MESH' and o not in visible and not o.name.startswith('_ROS')],
        'assembled_bounds_m':bounds(full_points),'navigation_footprint_candidate':hull(full_points),
        'limits':['Nominal CAD sensor frames, not measured calibration.',
                  'No masses/inertia or simulated sensors; visualization/TF model, not a dynamics model.',
                  'Saved RGB/IMU intrinsics, offsets, driver conventions and effective odometry untouched.',
                  'ToF station names follow CAD labels, not ROS forward convention; no I2C channel assignment inferred.']}
    (package/'config/mesh_export_r19.json').write_text(json.dumps(metadata,indent=2)+'\n')
    assert hashlib.sha256(source.read_bytes()).hexdigest()==source_hash
    print('ROS_EXPORT_DONE',len(visible),len(meshstats),sum(s['triangles'] for s in meshstats.values()),flush=True)


if __name__=='__main__':main()
