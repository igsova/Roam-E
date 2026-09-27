"""Independent R07 verification, print-layout scene, and release packaging.

Run with Blender 5.2 in background after create_split_shell_r07.py.
Does not execute the generator or write into any prior revision.
"""
import csv
import hashlib
import json
import math
import shutil
import zipfile
from pathlib import Path

import bpy
import bmesh
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'mechanical/split_shell_r07'
OLD=ROOT/'mechanical/geekworm_mux_r06'
PACK=OUT/'ROAM_STL_R07_Split_Shell_K1C'
CHANGED=OUT/'ROAM_R07_changed_parts'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((PACK/'manifest.json').read_text())
report=json.loads((OUT/'geometry_checks.json').read_text())
assert not report['assembly_clashes'] and not report['mating_clashes'] and not report['unexpected_original_changes']
assert len(manifest['parts'])==45 and sum(p['quantity'] for p in manifest['parts'])==120
assert len(report['joints'])==5
assert min(j['main_screw_shoulder_at_axis_mm'] for j in report['joints'])>2.5
assert HASH(OLD/'roam_geekworm_mux_r06.blend')==report['source_sha256']

def snapshot(o):
    h=hashlib.sha256(str(tuple(tuple(r) for r in o.matrix_world)).encode())
    if o.type=='MESH':
        for v in o.data.vertices:h.update(str(tuple(v.co)).encode())
        for p in o.data.polygons:h.update(str(tuple(p.vertices)).encode())
    return h.hexdigest()

def world_mesh(o):
    mesh=bpy.data.meshes.new_from_object(o.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    mesh.transform(o.matrix_world);return mesh

def tree(o):
    me=world_mesh(o);bm=bmesh.new();bm.from_mesh(me)
    result=BVHTree.FromBMesh(bm);bm.free();bpy.data.meshes.remove(me);return result

def metrics(o):
    bm=bmesh.new();bm.from_mesh(o.data);bm.transform(o.matrix_world)
    todo=set(bm.verts);components=0
    while todo:
        components+=1;stack=[todo.pop()]
        while stack:
            for e in stack.pop().link_edges:
                for v in e.verts:
                    if v in todo:todo.remove(v);stack.append(v)
    pts=[v.co.copy() for v in bm.verts]
    result={'components':components,'nonmanifold_edges':sum(not e.is_manifold for e in bm.edges),
        'volume':bm.calc_volume(signed=True),'dimensions':[max(p[i] for p in pts)-min(p[i] for p in pts) for i in range(3)],
        'z_min':min(p.z for p in pts)}
    bm.free();return result

old_shells=['Shell | Removable sensor hood','Shell | Independent lower skirt']
bpy.ops.wm.open_mainfile(filepath=str(OLD/'roam_geekworm_mux_r06.blend'))
bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in bpy.context.scene.objects if o.name not in old_shells and o.type!='CAMERA'}
points={}
for name in old_shells:
    obj=bpy.data.objects[name];me=world_mesh(obj)
    points[name]=[list(v.co*100) for v in me.vertices];bpy.data.meshes.remove(me)
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_split_shell_r07.blend'))
bpy.context.view_layer.update();S=bpy.context.scene
changed=[n for n,h in original.items() if n not in S.objects or snapshot(S.objects[n])!=h]
assert not changed,changed
assert not any('MCU' in o.name for o in S.objects)
assert 'Component | Arduino_Nano' in S.objects
assert 'C09 | TCA9548A removable carrier' in S.objects
assert 'C08 | Geekworm base edge retainer front' in S.objects
names=['P15A | Main smiling hood R07','P15B | Removable hood side panel R07',
       'P16A | Main ToF skirt R07','P16B | Removable ToF side panel R07']
surfaces={n:tree(S.objects[n]) for n in names}
surface_checks={}
for source,targets in zip(old_shells,[names[:2],names[2:]]):
    measured=[]
    for coords in points[source]:
        x,y,z=coords
        if abs(x-95)<.5:continue # intentional 0.30 mm seam gap
        skip=False
        for j in report['joints']:
            if j['main'] not in targets:continue
            if all(a-1<=p<=b+1 for p,(a,b) in zip(coords,j['qa_bounds_mm'])):skip=True
        if source==old_shells[0] and 89<x<101 and 130<z<140 and 78<abs(y)<88:skip=True
        if skip:continue
        p=Vector(coords)/100
        distances=[surfaces[n].find_nearest(p)[3]*100 for n in targets]
        measured.append(min(distances))
    surface_checks[source]={'sampled_original_vertices_outside_intentional_changes':len(measured),
        'max_surface_error_mm':max(measured),'limit_mm':.01}
    assert max(measured)<.01,surface_checks[source]

# Positive-X panel separation at sampled offsets; removes no assembly hardware.
# This tests the cover pair only, not a powered robot or a complete removal path.
separation=[]
for main_name,cap_name in [names[:2],names[2:]]:
    main=S.objects[main_name];cap=S.objects[cap_name]
    for offset in [0,1,3,7,12,20,40]:
        me=world_mesh(cap);me.transform(Matrix.Translation(Vector((offset/100,0,0))))
        obj=bpy.data.objects.new('_separation check',me);S.collection.objects.link(obj)
        mod=obj.modifiers.new('Cover separation intersection','BOOLEAN');mod.operation='INTERSECT';mod.solver='MANIFOLD';mod.object=main
        bpy.context.view_layer.objects.active=obj;bpy.ops.object.modifier_apply(modifier=mod.name)
        bm=bmesh.new();bm.from_mesh(obj.data);vol=abs(bm.calc_volume(signed=True))*1e6;bm.free()
        bpy.data.objects.remove(obj,do_unlink=True)
        separation.append({'cap':cap_name,'offset_x_mm':offset,'intersection_mm3':vol})
        assert vol<.03,separation[-1]

old_manifest=json.loads((OLD/'ROAM_STL_R06_Geekworm_MUX_K1C/manifest.json').read_text())
old_parts={r['id']:r for r in old_manifest['parts']}
unchanged_stls=[];new_checks={}
for row in manifest['parts']:
    path=PACK/row['file'];assert HASH(path)==row['sha256']
    if row['id'] in old_parts:
        assert HASH(path)==old_parts[row['id']]['sha256'];unchanged_stls.append(row['id'])
    else:
        bpy.ops.wm.stl_import(filepath=str(path),use_facet_normal=False)
        obj=bpy.context.object;checked=metrics(obj)
        assert checked['components']==1 and checked['nonmanifold_edges']==0 and checked['volume']>0,checked
        assert abs(checked['z_min'])<.001
        assert all(abs(a-b)<.002 for a,b in zip(checked['dimensions'],row['reimport_check']['dimensions_mm']))
        assert all(d+8<220 for d in checked['dimensions'][:2]) and checked['dimensions'][2]<250
        new_checks[row['id']]=checked;bpy.data.objects.remove(obj,do_unlink=True)
assert len(unchanged_stls)==39 and len(new_checks)==6
result={'revision':'R07','status':'digital geometry checks passed; physical and slicer tests outstanding',
    'source_sha256':report['source_sha256'],'unchanged_R06_objects_excluding_camera_and_replaced_shells':len(original),
    'unexpected_changes':changed,'protected_surface_checks':surface_checks,
    'unchanged_STL_ids':unchanged_stls,'new_STL_reimport_checks':new_checks,'sampled_panel_separation':separation,
    'slicer_check':{'status':'not completed','cause':'Installed Creality Print CLI crashes in PartPlate::set_shape before slicing the unmodified R06 baseline.',
        'support_or_time_savings_measured':False},
    'limits':['Surface-vertex comparison is not a physical measurement or volumetric proof.',
        'Sampled separation checks cover pair geometry only; remove fasteners and unplug harnesses first.',
        'Nominal 4 mm brim bounding-box fit is not a toolpath/bed-exclusion validation.']}
(OUT/'validation.json').write_text(json.dumps(result,indent=2))
print('R07_VALIDATION',json.dumps(result),flush=True)

# Print preparation scene contains only the four current cover meshes, plates,
# camera, labels and lighting. Parts retain their exact supplied STL orientation.
for obj in list(S.objects):
    if obj.type not in ('CAMERA','LIGHT'):bpy.data.objects.remove(obj,do_unlink=True)
bpy.data.orphans_purge(do_recursive=True)
S.unit_settings.system='METRIC';S.unit_settings.scale_length=.1
def material(name,color):
    m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
    m.node_tree.nodes.get('Principled BSDF').inputs['Base Color'].default_value=(*color,1)
    m.node_tree.nodes.get('Principled BSDF').inputs['Roughness'].default_value=.5
    return m
white=material('R07 print | main hood',(.70,.76,.78))
teal=material('R07 print | removable panel',(.025,.30,.32))
dark=material('R07 print | main skirt',(.12,.17,.20))
bedmat=material('R07 print | nominal bed',(.065,.095,.115))
textmat=material('R07 print | labels',(.7,.87,.87))
floor=material('R07 print | floor',(.025,.039,.052))
def cube(name,position,size,mat):
    bpy.ops.mesh.primitive_cube_add(size=1,location=Vector(position)/100)
    obj=bpy.context.object;obj.name=name;obj.dimensions=Vector(size)/100
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    obj.data.materials.append(mat);return obj
def label(text,position,size=7):
    data=bpy.data.curves.new(text,'FONT');data.body=text;data.align_x='CENTER';data.size=size/100;data.extrude=.015/100
    obj=bpy.data.objects.new(text,data);S.collection.objects.link(obj);obj.location=Vector(position)/100;data.materials.append(textmat)
centres=[(-132,143),(132,143),(-132,-137),(132,-137)]
for pid,centre,mat in zip(['P15A','P15B','P16A','P16B'],centres,[white,teal,dark,teal]):
    row=next(p for p in manifest['parts'] if p['id']==pid)
    bpy.ops.wm.stl_import(filepath=str(PACK/row['file']),use_facet_normal=False)
    obj=bpy.context.object;obj.name=pid+' | Print oriented';obj.data.transform(Matrix.Scale(.01,4));obj.location=Vector((*centre,0))/100
    obj.data.materials.append(mat);obj['STL source']=row['file'];obj['Orientation']=row['note']
    cube('Display bed 220 x 220 | '+pid,(*centre,-1.5),(220,220,3),bedmat)
    d=row['reimport_check']['dimensions_mm']
    label(f'{pid}  {d[0]:.1f} x {d[1]:.1f} x {d[2]:.1f} mm',(centre[0],centre[1]-123,0))
label('R07  /  FOUR SEPARATE K1C PLATES  /  DO NOT AUTO-ORIENT',(0,-285,0),8)
cube('Display floor',(0,0,-4),(2000,2000,2),floor)
S['Mechanical revision']='R07 print preparation - four independent plate layouts, display plates NOT printable'
S['Print preparation']='STL millimetres converted into 100 mm/BU display scene. Export the supplied STLs, NOT the decorative beds/text.'
S.pop('Exploded view',None)
cam=S.camera;cam.location=Vector((330,-630,980))/100
target=Vector((0,0,10))/100;cam.rotation_euler=(target-cam.location).to_track_quat('-Z','Y').to_euler()
cam.data.type='ORTHO';cam.data.ortho_scale=760/100
S.render.engine='CYCLES';S.cycles.samples=32;S.cycles.device='CPU'
S.render.resolution_x=1800;S.render.resolution_y=1500;S.render.resolution_percentage=100
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type=='VIEW_3D':
            v=area.spaces.active;v.region_3d.view_rotation=cam.rotation_euler.to_quaternion();v.region_3d.view_location=target;v.region_3d.view_distance=7.6
bpy.context.preferences.filepaths.save_version=0
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_split_shell_r07_print_preparation.blend'),compress=True)
S.render.filepath=str(OUT/'print_orientations.png');bpy.ops.render.render(write_still=True)

# Package only after independent validation succeeds.
with (PACK/'R07_JOINTS.csv').open('w',newline='') as f:
    writer=csv.writer(f);writer.writerow(['Joint','Screw','Quantity','Nut','Washer','Axis seat X mm','Y mm','Z mm','Main shoulder mm'])
    for j in report['joints']:writer.writerow([j['id'],j['screw'],1,'M3 5.5 AF x 2.4 mm','M3 6 OD x 0.5 mm',*[round(x,3) for x in j['screw_seat_mm']],round(j['main_screw_shoulder_at_axis_mm'],3)])
shutil.copytree(OLD/'ROAM_STL_R06_Geekworm_MUX_K1C/historical_R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
shutil.copy2(OLD/'ROAM_STL_R06_Geekworm_MUX_K1C/Geekworm_mount_R05_reference.md',PACK/'Geekworm_mount_R05_reference.md')
shutil.copy2(OLD/'ASSEMBLY_R06.md',PACK/'Electronics_R06_reference.md')
for target in (PACK,CHANGED):
    shutil.copy2(OUT/'ASSEMBLY_R07.md',target/'START_HERE.md')
    for name in ('geometry_checks.json','validation.json','print_orientations.png','split_covers_exploded.png'):
        shutil.copy2(OUT/name,target/name)
    if target!=PACK:
        for name in ('HARDWARE.csv','R07_JOINTS.csv'):shutil.copy2(PACK/name,target/name)
    with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as archive:
        for p in sorted(target.rglob('*')):
            if p.is_file():archive.write(p,p.relative_to(OUT))
    with zipfile.ZipFile(OUT/(target.name+'.zip')) as archive:
        assert archive.testzip() is None
        for row in json.loads((target/'manifest.json').read_text())['parts']:
            assert hashlib.sha256(archive.read(target.name+'/'+row['file'])).hexdigest()==row['sha256']
print('R07_RELEASE_COMPLETE',str(OUT),flush=True)
