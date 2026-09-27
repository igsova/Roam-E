"""Merge only the multiplexer collection into the Geekworm R05 robot.

Blender 5.2: --background --disable-autoexec --python THIS.py -- [--render] [--donor PATH]
Read-only source files; outputs go to mechanical/geekworm_mux_r06.
Assembly scale is 100 mm/BU; exported STL coordinates are millimetres.
"""
import argparse
import csv
import hashlib
import json
import math
import shutil
import sys
import zipfile
from pathlib import Path

import bpy
import bmesh
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

ROOT=Path(__file__).resolve().parents[2]
args=argparse.ArgumentParser()
args.add_argument('--render',action='store_true')
args.add_argument('--donor',type=Path,
    default=ROOT/'mechanical/source_archive/roam_tof_mux_donor.blend',
    help='Preserved R04 multiplexer donor; override explicitly for another source.')
args=args.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
OLD=ROOT/'mechanical/geekworm_r05'
OUT=ROOT/'mechanical/geekworm_mux_r06'
DONOR=args.donor.resolve()
SOURCE=OLD/'roam_geekworm_r05.blend'
PACK=OUT/'ROAM_STL_R06_Geekworm_MUX_K1C'
CHANGED=OUT/'ROAM_R06_changed_parts'
OUT.mkdir(parents=True,exist_ok=True)
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source_hash,donor_hash=HASH(SOURCE),HASH(DONOR)
bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene
assert abs(S.unit_settings.scale_length-.1)<1e-6
assert 'R05 Geekworm' in S['Mechanical revision']
assert not S.get('Exploded view')

def clean(o):
    bm=bmesh.new();bm.from_mesh(o.data)
    bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-7)
    bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=1e-8)
    bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
    bm.to_mesh(o.data);bm.free();o.data.update()

def bounds(o):
    return [(min((o.matrix_world@Vector(p))[i]*100 for p in o.bound_box),
             max((o.matrix_world@Vector(p))[i]*100 for p in o.bound_box)) for i in range(3)]

def snapshot(o):
    h=hashlib.sha256()
    h.update(str(tuple(tuple(row) for row in o.matrix_world)).encode())
    if o.type=='MESH':
        for v in o.data.vertices:h.update(str(tuple(v.co)).encode())
        for p in o.data.polygons:h.update(str(tuple(p.vertices)).encode())
    return h.hexdigest()

def box(name,location,dimensions):
    bpy.ops.mesh.primitive_cube_add(size=1,location=Vector(location)/100)
    o=bpy.context.object;o.name=name;o.dimensions=Vector(dimensions)/100
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    return o

def cylinder(name,x,y,z,radius,height):
    bpy.ops.mesh.primitive_cylinder_add(vertices=64,radius=radius/100,depth=height/100,location=(x/100,y/100,z/100))
    o=bpy.context.object;o.name=name;return o

def boolean(a,b,operation='DIFFERENCE'):
    bpy.context.view_layer.update()
    mod=a.modifiers.new('R06 '+operation,'BOOLEAN');mod.operation=operation;mod.solver='MANIFOLD';mod.object=b
    bpy.context.view_layer.objects.active=a;bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(b,do_unlink=True);clean(a)

def clone(o,name):
    me=bpy.data.meshes.new_from_object(o.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    obj=bpy.data.objects.new(name,me);S.collection.objects.link(obj);obj.matrix_world=o.matrix_world.copy();return obj

def metrics(o):
    bm=bmesh.new();bm.from_mesh(o.data);bm.transform(o.matrix_world)
    pending=set(bm.verts);components=0
    while pending:
        components+=1;stack=[pending.pop()]
        while stack:
            v=stack.pop()
            for e in v.link_edges:
                n=e.other_vert(v)
                if n in pending:pending.remove(n);stack.append(n)
    points=[v.co.copy() for v in bm.verts]
    result={'components':components,'nonmanifold_edges':sum(not e.is_manifold for e in bm.edges),
            'loose_vertices':sum(not v.link_faces for v in bm.verts),'volume':bm.calc_volume(signed=True),
            'dimensions':[max(p[i] for p in points)-min(p[i] for p in points) for i in range(3)],
            'min_z':min(p.z for p in points)}
    bm.free();return result

def intersection(a,b):
    if not all(min(x[1],y[1])-max(x[0],y[0])>.015 for x,y in zip(bounds(a),bounds(b))):return 0.0
    tmp=clone(a,'_QA')
    mod=tmp.modifiers.new('Clash','BOOLEAN');mod.operation='INTERSECT';mod.solver='MANIFOLD';mod.object=b
    bpy.context.view_layer.objects.active=tmp;bpy.ops.object.modifier_apply(modifier=mod.name)
    bm=bmesh.new();bm.from_mesh(tmp.data);bm.transform(tmp.matrix_world)
    volume=abs(bm.calc_volume(signed=True))*1e6
    bm.free();bpy.data.objects.remove(tmp,do_unlink=True)
    return volume

def assert_same_part(a,b):
    # Normalize translation; opposite clips require 180-degree in-plane rotation.
    copies=[clone(o,'_duplicate_test') for o in (a,b)]
    for i,o in enumerate(copies):
        o.data.transform(o.matrix_world);o.matrix_world=Matrix.Identity(4)
        if i:o.data.transform(Matrix.Rotation(math.pi,4,'Z'))
        pts=[v.co for v in o.data.vertices]
        centre=Vector(tuple((min(p[k] for p in pts)+max(p[k] for p in pts))/2 for k in range(3)))
        o.data.transform(Matrix.Translation(-centre))
    max_error=0
    for a,b in (copies,copies[::-1]):
        bm=bmesh.new();bm.from_mesh(b.data);tree=BVHTree.FromBMesh(bm)
        for v in a.data.vertices:max_error=max(max_error,tree.find_nearest(v.co)[3]*100)
        bm.free()
    for o in copies:bpy.data.objects.remove(o,do_unlink=True)
    assert max_error<.005,max_error
    return max_error

bpy.context.view_layer.update()
ORIGINAL={o.name:snapshot(o) for o in S.objects}
removed=[o.name for o in S.objects if o.name in ('Component | MCU','Label | MCU') or o.name.startswith(('MCU |','N_MCU_'))]
assert len(removed)==15,removed
for name in removed:bpy.data.objects.remove(bpy.data.objects[name],do_unlink=True)
assert 'Component | Arduino_Nano' in bpy.data.objects

# Append the named collection, never replace the robot/deck with the older file.
with bpy.data.libraries.load(str(DONOR),link=False) as (available,requested):
    assert '11 Multiplexer assembly' in available.collections
    requested.collections=['11 Multiplexer assembly']
MUX=requested.collections[0];S.collection.children.link(MUX)
MUX.name='14 R06 TCA9548A multiplexer'
mapping={}
for o in MUX.objects:
    before=o.name
    if before.startswith('C08 |'):o.name=before.replace('C08 |','C09 |',1)
    if before.startswith('F05 |'):o.name=before.replace('F05 |','F06 |',1)
    mapping[before]=o.name
    if o.name.startswith(('C09 |','P20 |','F06 |')):o['Manufacture']='PRINT; R06 ID '+o.name.split(' |')[0]
    o.hide_set(False);o.hide_render=False
assert len(MUX.objects)==89,len(MUX.objects)
carrier=bpy.data.objects['C09 | TCA9548A removable carrier']
clips=[o for o in MUX.objects if o.name.startswith('P20 |')]
pads=[o for o in MUX.objects if o.name.startswith('F06 |')]
pcb=bpy.data.objects['MUX reference | TCA9548A HW-617 PCB 21.4 x 31.5']
bpy.context.view_layer.update()
DONOR_GEOMETRY={o.name:snapshot(o) for o in MUX.objects}

deck=bpy.data.objects['P04 | Removable electronics deck']
old_holes=[(2,24.25),(18,24.25)]
new_holes=[(-6,-2),(24,-2)]
# Fill the now-unused MCU holes flush, without changing the deck outline.
for x,y in old_holes:boolean(deck,cylinder('_obsolete MCU hole fill',x,y,111,2.2,3),'UNION')
for x,y in new_holes:boolean(deck,cylinder('_MUX 2.9 mm deck hole',x,y,111,1.45,9))
deck['R06 holes']='Filled former MCU holes (2,24.25),(18,24.25); added mux 2.9 mm holes (-6,-2),(24,-2), in mm.'
bpy.context.view_layer.update()

# Validate the actual through-holes and filled old axes with vertical rays.
ray_checks=[]
for expected,axes in ((False,new_holes),(True,old_holes)):
    for x,y in axes:
        hit=deck.ray_cast(deck.matrix_world.inverted()@Vector((x/100,y/100,1.30)),Vector((0,0,-1)),distance=.4)[0]
        ray_checks.append({'xy_mm':[x,y],'surface_hit':hit,'expected_surface_hit':expected})
        assert hit==expected,ray_checks[-1]

GUIDES=bpy.data.collections.new('15 R06 MUX connector guides - hidden');S.collection.children.link(GUIDES)
guides=[]
for x in (-.2,18.2):
    o=box('CHECK | MUX jumper and wire bend',(x,-2,138.5),(4,32,20))
    for c in list(o.users_collection):c.objects.unlink(o)
    GUIDES.objects.link(o);guides.append(o)

# New group against existing visible geometry, including the new Geekworm.
environment=[o for o in S.objects if o.name in ORIGINAL and o.name not in removed and o!=deck and o.type=='MESH'
             and not o.hide_render and not any(c.name.startswith('06 ') for c in o.users_collection)]
clashes=[];guide_clashes=[]
for o in MUX.objects:
    if o.type!='MESH':continue
    for other in environment:
        v=intersection(o,other)
        if v>.03:clashes.append([o.name,other.name,round(v,5)])
for o in guides:
    for other in environment:
        v=intersection(o,other)
        if v>.03:guide_clashes.append([o.name,other.name,round(v,5)])
retention_clashes=[]
for o in [carrier,*clips,*pads]:
    for other in [q for q in MUX.objects if q.name.startswith('MUX reference |')]:
        # Schematic solder pins intentionally pass through the reference PCB,
        # but no reference electronics may penetrate the printed mount.
        v=intersection(o,other)
        if v>.03:retention_clashes.append([o.name,other.name,round(v,5)])
drawer_clashes=[]
for distance in (0,5,30,70,145):
    for other in bpy.data.collections['02 Battery drawer'].objects:
        if other.type!='MESH':continue
        # Probe a copy: assigning/restoring matrix_world can decompose and
        # round nonuniform original transforms even when geometry is unchanged.
        probe=clone(other,'_drawer_sweep');probe.location.y+=distance/100;bpy.context.view_layer.update()
        for o in MUX.objects:
            if o.type!='MESH':continue
            v=intersection(o,probe)
            if v>.03:drawer_clashes.append([distance,o.name,other.name,round(v,5)])
        bpy.data.objects.remove(probe,do_unlink=True)
    bpy.context.view_layer.update()
unexpected=[n for n,h in ORIGINAL.items() if n not in removed and n!=deck.name and snapshot(bpy.data.objects[n])!=h]
donor_changes=[n for n,h in DONOR_GEOMETRY.items() if snapshot(bpy.data.objects[n])!=h]
repeat={'P20_surface_difference_mm':assert_same_part(*clips),'F06_surface_difference_mm':assert_same_part(*pads)}
report={'revision':'R06 Geekworm + multiplexer, unused MCU removed','source':str(SOURCE),'source_sha256':source_hash,
        'donor':str(DONOR),'donor_sha256':donor_hash,'imported_collection':MUX.name,'imported_objects':len(MUX.objects),
        'part_name_mapping':mapping,'removed_objects':removed,'unchanged_original_objects':len(ORIGINAL)-len(removed)-1,
        'unexpected_changes':unexpected,'donor_geometry_changes':donor_changes,
        'assembly_clashes':clashes,'jumper_keepout_clashes':guide_clashes,'board_mount_clashes':retention_clashes,
        'sampled_drawer_sweep_clashes':drawer_clashes,'duplicate_parts':repeat,'deck_ray_checks':ray_checks,
        'board_outline_mm':[21.4,31.5],'new_holes_xy_mm':new_holes,'filled_holes_xy_mm':old_holes,
        'limitations':['Donor board thickness, headers, pads and screw fit remain provisional.',
                      'Static and sampled geometry checks are not physical tests, thermal tests or continuous swept-volume validation.',
                      'Geekworm R05 estimate limitations and fit checks still apply. No electronics wiring was changed.']}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
print('MERGE_CHECKS',json.dumps({k:report[k] for k in ('assembly_clashes','jumper_keepout_clashes','board_mount_clashes','sampled_drawer_sweep_clashes','unexpected_changes','donor_geometry_changes','duplicate_parts')}),flush=True)
assert not any((clashes,guide_clashes,retention_clashes,drawer_clashes,unexpected,donor_changes)), 'Review geometry_checks.json'
assert not any('MCU' in o.name for o in S.objects)
assert 'C08 | Geekworm base edge retainer front' in S.objects

def export(obj,path):
    tmp=clone(obj,'_export')
    tmp.data.transform(Matrix.Scale(100,4)@tmp.matrix_world);tmp.matrix_world=Matrix.Identity(4)
    lo=[min(v.co[i] for v in tmp.data.vertices) for i in range(3)]
    hi=[max(v.co[i] for v in tmp.data.vertices) for i in range(3)]
    tmp.data.transform(Matrix.Translation(Vector((-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2]))));clean(tmp)
    expected=metrics(tmp)
    path.parent.mkdir(parents=True,exist_ok=True)
    bpy.ops.object.select_all(action='DESELECT');tmp.select_set(True);bpy.context.view_layer.objects.active=tmp
    bpy.ops.wm.stl_export(filepath=str(path),export_selected_objects=True,use_scene_unit=False,global_scale=1,forward_axis='Y',up_axis='Z')
    bpy.ops.object.select_all(action='DESELECT');bpy.ops.wm.stl_import(filepath=str(path),use_facet_normal=False)
    actual_obj=bpy.context.object;actual=metrics(actual_obj)
    assert actual['components']==1 and actual['nonmanifold_edges']==0 and actual['loose_vertices']==0 and actual['volume']>0,actual
    assert abs(actual['min_z'])<.001
    assert all(abs(x-y)<.002 for x,y in zip(expected['dimensions'],actual['dimensions']))
    assert all(x<=y+.001 for x,y in zip(actual['dimensions'],(220,220,250)))
    result={'components':actual['components'],'nonmanifold_edges':actual['nonmanifold_edges'],'loose_vertices':actual['loose_vertices'],
            'volume_mm3':actual['volume'],'dimensions_mm':actual['dimensions'],'min_z_mm':actual['min_z']}
    bpy.data.objects.remove(actual_obj,do_unlink=True);bpy.data.objects.remove(tmp,do_unlink=True)
    print('EXPORTED',path.name,json.dumps(result),flush=True)
    return result

# Fuse the same 1 mm channel support feet used by previous print exports.
print_deck=clone(deck,'_P04 print preparation')
for wall in [o for o in S.objects if o.name.startswith('Wiring | Cable channel')]:
    b=bounds(wall);piece=clone(wall,'_wall')
    bridge=box('_channel foot',((b[0][0]+b[0][1])/2,(b[1][0]+b[1][1])/2,(112.5+b[2][0])/2),
               (b[0][1]-b[0][0],b[1][1]-b[1][0],b[2][0]-112.5+.1))
    boolean(piece,bridge,'UNION');boolean(print_deck,piece,'UNION')

old_manifest=json.loads((OLD/'ROAM_STL_R05_K1C/manifest.json').read_text())
records=[]
for old in old_manifest['parts']:
    if old['id'] in ('P04','C04','S06'):continue
    row=json.loads(json.dumps(old))
    if row['id']=='S01':
        row['quantity']=12;row['sources']=[n for n in row['sources'] if not n.startswith('MCU |')]
        row['file']=row['file'].replace('qty14','qty12')
        row['note']='6 mm OD, 2.9 mm bore, 2.5 mm high. Pi x4, Geekworm x2, DRV boards x4, Nano x2; no unused MCU spacers.'
        assert len(row['sources'])==12
    dest=PACK/row['file'];dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(OLD/'ROAM_STL_R05_K1C'/old['file'],dest);assert HASH(dest)==row['sha256']
    records.append(row)
for id,label,obj,qty,folder,note in [
    ('P04','electronics_deck_with_cable_channels',print_deck,1,'01_rigid','R06: unused MCU holes filled; new 2.9 mm MUX holes at (-6,-2),(24,-2). Channel feet fused as in R05. Flat base down.'),
    ('C09','tca9548a_carrier',carrier,1,'01_rigid','Imported donor C08, renamed to avoid Geekworm C08 conflict. Base down. Trial-fit board first; holes in PCB not used.'),
    ('P20','multiplexer_end_clip',clips[0],2,'01_rigid','Two identical clips, flat underside down. M2x6 screws into 1.7 mm pilots; test threads without forcing.'),
    ('F06','multiplexer_soft_pad',pads[0],2,'03_flexible_optional','Imported donor F05, renamed to avoid Geekworm F05 conflict. Prefer cut 0.4 mm compliant tape; never rigid plastic.')]:
    filename=f'{folder}/{id}_{label}_qty{qty}.stl'
    checked=export(obj,PACK/filename)
    records.append({'id':id,'label':label,'sources':[deck.name if id=='P04' else obj.name],'quantity':qty,'folder':folder,'file':filename,
                    'note':note,'support':'No expected in supplied orientation; inspect holes and bridges in slicer',
                    'reimport_check':checked,'sha256':HASH(PACK/filename),'brim_4mm_fits_K1C':all(d+8<=220 for d in checked['dimensions_mm'][:2])})
bpy.data.objects.remove(print_deck,do_unlink=True)
assert len(records)==41 and sum(r['quantity'] for r in records)==116
assert not any(r['id'] in ('C04','S06') or 'mcu' in r['file'].lower() for r in records)
manifest={**old_manifest,'revision':'R06 Geekworm + TCA9548A; unused MCU removed','source':'roam_geekworm_mux_r06.blend',
          'unique_stl_files':len(records),'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),'parts':records,
          'unchanged_parts_source':str(OLD/'ROAM_STL_R05_K1C'),'replaces':['R05 P04','R05 C04 removed','R05 S06 removed','S01 quantity 14 -> 12'],
          'merge_part_id_map':{'donor C08 MUX carrier':'C09','donor F05 MUX pad':'F06','Geekworm C08 and F05':'unchanged'}}
(PACK/'manifest.json').write_text(json.dumps(manifest,indent=2))
with (PACK/'PRINT_PARTS.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
    for r in records:w.writerow([r['id'],r['label'],r['quantity'],r['file'],*[round(v,3) for v in r['reimport_check']['dimensions_mm']],r['support'],r['note']])
with (OLD/'ROAM_STL_R05_K1C/HARDWARE.csv').open(newline='') as f:hardware=list(csv.DictReader(f))
for row in hardware:
    if row['Modeled screw']=='M2.5x20':row['Quantity']='12';row['Notes']='R06 removes two unused MCU screws, in addition to the two old buck screws removed in R05.'
    if row['Modeled screw']=='M2.5x10':row['Quantity']='6';row['Notes']='Four Geekworm retainer screws plus two MUX deck screws. MUX additionally uses two nuts and four washers (reuse removed MCU nuts/washers if suitable).'
hardware.append({'Modeled screw':'M2x6','Quantity':'2','Notes':'MUX P20 clips; form threads in provisional 1.7 mm printed pilots; test gently.'})
with (PACK/'HARDWARE.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=['Modeled screw','Quantity','Notes']);w.writeheader();w.writerows(hardware)
for r in records:
    if r['id'] in ('P04','C09','P20','F06'):
        p=CHANGED/r['file'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/r['file'],p)
(CHANGED/'manifest.json').write_text(json.dumps({**manifest,'parts':[r for r in records if r['id'] in ('P04','C09','P20','F06')],
          'unique_stl_files':4,'total_physical_pieces_including_optional':6},indent=2))
shutil.copytree(OLD/'ROAM_STL_R05_K1C/R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
shutil.copy2(OLD/'ASSEMBLY_R05.md',PACK/'Geekworm_mount_R05_reference.md')
for target in (PACK,CHANGED):
    shutil.copy2(OUT/'ASSEMBLY_R06.md',target/'START_HERE.md')
    shutil.copy2(OUT/'geometry_checks.json',target/'geometry_checks.json')
    if target!=PACK:shutil.copy2(PACK/'HARDWARE.csv',target/'HARDWARE.csv')
    with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(target.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(OUT))

# Stage and save the assembled, inspection and exploded views.
for o in guides:o.hide_render=True;o.hide_set(True)
S['Mechanical revision']='R06 Geekworm + TCA9548A multiplexer; unused MCU removed'
S['Revision']=S['Mechanical revision']
S['Multiplexer fit']='Imported HW-617 outline 21.4x31.5; C09 + P20 + F06. Board thickness, headers and retention remain provisional.'
S['R06 removal']='Separate MCU board, carrier C04, S06 PCB spacers, two S01 spacers, hardware and label removed. Arduino Nano remains.'
S['Geometry checks']='geometry_checks.json in geekworm_mux_r06. Sources preserved; donor collection geometry unchanged except IDs.'
S.render.engine='CYCLES';S.cycles.samples=24;S.cycles.device='CPU'
S.render.resolution_x=1400;S.render.resolution_y=1100;S.render.resolution_percentage=100
def show(c,visible):
    for o in c.objects:o.hide_render=not visible;o.hide_set(not visible)
def aim(pos,target,span):
    cam=S.camera;cam.location=Vector(pos)/100;cam.rotation_euler=(Vector(target)/100-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.data.type='ORTHO';cam.data.ortho_scale=span/100
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=='VIEW_3D':
                view=area.spaces.active;view.region_3d.view_rotation=cam.rotation_euler.to_quaternion()
                view.region_3d.view_location=Vector(target)/100;view.region_3d.view_distance=span/100;view.overlay.show_extras=False
def save(name):bpy.ops.wm.save_as_mainfile(filepath=str(OUT/name),compress=True)
def render(name):
    if args.render:S.render.filepath=str(OUT/name);bpy.ops.render.render(write_still=True)
bpy.ops.object.select_all(action='DESELECT');carrier.select_set(True);bpy.context.view_layer.objects.active=carrier
aim((390,-600,360),(0,0,110),345);save('roam_geekworm_mux_r06.blend');render('robot_assembled.png')
for n in ('04 Sensor hood','07 Bolted sensor frame'):show(bpy.data.collections[n],False)
aim((135,-190,345),(0,-8,120),213);save('roam_geekworm_mux_r06_inspection.blend');render('electronics_installation.png')
for c in S.collection.children:
    if c!=MUX and c.name!='06 Studio':show(c,False)
aim((72,-86,211),(9,-2,122),72);render('multiplexer_detail.png')
# Standalone MUX assembly with only studio/camera retained.
for o in list(S.objects):
    if o.type not in ('CAMERA','LIGHT') and not any(c==MUX or c.name=='06 Studio' for c in o.users_collection):bpy.data.objects.remove(o,do_unlink=True)
bpy.data.orphans_purge(do_recursive=True)
save('tca9548a_mount_reference.blend')
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_geekworm_mux_r06.blend'));S=bpy.context.scene
for name in ('03 Electronics cassette','10 Geekworm - purchased reference, NOT FOR PRINT','11 R05 Geekworm - printable adapter and retainers',
             '12 R05 Geekworm - purchased fasteners and pads','13 R05 Geekworm - clearance guides, hidden','14 R06 TCA9548A multiplexer','15 R06 MUX connector guides - hidden'):
    for o in bpy.data.collections[name].objects:o.location.z+=.45
for o in bpy.data.collections['07 Bolted sensor frame'].objects:o.location.z+=.9
for o in bpy.data.collections['02 Battery drawer'].objects:o.location.y+=1.55
for o in bpy.data.collections['04 Sensor hood'].objects:o.location.z+=2.5
for name in ('05 Removable wheel skirt','09 Lower ToF sensors'):
    for o in bpy.data.collections[name].objects:o.location.x+=2.85
S['Exploded view']='R06 identification view: MUX and Geekworm follow electronics cassette. Not a collision-free disassembly animation.'
aim((490,700,460),(90,40,220),790);save('roam_geekworm_mux_r06_exploded.blend');render('robot_exploded.png')
assert HASH(SOURCE)==source_hash and HASH(DONOR)==donor_hash
print('R06_COMPLETE',str(OUT),flush=True)
