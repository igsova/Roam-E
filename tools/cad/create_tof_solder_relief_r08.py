"""R08: local, closed solder-tail reliefs in all eight R07 ToF pockets.

Blender --background --disable-autoexec --python THIS.py [-- --render]
No source revision is overwritten. Nominal 2.0 mm clearance MUST be checked
on the real soldered breakout with P19 before committing to a full skirt print.
"""
import ast
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

ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT/'mechanical/split_shell_r07'
SOURCE = OLD/'roam_split_shell_r07.blend'
OLDPACK = OLD/'ROAM_STL_R07_Split_Shell_K1C'
OUT = ROOT/'mechanical/tof_solder_relief_r08'
PACK = OUT/'ROAM_STL_R08_ToF_Solder_Relief_K1C'
CHANGED = OUT/'ROAM_R08_changed_parts'
OUT.mkdir(parents=True, exist_ok=True)
HASH = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
source_hash = HASH(SOURCE)
RENDER = '--render' in sys.argv

# Reuse geometry primitives only. Do not import/run the R07 generator's scene
# loading, mutation, exports or packaging. Its functions operate on this scene.
helpers = {'clean','remove','move_collection','clone','box','mesh_object',
           'boolean','bounds','metrics','snapshot','intersection'}
parsed = ast.parse((ROOT/'tools/cad/create_split_shell_r07.py').read_text())
for node in parsed.body:
    if isinstance(node, ast.FunctionDef) and node.name in helpers:
        exec(compile(ast.Module(body=[node], type_ignores=[]), '<R07 geometry helper>', 'exec'))

bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version = 0
S = bpy.context.scene
assert 'R07' in S['Mechanical revision'] and not S.get('Exploded view')
bpy.context.view_layer.update()
ORIGINAL = {o.name:snapshot(o) for o in S.objects}
main = S.objects['P16A | Main ToF skirt R07']
cap = S.objects['P16B | Removable ToF side panel R07']
oldcoupon = S.objects['P19 | ToF fit-test pocket']
oldcoupon_mesh = oldcoupon.data.copy()
main_original = clone(main, '_R07 main surface reference')
cap_original = clone(cap, '_R07 cap surface reference')

PCB_FACE = -.85
CLEARANCE = 2.0   # user estimates 1 mm projection; allow another 1 mm
FLOOR = PCB_FACE + CLEARANCE
SKIN = 1.2
OUTSIDE = FLOOR + SKIN
WIDTH = 16.2
HEIGHT = 4.2
ROW_V = -7.9
report = {'revision':'R08 ToF solder-tail relief', 'source_sha256':source_hash,
    'assumptions':{'solder_projection_precisely_measured':False,'user_estimated_projection_mm':1.0,
        'clearance_from_PCB_face_mm':CLEARANCE,'checked_pin_envelope_height_mm':1.5,
        'margin_above_user_estimate_mm':1.0,'channel_width_mm':WIDTH,'channel_height_mm':HEIGHT,
        'external_closed_skin_mm':SKIN,'external_bump_from_original_flat_face_mm':OUTSIDE},
    'stations':[], 'clearance_checks':[], 'addition_clashes':[],
    'optical_ray_obstructions':[], 'bed_fit_is_geometric_only':True}
tags = ['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']
added = []
matrices = {}

def local_box(name, centre, size, mat):
    o=box(name,centre,size);bpy.context.view_layer.update()
    o.matrix_world=mat@o.matrix_world
    return o

def bump(mat):
    # Closed tapered bulge: shallow base overlap into existing panel. Lower edge
    # ramps outward over 4.2 mm of height, avoiding a horizontal shelf on P16A.
    rings=[(-.4,10.2,-15.5,-4.1),(OUTSIDE,9.4,-11.3,-4.5)]
    vs=[]
    for w,half,low,high in rings:
        vs += [(-half,low,w),(half,low,w),(half,high,w),(-half,high,w)]
    o=mesh_object('_closed solder bulge',vs,
        [(3,2,1,0),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
    o.matrix_world=mat
    return o

def tree(o):
    tmp=clone(o,'_BVH');bm=bmesh.new();bm.from_mesh(tmp.data)
    t=BVHTree.FromBMesh(bm);bm.free();remove(tmp);return t

def ray(obj,mat,u,v,w,direction):
    p=mat@(Vector((u,v,w))/100)
    n=mat.to_3x3()@Vector((0,0,direction))
    inv=obj.matrix_world.inverted()
    hit,at,_,_=obj.ray_cast(inv@p,(inv.to_3x3()@n).normalized())
    return (obj.matrix_world@at-p).length*100 if hit else None

for tag in tags:
    ref=S.objects['ToF '+tag+' | Reference PCB 25x12.6']
    mat=ref.matrix_world.copy();matrices[tag]=mat
    part=cap if tag=='Right' else main
    b=bump(mat)
    added.append((tag,clone(b,'_new material region '+tag)))
    boolean(part,b,'UNION')
    # Opens only toward the inner PCB recess. The new external skin stays closed.
    boolean(part,local_box('_solder row pocket',(0,ROW_V,(FLOOR-1.4)/2),
                          (WIDTH,HEIGHT,FLOOR+1.4),mat))
    checked=metrics(part)
    assert checked['components']==1 and checked['nonmanifold_edges']==0,(tag,checked)
    depths=[];skins=[]
    for u in [-7.8,-6.35,-3.81,-1.27,0,1.27,3.81,6.35,7.8]:
        for v in [ROW_V-1.9,ROW_V,ROW_V+1.9]:
            d=ray(part,mat,u,v,PCB_FACE,1)
            t=ray(part,mat,u,v,OUTSIDE+1,-1)
            assert d is not None and abs(d-CLEARANCE)<.005,(tag,u,v,d)
            assert t is not None and abs(t-1)<.005,(tag,u,v,t)
            depths.append(d);skins.append(OUTSIDE+1-t-PCB_FACE-d)
    assert min(skins)>1.19
    # Verify a 1.5 mm pin/solder envelope is empty with 0.5 mm tip gap.
    probe=local_box('_pin envelope',(0,ROW_V,PCB_FACE+.75),(15.4,3.6,1.5),mat)
    vol=intersection(part,probe);remove(probe)
    assert vol<.02,(tag,'pin envelope',vol)
    # Original board supports, optic aperture and ear seating surfaces unchanged.
    support=[]
    for u,v in [(0,-4),(10.9,-3.2),(-10.9,-3.2)]:
        support.append(ray(part,mat,u,v,PCB_FACE-.1,1))
    assert all(d is not None and abs(d-.1)<.005 for d in support),support
    for u,v in [(0,0),(-2.2,0),(2.2,0),(0,1.3),(0,-1.3)]:
        assert ray(part,mat,u,v,-1,1) is None,(tag,'optics obstructed',u,v)
    report['clearance_checks'].append({'station':tag,'sample_count':len(depths),
        'min_clearance_mm':min(depths),'max_clearance_mm':max(depths),
        'min_skin_mm':min(skins),'pin_envelope_intersection_mm3':vol,
        'unchanged_seating_probe_distances_mm':support})
    report['stations'].append({'station':tag,'part':part.name,
        'local_to_world_matrix':[[float(v) for v in r] for r in mat]})
    print('R08_STATION',tag,flush=True)

# Confirm added material does not touch any other existing visible mesh. The
# prism overlaps the skirt intentionally; unchanged parts are not modified.
environment=[o for o in S.objects if o.name in ORIGINAL and o not in (main,cap,oldcoupon)
    and o.type=='MESH' and not o.hide_render and not any(c.name=='06 Studio' for c in o.users_collection)]
for tag,b in added:
    for other in environment:
        vol=intersection(b,other)
        if vol>.03:report['addition_clashes'].append([tag,other.name,vol])
    remove(b)
assert not report['addition_clashes'],report['addition_clashes']

# Retain R07 split/joints: test the two actual updated solids and a short sample
# of positive-X cap separation. This does not replace a physical assembly test.
report['panel_separation']=[]
start=cap.matrix_world.copy()
for offset in [0,1,3,7,12,20,40]:
    cap.matrix_world=start.copy();cap.matrix_world.translation.x+=offset/100
    v=intersection(main,cap)
    report['panel_separation'].append({'offset_x_mm':offset,'intersection_mm3':v})
    assert v<.03
cap.matrix_world=start;bpy.context.view_layer.update()

# Compare original vertices outside the explicit eight local edit regions.
report['preserved_surfaces']=[]
for new,old in [(main,main_original),(cap,cap_original)]:
    t=tree(new);distances=[]
    invs=[m.inverted() for m in matrices.values()]
    for vert in old.data.vertices:
        point=old.matrix_world@vert.co
        local=[inv@point*100 for inv in invs]
        if any(abs(p.x)<10.5 and -15.8<p.y<-3.8 and -1.7<p.z<OUTSIDE+.3 for p in local):continue
        distances.append(t.find_nearest(point)[3]*100)
    assert max(distances)<.01,(new.name,max(distances))
    report['preserved_surfaces'].append({'part':new.name,'vertices_checked':len(distances),
        'max_error_mm':max(distances)})
for old in [main_original,cap_original]:remove(old)

# Conservative local 40-degree optical cone on the revised covers (both halves).
for tag,mat in matrices.items():
    for ox in [-1.7,-1,1,1.7]:
        origin=mat@(Vector((ox,0,.19))/100)
        for tilt in [0,10,20]:
            for azi in range(0,360,30):
                r=math.radians(tilt);a=math.radians(azi)
                direction=mat.to_3x3()@Vector((math.sin(r)*math.cos(a),math.sin(r)*math.sin(a),math.cos(r)))
                for part in [main,cap]:
                    inv=part.matrix_world.inverted()
                    hit,pt,_,_=part.ray_cast(inv@origin,(inv.to_3x3()@direction).normalized())
                    if hit and .001<(part.matrix_world@pt-origin).length*100<300:
                        report['optical_ray_obstructions'].append([tag,tilt,azi,part.name])
assert not report['optical_ray_obstructions'],report['optical_ray_obstructions']

# P19 is an actual cropped front-centre revised pocket, including the bulge.
coupon=clone(main,'_new P19')
coupon.data.transform(matrices['Front_C'].inverted())
boolean(coupon,box('_coupon boundary',(0,-3.2,-2.5),(43,27,15)),'INTERSECT')
oldcoupon.data=coupon.data.copy();remove(coupon)
oldcoupon['R08 fit']='2.0 mm solder clearance for approximately 1 mm pin projection. Test soldered board first.'
main['R08 modification']='Seven closed solder-row recesses; all other R07 features retained.'
cap['R08 modification']='One closed solder-row recess; all other R07 features retained.'
modified={main.name,cap.name,oldcoupon.name}
unexpected=[n for n,h in ORIGINAL.items() if n not in modified and snapshot(S.objects[n])!=h]
assert not unexpected,unexpected
report['unchanged_original_objects']=len(ORIGINAL)-len(modified)
report['unexpected_original_changes']=unexpected
report['solid_checks']={o.name:metrics(o) for o in [main,cap,oldcoupon]}
assert all(m['components']==1 and m['nonmanifold_edges']==0 and m['volume_mm3']>0 for m in report['solid_checks'].values())

def export(obj,path,rot):
    tmp=clone(obj,'_export');tmp.hide_set(False);tmp.hide_render=False
    tmp.data.transform(rot@Matrix.Scale(100,4));tmp.matrix_world=Matrix.Identity(4)
    pts=[v.co for v in tmp.data.vertices]
    lo=[min(p[i] for p in pts) for i in range(3)];hi=[max(p[i] for p in pts) for i in range(3)]
    tmp.data.transform(Matrix.Translation(Vector((-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2]))));clean(tmp)
    before=metrics(tmp,1);path.parent.mkdir(parents=True,exist_ok=True)
    bpy.ops.object.select_all(action='DESELECT');tmp.select_set(True);bpy.context.view_layer.objects.active=tmp
    bpy.ops.wm.stl_export(filepath=str(path),export_selected_objects=True,use_scene_unit=False,forward_axis='Y',up_axis='Z')
    bpy.ops.object.select_all(action='DESELECT');bpy.ops.wm.stl_import(filepath=str(path),use_facet_normal=False)
    imp=bpy.context.object;check=metrics(imp,1)
    assert check['components']==1 and check['nonmanifold_edges']==0 and check['volume_mm3']>0
    assert abs(check['min_z_mm'])<.001 and all(abs(a-b)<.002 for a,b in zip(before['dimensions_mm'],check['dimensions_mm']))
    assert all(d+4<220 for d in check['dimensions_mm'][:2])
    remove(imp);remove(tmp);return check

manifest=json.loads((OLDPACK/'manifest.json').read_text())
records=[];unchanged=[]
for row in manifest['parts']:
    row=dict(row);pid=row['id'];path=PACK/row['file'];path.parent.mkdir(parents=True,exist_ok=True)
    if pid not in ['P16A','P16B','P19']:
        shutil.copy2(OLDPACK/row['file'],path);assert HASH(path)==row['sha256'];unchanged.append(pid)
    else:
        obj={'P16A':main,'P16B':cap,'P19':oldcoupon}[pid]
        rot=Matrix.Identity(4) if pid=='P16A' else Matrix.Rotation(math.pi/2 if pid=='P16B' else math.pi,4,'Y' if pid=='P16B' else 'X')
        row['reimport_check']=export(obj,path,rot)
        row['sha256']=HASH(path);row['sources']=[obj.name]
        row['revision']='R08';row['solder_clearance_mm']=CLEARANCE
        row['export_rotation_matrix']=[[float(v) for v in r] for r in rot]
        row['brim_4mm_fits_K1C']=all(d+8<220 for d in row['reimport_check']['dimensions_mm'][:2])
        row['brim_2mm_fits_nominal_K1C']=True
        row['note']='R08 solder relief. '+row['note']
        if pid=='P16A':row['note']+=' Use at most 2 mm brim as a starting point; check bed exclusion regions and supports.'
        if pid=='P19':row['note']='R08 actual solder-relieved front-centre pocket. Test soldered board, P17, P18 and original M2.5 hardware before either skirt half.'
        row['support']+='; inspect the new solder recess/closed bulge and remove trapped support'
    records.append(row)
report['unchanged_STL_ids']=unchanged
assert len(unchanged)==42
manifest.update({'revision':'R08 solder-tail relief in R07 split skirt','source':'roam_tof_solder_relief_r08.blend',
    'parts':records,'unchanged_parts_source':str(OLDPACK),'changed_from_R07':['P16A','P16B','P19'],
    'R08_assumptions':report['assumptions']})
(PACK/'manifest.json').write_text(json.dumps(manifest,indent=2))
changed=[r for r in records if r['id'] in ['P16A','P16B','P19']]
CHANGED.mkdir(parents=True,exist_ok=True)
for row in changed:
    target=CHANGED/row['file'];target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/row['file'],target)
(CHANGED/'manifest.json').write_text(json.dumps({**manifest,'parts':changed,'unique_stl_files':3,'total_physical_pieces_including_optional':3},indent=2))
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
for name in ['HARDWARE.csv','R07_JOINTS.csv']:
    shutil.copy2(OLDPACK/name,PACK/name)
for target,rows in [(PACK,records),(CHANGED,changed)]:
    with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
        for r in rows:w.writerow([r['id'],r['label'],r['quantity'],r['file'],*[round(d,3) for d in r['reimport_check']['dimensions_mm']],r['support'],r['note']])

S['Mechanical revision']='R08 ToF solder-tail relief; R07 split covers and R06 electronics retained'
S['Revision']=S['Mechanical revision']
S['R08 solder relief']='All eight: 16.2 x 4.2 mm channel, 2.0 mm clearance above PCB, 1.2 mm closed external skin. Print P19 first.'
S['Geometry checks']='tof_solder_relief_r08/geometry_checks.json'
bpy.ops.object.select_all(action='DESELECT');main.select_set(True);bpy.context.view_layer.objects.active=main
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_tof_solder_relief_r08.blend'),compress=True)

def aim(pos,target,span):
    cam=S.camera;cam.location=Vector(pos)/100
    cam.rotation_euler=(Vector(target)/100-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.data.type='ORTHO';cam.data.ortho_scale=span/100
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=='VIEW_3D':
                v=area.spaces.active;v.region_3d.view_rotation=cam.rotation_euler.to_quaternion()
                v.region_3d.view_location=Vector(target)/100;v.region_3d.view_distance=span/100

def render(name):
    if RENDER:
        S.render.filepath=str(OUT/name);bpy.ops.render.render(write_still=True)
S.render.engine='CYCLES';S.cycles.samples=24;S.cycles.device='CPU'
S.render.resolution_x=1500;S.render.resolution_y=1050;S.render.resolution_percentage=100
render('robot_assembled.png')
for o in S.objects:
    if o not in [main,cap] and not o.name.startswith(('ToF ','P17 |','P18 |','T1.','T2.','T3.','T4.','T5.','T6.','T7.','T8.')) and o.type not in ['CAMERA','LIGHT'] and not any(c.name=='06 Studio' for c in o.users_collection):
        o.hide_render=True;o.hide_set(True)
aim((180,-300,270),(0,0,38),330)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_tof_solder_relief_r08_skirt_inspection.blend'),compress=True)
render('skirt_inspection.png')

# Side-by-side native-CAD comparison of the real P19 section, inside facing up.
for o in list(S.objects):
    if o.type not in ['CAMERA','LIGHT']:remove(o)
def material(name,color):
    m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=(*color,1);bs.inputs['Roughness'].default_value=.45
    return m
grey=material('Original grey',(.30,.36,.39));teal=material('R08 relief teal',(.025,.32,.35))
floor=material('Display floor',(.025,.039,.052));white=material('Label white',(.8,.9,.92))
def label(body,position,size):
    data=bpy.data.curves.new(body,'FONT');data.body=body;data.align_x='CENTER';data.size=size/100
    ob=bpy.data.objects.new(body,data);S.collection.objects.link(ob);ob.location=Vector(position)/100;data.materials.append(white)
old=bpy.data.objects.new('R07 reference pocket - DO NOT PRINT',oldcoupon_mesh);S.collection.objects.link(old)
bpy.ops.wm.stl_import(filepath=str(PACK/next(r['file'] for r in changed if r['id']=='P19')),use_facet_normal=False)
new=bpy.context.object;new.name='R08 P19 - actual print STL';new.data.transform(Matrix.Scale(.01,4))
old.data.transform(Matrix.Rotation(math.pi,4,'X'))
bb=bounds(old);old.location=Vector((0,0,-bb[2][0]))/100
# Center the old reference the same way as exported P19.
old.location.x-=sum(bb[0])/200;old.location.y-=sum(bb[1])/200
for ob,x,mat in [(old,-28,grey),(new,28,teal)]:
    ob.location.x+=x/100;ob.data.materials.clear();ob.data.materials.append(mat)
    for p in ob.data.polygons:p.material_index=0
label('R07  |  0.2 mm pad clearance',(-28,-23,.2),2.4)
label('R08  |  2.0 mm solder clearance',(28,-23,.2),2.4)
f=box('Display floor NOT PRINTABLE',(0,0,-1),(500,500,2));f.data.materials.append(floor)
aim((45,-80,125),(0,1,2),125)
S['Mechanical revision']='R08 pocket comparison; left R07 reference only, right R08 P19 fit coupon'
S['Not production layout']='Use supplied STLs; display text and floor are NOT printable.'
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_tof_solder_relief_r08_fit_comparison.blend'),compress=True)
render('solder_relief_comparison.png')
assert HASH(SOURCE)==source_hash
print('R08_COMPLETE',json.dumps({'changed_parts':[(r['id'],r['reimport_check']['dimensions_mm']) for r in changed],
    'unchanged_STLs':len(unchanged),'source_unchanged':True}),flush=True)
