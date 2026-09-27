"""R12: two reinforced bolted side joints replace the thin R10/R11 locators.

Run in Blender background with --disable-autoexec; optional -- --render.
Scene units: 100 mm/unit. Does not modify any earlier revision.
"""
import ast, csv, hashlib, json, math, shutil, sys
from pathlib import Path
import bpy, bmesh
from mathutils import Matrix, Vector

ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/flat_roof_r11'
SOURCE=OLD/'roam_flat_roof_r11.blend'
OLDPACK=OLD/'ROAM_STL_R11_Flat_Roof_K1C'
OUT=ROOT/'mechanical/bolted_sides_r12'
PACK=OUT/'ROAM_STL_R12_Bolted_Sides_K1C'
CHANGED=OUT/'ROAM_R12_hood_replacement'
OUT.mkdir(parents=True,exist_ok=True)
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
SOURCE_HASH=HASH(SOURCE)
RENDER='--render' in sys.argv
PLANE=190.8
FN='P15C | Front slide-on smiling hood R12'
RN='P15D | Rear slide-on hood R12'
for file,names in [
 ('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','mesh_object','boolean','bounds','metrics','snapshot','intersection','envelope','ray_surface','move_collection'}),
 ('create_tof_solder_relief_r08.py',{'export','aim','render','material'})]:
 for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(node,ast.FunctionDef) and node.name in names:
   exec(compile(ast.Module(body=[node],type_ignores=[]),'<CAD helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update()
assert 'R11' in S['Mechanical revision'] and not S.get('Exploded view')
ORIGINAL={o.name:snapshot(o) for o in S.objects}
names={'P15C | Front slide-on smiling hood R11':FN,'P15D | Rear slide-on hood R11':RN}
for oldname,newname in names.items():S.objects[oldname].name=newname
front=S.objects[FN];rear=S.objects[RN];cap=S.objects['P15B | Removable hood side panel R07']
env=envelope([(113,102,102,0,.5),(113,102,107,0,.5),(111,101,157,0,.5),
 (109,99,176,0,.5),(105,94,185,0,.5),(97,85,190,0,.5),(52,52,191,5,1)])
C=bpy.data.collections.new('18 R12 bolted hood side joints - purchased hardware');S.collection.children.link(C)
joints=[];hardware=[]
def x_cylinder(name,x,y,z,r,h,n=64):
 ob=cyl(name,(0,0,0),r,h,n)
 ob.matrix_world=Matrix.Translation(Vector((x,y,z))/100)@Matrix.Rotation(math.pi/2,4,'Y')
 ob.data.transform(ob.matrix_world);ob.matrix_world=Matrix.Identity(4);clean(ob);return ob
def yz_prism(name,x0,x1,poly):
 n=len(poly)
 return mesh_object(name,[(x,y,z) for x in (x0,x1) for y,z in poly],
  [tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)])
for jid,z in [('H5',118.),('H6',151.)]:
 # Flat external washer seat, kept near the original side-wall envelope.
 wall=ray_surface(rear,(-200,12,z),(1,0,0));seat=wall.x-.25
 # Remove only the old 1 mm projecting tab, stopping exactly at the rear seam.
 boolean(rear,box('_remove old thin tab',(-112,-44.9,z),(25,110.2,12)))
 # Thick front clamping pad also completely fills the former locator slot.
 pad=box('_front clamp pad',(seat+1.8,.9,z),(3.6,17.8,20))
 boolean(front,pad,'UNION')
 # Substantial rear root, clipped inside the existing outside profile.
 root=box('_rear reinforced root',(seat+4.45,17.1,z),(8.9,13.8,20))
 boolean(root,clone(env,'_clip'),'INTERSECT');boolean(rear,root,'UNION')
 # 5 mm lap ear, not a snap clip. Clipped free corners remove sharp tips.
 # There is 0.30 mm clearance to the pad; local print support is allowed.
 ear=yz_prism('_5mm side lap',seat+3.9,seat+8.9,
   [(-6,z-6),(-5,z-7),(18,z-7),(18,z+7),(-5,z+7),(-6,z+6)])
 boolean(rear,ear,'UNION')
 for ob in [front,rear]:boolean(ob,x_cylinder('_M3 clearance',seat+5,2,z,1.7,35))
 boolean(rear,x_cylinder('_captive nut pocket',seat+7.6,2,z,5.9/math.sqrt(3),3,6))
 shaft=x_cylinder('_shaft',seat+5.5,2,z,1.45,12)
 head=x_cylinder('_head',seat-2,2,z,2.75,3)
 boolean(head,x_cylinder('_hex socket',seat-3.1,2,z,2.5/math.sqrt(3),1.4,6))
 boolean(shaft,head,'UNION');shaft.name=f'R12 {jid} | M3x12 screw'
 nut=x_cylinder('_nut',seat+7.3,2,z,5.5/math.sqrt(3),2.4,6)
 boolean(nut,x_cylinder('_nut bore',seat+7.3,2,z,1.4,5));nut.name=f'R12 {jid} | M3 captive nut'
 washer=x_cylinder('_washer',seat-.25,2,z,3,.5)
 boolean(washer,x_cylinder('_washer bore',seat-.25,2,z,1.7,2));washer.name=f'R12 {jid} | M3 washer'
 for ob in [shaft,nut,washer]:
  move_collection(ob,C);ob.data.materials.clear();ob.data.materials.append(bpy.data.materials['Machined metal'])
  ob['Reference only']='Purchased M3 hardware, not a print.';hardware.append(ob)
 joints.append({'id':jid,'seat_mm':[seat,2,z],'axis_inward':[1,0,0],
  'front_clamping_pad_thickness_mm':3.6,'rear_lap_thickness_mm':5.,
  'lap_gap_mm':.3,'bore_mm':3.4,'nut_pocket_AF_mm':5.9,'nut_pocket_depth_mm':2.8,
  'nut_shoulder_mm':2.2,'screw':'M3x12','nut':'M3 standard 5.5 AF x 2.4 mm',
  'washer':'M3 6 mm OD x 0.5 mm','nut_insertion':'From inside rear cover before installing it',
  'edit_bounds_mm':[[seat-.1,seat+9.5],[-8.1,24.1],[z-10.1,z+10.1]]})
remove(env)
for ob in [front,rear]:
 ob['R12 side joints']='H5/H6 reinforced 5 mm bolted laps; M3 nuts preloaded in rear, screws inserted from left outside.'
 m=metrics(ob);print('R12_SOLID',ob.name,json.dumps(m),flush=True)
 assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0,m
assert intersection(front,rear)<.03
for ob in [front,rear]:assert intersection(ob,cap)<.03
for h in hardware:
 for part in [front,rear]:assert intersection(h,part,inset=True)<.03,(h.name,part.name)
# One exact lower-joint crop pair, including the reinforced root and nut pocket.
coupons=[]
for ob,pid in [(front,'Q12C'),(rear,'Q12D')]:
 q=clone(ob,pid+' | H5 actual bolted side joint coupon')
 boolean(q,box('_coupon boundary',(-108,8,118),(30,36,26)),'INTERSECT');coupons.append(q)
printfront=clone(front,'_P15C smile union')
smiles=[o for o in S.objects if o.name.startswith('Smile |')]
for ob in smiles:boolean(printfront,clone(ob,'_smile'),'UNION')
# Same sub-micron cleanup as R10; not a shape-remeshing operation.
bm=bmesh.new();bm.from_mesh(printfront.data)
bmesh.ops.triangulate(bm,faces=list(bm.faces))
bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=.001/100)
bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=.0001/100)
bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
bm.to_mesh(printfront.data);bm.free();printfront.data.update()
oldmanifest=json.loads((OLDPACK/'manifest.json').read_text())
records=[];changed=[]
for row in oldmanifest['parts']:
 if row['id'] in {'P15C','P15D'}:continue
 path=PACK/row['file'];path.parent.mkdir(parents=True,exist_ok=True)
 shutil.copy2(OLDPACK/row['file'],path);assert HASH(path)==row['sha256'];records.append(row)
ROT=Matrix.Rotation(math.pi,4,'X')
for pid,label,ob,folder,note in [
 ('P15C','front_smiling_hood_bolted_sides',printfront,'01_rigid','Front cover; smile stays attached. Flat roof on bed.'),
 ('P15D','rear_hood_bolted_sides',rear,'01_rigid','Rear cover; preload H4/H5/H6 nuts before assembly. Flat roof on bed.'),
 ('Q12C','H5_side_clamp_test',coupons[0],'04_fit_test','Optional actual H5 front pad crop. Mate with Q12D; printed in the same orientation as the full hood.'),
 ('Q12D','H5_side_nut_lap_test',coupons[1],'04_fit_test','Optional actual H5 5 mm lap crop. Insert M3 nut first.')]:
 rot=ROT
 path=f'{folder}/{pid}_{label}_qty1.stl';m=export(ob,PACK/path,rot)
 row={'id':pid,'label':label,'sources':[FN if pid=='P15C' else ob.name],
  'quantity':1,'file':path,'folder':folder,'note':note+' 100%, millimetres; supplied orientation.',
  'support':('Flat roof on bed. Local support for camera bridge, shoulders, vents and side laps as applicable; inspect sliced preview.' if pid in {'P15C','P15D'} else 'Matching hood layer orientation. Inspect local support for projecting lap, nut pocket and bore.'),
  'reimport_check':m,'sha256':HASH(PACK/path),'brim_4mm_fits_K1C':all(d+8<220 for d in m['dimensions_mm'][:2]),
  'export_rotation_matrix':[list(r) for r in rot]}
 records.append(row);changed.append(row)
 dest=CHANGED/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/path,dest)
 print('R12_EXPORT',pid,json.dumps(m),flush=True)
for ob in coupons+[printfront]:remove(ob)
unexpected=[n for n,h in ORIGINAL.items() if n not in names and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
manifest={**oldmanifest,'revision':'R12 bolted side joints; R11 flat roof and attached smile retained',
 'source':'roam_bolted_sides_r12.blend','parts':records,'unique_stl_files':len(records),
 'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),
 'replaces':['R11 P15C + P15D with R12 P15C + P15D; Q12C/Q12D are new side-joint tests'],
 'unchanged_parts_source':str(OLDPACK),'new_optional_coupons':['Q12C','Q12D'],
 'hardware_change_from_R11':'Add two M3x12 screws, two standard M3 nuts and two M3 washers for H5/H6.',
 'additional_hardware':oldmanifest['additional_hardware']+joints}
manifest.pop('hardware_change_from_R10',None)
for target,rows in [(PACK,records),(CHANGED,changed)]:
 (target/'manifest.json').write_text(json.dumps({**manifest,'parts':rows,'unique_stl_files':len(rows),
   'total_physical_pieces_including_optional':sum(r['quantity'] for r in rows)},indent=2))
 with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
  wr=csv.writer(f);wr.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
  for r in rows:wr.writerow([r['id'],r['label'],r['quantity'],r['file'],*r['reimport_check']['dimensions_mm'],r['support'],r['note']])
with (OLDPACK/'HARDWARE.csv').open(newline='') as f:hw=list(csv.DictReader(f))
hw.append({'Modeled screw':'M3x12','Quantity':'2','Notes':'ADDITIONAL R12 H5/H6: also 2 standard M3 nuts + 2 M3 washers. Reuse all R11 hardware.'})
with (PACK/'HARDWARE.csv').open('w',newline='') as f:
 wr=csv.DictWriter(f,fieldnames=['Modeled screw','Quantity','Notes']);wr.writeheader();wr.writerows(hw)
prior=json.loads((OLD/'geometry_checks.json').read_text())
report={'revision':'R12 bolted sides','source_R11_sha256':SOURCE_HASH,'renamed_hood_objects':names,
 'roof_plane_mm':PLANE,'roof_joint':prior['roof_joint'],'side_joints':joints,
 'unchanged_roof_checks_from_R11':prior['roof_checks'],
 'unexpected_original_changes':unexpected,'unchanged_original_object_count':len(ORIGINAL)-2,
 'unchanged_STL_ids':[r['id'] for r in records if r not in changed],
 'design_choice':'Replace two thin seam tabs with bolted laps; retain attached face and flat roof. Local support allowed.',
 'motion_validation':'validate_bolted_sides_r12.py must pass before release.'}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
S['Mechanical revision']='R12 bolted side joints; R11 flat roof; R09 measured ToF / R06 electronics retained'
S['Revision']=S['Mechanical revision'];S['Geometry checks']='bolted_sides_r12/geometry_checks.json and validation.json'
S['R12 printing']='No raised LiDAR ring. Roof Z190.8 planar; keep face attached; local support allowed.'
bpy.ops.object.select_all(action='DESELECT');front.select_set(True);bpy.context.view_layer.objects.active=front
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_bolted_sides_r12.blend'),compress=True)
S.render.engine='CYCLES';S.cycles.samples=24;S.cycles.device='CPU'
S.render.resolution_x=1500;S.render.resolution_y=1150;S.render.resolution_percentage=100
aim((-360,-480,355),(0,0,140),330);render('hood_assembled.png')
# Isolate the actual hood to show the planar outer surface without LiDAR hiding it.
hood=[front,rear,cap]+smiles+hardware
for ob in S.objects:
 if ob.type not in {'LIGHT','CAMERA'} and ob not in hood:
  ob.hide_render=True;ob.hide_set(True)
aim((-350,80,240),(-95,7,144),115)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_bolted_sides_r12_hood_inspection.blend'),compress=True)
render('bolted_side_detail.png')
# Separate front cover to expose rear nut pockets and substantial lap ears.
for ob in [front]+smiles:ob.location.y-=.32
for ob in hardware:
 if 'screw' in ob.name or 'washer' in ob.name:ob.location.x-=.30
aim((40,-45,151),(-105,2,138),82)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_bolted_sides_r12_joint_inspection.blend'),compress=True)
render('side_joint_exploded.png')
for ob in [front]+smiles:ob.location.y+=.32
for ob in hardware:
 if 'screw' in ob.name or 'washer' in ob.name:ob.location.x+=.30
# Native CAD preview of the delivered print orientation, on two separate beds.
# It is a geometry illustration, NOT a sliced toolpath preview.
cap.hide_render=True;cap.hide_set(True)
for ob in hardware:ob.hide_render=True;ob.hide_set(True)
for group,bed_x in [([front]+smiles,-118),([rear],118)]:
 for ob in group:ob.matrix_world=ROT@ob.matrix_world
 bpy.context.view_layer.update()
 boxes=[bounds(ob) for ob in group]
 lo=[min(bb[i][0] for bb in boxes) for i in range(3)]
 hi=[max(bb[i][1] for bb in boxes) for i in range(3)]
 shift=Vector((bed_x-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2]))/100
 for ob in group:ob.matrix_world=Matrix.Translation(shift)@ob.matrix_world
bedmat=material('R12 print bed',(.055,.085,.12))
for x in [-118,118]:
 bed=box('Illustration | 220 x 220 print bed',(x,0,-1),(220,220,2))
 bed.data.materials.append(bedmat)
S['Exploded view']='Print orientation illustration: P15C/P15D roof down on separate 220 mm beds, no slicer supports shown.'
aim((400,-520,410),(0,0,30),525)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_bolted_sides_r12_print_inspection.blend'),compress=True)
render('roof_down_print_layout.png')
assert HASH(SOURCE)==SOURCE_HASH
print('R12_BUILD_COMPLETE',json.dumps({'new_STLs':len(changed),'unchanged_STLs':len(records)-len(changed)}),flush=True)
