"""R10: replace P15A only with fore/aft service sections; keep printed P15B.

Blender --background --disable-autoexec --python THIS.py -- --render
100 mm per scene unit; exported STLs are mm. Earlier revisions are read-only.
"""
import ast, csv, hashlib, json, math, shutil, sys
from pathlib import Path
import bpy, bmesh
from mathutils import Matrix, Vector

ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/tof_measured_fit_r09'
SOURCE=OLD/'roam_tof_measured_fit_r09.blend'
OLDPACK=OLD/'ROAM_STL_R09_Measured_ToF_K1C'
OUT=ROOT/'mechanical/service_hood_r10'
PACK=OUT/'ROAM_STL_R10_Service_Hood_K1C'
CHANGED=OUT/'ROAM_R10_hood_replacement'
OUT.mkdir(parents=True,exist_ok=True)
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
SOURCE_HASH=HASH(SOURCE)
RENDER='--render' in sys.argv
for file,names in [
 ('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','mesh_object','boolean','bounds','metrics','snapshot','intersection','envelope','ray_surface','move_collection'}),
 ('create_tof_solder_relief_r08.py',{'export','aim','render','material'})]:
 for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(node,ast.FunctionDef) and node.name in names:
   exec(compile(ast.Module(body=[node],type_ignores=[]),'<CAD helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update()
assert 'R09' in S['Mechanical revision'] and not S.get('Exploded view')
ORIGINAL={o.name:snapshot(o) for o in S.objects if o.type not in ('CAMERA','LIGHT')}
old=S.objects['P15A | Main smiling hood R07']
cap=S.objects['P15B | Removable hood side panel R07']
front=clone(old,'P15C | Front slide-on smiling hood R10')
rear=clone(old,'P15D | Rear slide-on hood R10')
SEAM=10.;GAP=.4;FIT=.3
def half(front):
 edge=SEAM+(-GAP/2 if front else GAP/2)
 return box('_half',(0,edge+(-250 if front else 250),150),(600,500,400))
for ob,is_front in [(front,True),(rear,False)]:
 boolean(ob,half(is_front),'INTERSECT');move_collection(ob,old.users_collection[0])
 ob['Manufacture']='PRINT';ob['R10 installation']='Horizontal '+('+Y from front' if is_front else '-Y from rear')+'; never lower over camera.'
env=envelope([(113,102,102,0,.5),(113,102,107,0,.5),(111,101,157,0,.5),
 (109,99,176,0,.5),(105,94,185,0,.5),(97,85,190,0,.5),(52,52,191,5,1)])

# New H4 lap joint, wholly on the left roof; existing H1/H2/H3 interface to
# P15B is untouched. Rear tongue enters the front receiver along Y.
p=ray_surface(old,(-85,SEAM,250),(0,0,-1))
u=Vector((0,1,0));v=Vector((-1,0,0));w=Vector((0,0,-1))
mat=Matrix(((u.x,v.x,w.x,p.x/100),(u.y,v.y,w.y,p.y/100),(u.z,v.z,w.z,p.z/100),(0,0,0,1)))
def local(ob):
 bpy.context.view_layer.update();ob.matrix_world=mat@ob.matrix_world
 ob.data.transform(ob.matrix_world);ob.matrix_world=Matrix.Identity(4);clean(ob);return ob
def cb(pos,dims):return local(box('_H4 box',pos,dims))
def cy(pos,r,h,n=64):return local(cyl('_H4 cylinder',pos,r,h,n))
def clipped(ob):return boolean(ob,clone(env,'_outer envelope'),'INTERSECT')
pad=clipped(cb((-6.1,0,(-12+4.2)/2),(11.8,14,16.2)))
boolean(front,pad,'UNION')
boolean(front,cb((-1,0,7),(22.6,14.6,5.6)))
root=clipped(cb((5.1,0,(1.4+9.5)/2),(9.8,14,8.1)))
boolean(rear,root,'UNION')
tongue=clipped(cb((-1,0,7),(22,14,5)))
boolean(rear,tongue,'UNION')
axis=p-6*u
depths=[]
for i in range(32):
 a=math.tau*i/32;q=axis+3*(u*math.cos(a)+v*math.sin(a))
 depths.append((ray_surface(old,q-w*100,w)-axis).dot(w))
seat=max(depths)+.25
for ob in (front,rear):boolean(ob,cy((-6,0,0),1.7,70))
boolean(front,cy((-6,0,seat-25),3.2,50))
boolean(rear,cy((-6,0,9.1),5.9/math.sqrt(3),4.8,6))
assert 4.2-seat>2.5
L=12
assert seat-.5+L>10.3
C=bpy.data.collections.new('17 R10 service hood joint - purchased hardware');S.collection.children.link(C)
shaft=cy((-6,0,seat-.5+L/2),1.45,L)
head=cy((-6,0,seat-2),2.75,3)
boolean(head,cy((-6,0,seat-3.1),2.5/math.sqrt(3),1.4,6))
boolean(shaft,head,'UNION');shaft.name='R10 H4 | M3x12 screw'
nut=cy((-6,0,8),5.5/math.sqrt(3),2.4,6);boolean(nut,cy((-6,0,8),1.4,5));nut.name='R10 H4 | M3 captive nut'
washer=cy((-6,0,seat-.25),3,.5);boolean(washer,cy((-6,0,seat-.25),1.7,2));washer.name='R10 H4 | M3 washer'
hardware=[shaft,nut,washer]
for ob in hardware:
 move_collection(ob,C);ob.data.materials.clear();ob.data.materials.append(bpy.data.materials['Machined metal'])
 ob['Reference only']='Purchased hardware. Not a print.'

# Two shallow sliding tabs on the left side. No hooks/vertical insertion.
locators=[]
for z in (118.,151.):
 pt=ray_surface(old,(-200,SEAM,z),(1,0,0))
 pos=(pt.x+1.9,SEAM,z)
 boolean(front,box('_locator clearance',pos,(1.6,10.6,8.6)))
 tab=clipped(box('_rear locating tab',pos,(1.,10.,8.)))
 boolean(rear,tab,'UNION')
 locators.append({'centre_mm':list(pos),'tongue_mm':[1.,10.,8.],'receiver_mm':[1.6,10.6,8.6]})
remove(env)
for ob in (front,rear):
 m=metrics(ob);print('R10_SOLID',ob.name,json.dumps(m),flush=True)
 assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 and m['volume_mm3']>0,m
assert intersection(front,rear)<.03
for ob in (front,rear):assert intersection(ob,cap)<.03
for ob in hardware:
 for part in (front,rear,cap):assert intersection(ob,part,inset=True)<.03,(ob.name,part.name)

# H4 sample is an exact crop of the actual new joint, not an idealised replica.
coupons=[]
for ob,pid in [(front,'Q10C'),(rear,'Q10D')]:
 q=clone(ob,pid+' | H4 actual joint coupon')
 boolean(q,box('_coupon boundary',(-85,9,186),(20,28,18)),'INTERSECT')
 coupons.append(q)
printfront=clone(front,'_P15C smile union')
smiles=[o for o in S.objects if o.name.startswith('Smile |')]
for ob in smiles:boolean(printfront,clone(ob,'_smile'),'UNION')
# Resolve sub-micron Boolean slivers where the inherited smile joins the skin
# before STL triangulation. This is a 0.001 mm weld, not a remesh/shape change.
bm=bmesh.new();bm.from_mesh(printfront.data)
bmesh.ops.triangulate(bm,faces=list(bm.faces))
bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=.001/100)
bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=.0001/100)
bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
bm.to_mesh(printfront.data);bm.free();printfront.data.update()
assert metrics(printfront)['nonmanifold_edges']==0
records=[];oldmanifest=json.loads((OLDPACK/'manifest.json').read_text())
for row in oldmanifest['parts']:
 if row['id']=='P15A':continue
 path=PACK/row['file'];path.parent.mkdir(parents=True,exist_ok=True)
 shutil.copy2(OLDPACK/row['file'],path);assert HASH(path)==row['sha256'];records.append(row)
ROOF_DOWN=Matrix.Rotation(math.pi,4,'X')
changed=[]
for pid,label,ob,folder,note in [
 ('P15C','front_slide_on_smiling_hood',printfront,'01_rigid','Replaces front of P15A. Fit horizontally from front after P15D; smile integrated.'),
 ('P15D','rear_slide_on_hood',rear,'01_rigid','Replaces rear of P15A. Fit horizontally from rear first, with H4 nut inserted.'),
 ('Q10C','H4_front_receiver_test',coupons[0],'04_fit_test','Optional actual H4 receiver crop. Test with Q10D.'),
 ('Q10D','H4_rear_tongue_test',coupons[1],'04_fit_test','Optional actual H4 tongue crop; insert M3 nut before mating.')]:
 path=f'{folder}/{pid}_{label}_qty1.stl';m=export(ob,PACK/path,ROOF_DOWN)
 row={'id':pid,'label':label,'sources':[front.name if pid=='P15C' else ob.name],
      'quantity':1,'file':path,'folder':folder,'note':note+' Supplied roof-down; 100% millimetres.',
      'support':'Local supports under curved roof/joint details; inspect sliced preview. Not support-free.',
      'reimport_check':m,'sha256':HASH(PACK/path),'brim_4mm_fits_K1C':all(d+8<220 for d in m['dimensions_mm'][:2]),
      'export_rotation_matrix':[list(r) for r in ROOF_DOWN]}
 records.append(row);changed.append(row)
 dest=CHANGED/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/path,dest)
 print('R10_EXPORT',pid,json.dumps(m),flush=True)
for ob in coupons+[printfront]:remove(ob)
remove(old)
unexpected=[n for n,h in ORIGINAL.items() if n!='P15A | Main smiling hood R07' and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
joint={'id':'R10 H4','screw':'M3x12','quantity':1,'captive_nut':'M3 standard 5.5 AF x 2.4 mm',
 'washer':'M3, 6 mm OD x 0.5 mm','seat_mm':list(axis+w*seat),'axis_inward':list(w),
 'bore_mm':3.4,'nut_recess_AF_mm':5.9,'nut_recess_depth_mm':2.8,'nut_shoulder_mm':2.2,
 'main_screw_shoulder_mm':4.2-seat,'tongue_clearance_per_face_mm':FIT}
manifest={**oldmanifest,'revision':'R10 horizontally assembled service hood; R09 measured ToF retained',
 'source':'roam_service_hood_r10.blend','parts':records,'unique_stl_files':len(records),
 'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),
 'replaces':['P15A -> P15C + P15D; retain printed P15B unchanged'],
 'unchanged_parts_source':str(OLDPACK),'additional_hardware':[joint],
 'new_optional_coupons':['Q10C','Q10D']}
for target,rows in [(PACK,records),(CHANGED,changed)]:
 (target/'manifest.json').write_text(json.dumps({**manifest,'parts':rows,'unique_stl_files':len(rows),
   'total_physical_pieces_including_optional':sum(r['quantity'] for r in rows)},indent=2))
 with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
  wri=csv.writer(f);wri.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
  for r in rows:wri.writerow([r['id'],r['label'],r['quantity'],r['file'],*r['reimport_check']['dimensions_mm'],r['support'],r['note']])
with (OLDPACK/'HARDWARE.csv').open(newline='') as f:hw=list(csv.DictReader(f))
hw.append({'Modeled screw':'M3x12','Quantity':'1','Notes':'ADDITIONAL R10 H4: also 1 M3 standard nut + 1 M3 washer. Reuse all R09 hardware.'})
with (PACK/'HARDWARE.csv').open('w',newline='') as f:
 wri=csv.DictWriter(f,fieldnames=['Modeled screw','Quantity','Notes']);wri.writeheader();wri.writerows(hw)
report={'revision':'R10 service hood','source_R09_sha256':SOURCE_HASH,'new_seam_y_mm':SEAM,'new_seam_gap_mm':GAP,
 'roof_joint':joint,'passive_locators':locators,'new_solid_checks':{o.name:metrics(o) for o in (front,rear)},
 'unchanged_original_object_count':len(ORIGINAL)-1,'unexpected_original_changes':unexpected,
 'unchanged_STL_ids':[r['id'] for r in records if r not in changed],
 'assembly_order':['P15D from rear (-Y travel)','P15C from front (+Y travel)','P15B from right (-X travel)','Install H4/H1-H3 and four K screws'],
 'motion_validation':'Separate validate_service_hood_r10.py must pass before release.'}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
S['Mechanical revision']='R10 service hood; R09 measured ToF / R06 electronics retained'
S['Revision']=S['Mechanical revision']
S['R10 assembly']='Replace only P15A with P15C/front + P15D/rear. Existing P15B fitted last. Horizontal paths; no vertical lowering over camera.'
S['Geometry checks']='service_hood_r10/geometry_checks.json and validation.json'
bpy.ops.object.select_all(action='DESELECT');front.select_set(True);bpy.context.view_layer.objects.active=front
# Preserve every existing object transform, including the original camera, in
# the master model. Presentation scenes below are saved under separate names.
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_service_hood_r10.blend'),compress=True)
S.render.engine='CYCLES';S.cycles.samples=24;S.cycles.device='CPU'
S.render.resolution_x=1500;S.render.resolution_y=1150;S.render.resolution_percentage=100
rear_mat=material('R10 rear shell muted blue',(.19,.32,.40))
rear.data.materials.clear();rear.data.materials.append(rear_mat)
for poly in rear.data.polygons:poly.material_index=0
aim((360,-480,355),(0,0,140),330);render('hood_assembled.png')

# Motion/inspection presentation: actual verified removal directions, with
# detachable screws out. The sensor frame stays assembled at its original pose.
groups={front.name:[front]+smiles,rear.name:[rear,nut],cap.name:[cap]}
for ob in S.objects:
 if ob.name.startswith(('P12 |','P14 |')):
  bb=bounds(ob)
  target=cap if ob.name.startswith('P12 |') and sum(bb[0])>0 else (front if sum(bb[1])/2<SEAM else rear)
  groups[target.name].append(ob)
 if ob.name in ['H1 | M3 captive nut','H2 | M3 captive nut','H3 | M3 captive nut']:groups[cap.name].append(ob)
 if ((ob.name.startswith('K') and ('screw' in ob.name or 'washer' in ob.name)) or
     ob.name in ['H1 | M3x12 screw','H2 | M3x12 screw','H3 | M3x12 screw','H1 | M3 washer','H2 | M3 washer','H3 | M3 washer'] or ob in [shaft,washer]):
  ob.hide_render=True;ob.hide_set(True)
for owner,direction in [(front,Vector((0,-1,0))),(rear,Vector((0,1,0))),(cap,Vector((1,0,0)))]:
 for ob in groups[owner.name]:ob.location+=direction*.80
S['Exploded view']='R10: horizontal separation by 80 mm with cover screws removed. Sensors/frame stay in place. See assembly guide for sequence.'
aim((390,-540,420),(20,0,155),445)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_service_hood_r10_assembly_inspection.blend'),compress=True)
render('hood_assembly_paths.png')
assert HASH(SOURCE)==SOURCE_HASH
print('R10_BUILD_COMPLETE',json.dumps({'new_STLs':len(changed),'unchanged_STLs':len(records)-len(changed),'extra_screw':'one M3x12'}),flush=True)
