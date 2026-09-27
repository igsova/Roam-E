"""R14: replace P15B's two remaining passive tabs by H7/H8 bolted joints.
Blender background, --disable-autoexec, optional -- --render. 100 mm/BU.
R13 and all earlier revisions remain immutable. All new nuts use R13 ribs.
"""
import ast,csv,hashlib,json,math,shutil,sys
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/nut_retention_r13';SOURCE=OLD/'roam_nut_retention_r13.blend'
OLDPACK=OLD/'ROAM_STL_R13_Nut_Retention_K1C'
OUT=ROOT/'mechanical/panel_bolts_r14';PACK=OUT/'ROAM_STL_R14_Panel_Bolts_K1C';CHANGED=OUT/'ROAM_R14_hood_replacement'
OUT.mkdir(parents=True,exist_ok=True)
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
SOURCE_HASH=HASH(SOURCE);RENDER='--render' in sys.argv
FN='P15C | Front slide-on smiling hood R14';RN='P15D | Rear slide-on hood R14';CN='P15B | Removable hood side panel R14'
for file,names in [
 ('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','mesh_object','prism','boolean','bounds','metrics','snapshot','intersection','envelope','ray_surface','move_collection'}),
 ('create_tof_solder_relief_r08.py',{'export','aim','render','material'}),
 ('create_nut_retention_r13.py',{'ribs'})]:
 for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(node,ast.FunctionDef) and node.name in names:exec(compile(ast.Module(body=[node],type_ignores=[]),'<CAD helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE));bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update()
assert 'R13' in S['Mechanical revision'] and not S.get('Exploded view')
ORIGINAL={o.name:snapshot(o) for o in S.objects}
NAMES={'P15B | Removable hood side panel R13':CN,'P15C | Front slide-on smiling hood R12':FN,'P15D | Rear slide-on hood R13':RN}
for a,b in NAMES.items():S.objects[a].name=b
def freeze(ob):
 me=bpy.data.meshes.new_from_object(ob.evaluated_get(bpy.context.evaluated_depsgraph_get()));me.transform(ob.matrix_world);me.calc_loop_triangles()
 mesh=bpy.data.meshes.new(ob.name+' frozen triangles');mesh.from_pydata([v.co[:] for v in me.vertices],[],[tuple(t.vertices) for t in me.loop_triangles]);mesh.update()
 for m in me.materials:mesh.materials.append(m)
 for p,t in zip(mesh.polygons,me.loop_triangles):p.material_index=me.polygons[t.polygon_index].material_index;p.use_smooth=me.polygons[t.polygon_index].use_smooth
 ob.data=mesh;ob.modifiers.clear();ob.matrix_world=Matrix.Identity(4);bpy.data.meshes.remove(me)
for n in NAMES.values():freeze(S.objects[n])
front,rear,cap=[S.objects[n] for n in [FN,RN,CN]]
env=envelope([(113,102,102,0,.5),(113,102,107,0,.5),(111,101,157,0,.5),
 (109,99,176,0,.5),(105,94,185,0,.5),(97,85,190,0,.5),(52,52,191,5,1)])
hardware=[];joints=[];retention=[]
C=bpy.data.collections.new('19 R14 side panel end bolts - purchased hardware');S.collection.children.link(C)

for jid,sign,main in [('H7',-1,front),('H8',1,rear)]:
 z=135.;axis=Vector((0,-sign,0));u=Vector((1,0,0));v=Vector((0,0,1))
 # Seat clears the full 6 mm washer on the original curved outside wall.
 samples=[ray_surface(main,(89+3*math.cos(a),sign*200,z+3*math.sin(a)),axis) for a in [k*math.tau/32 for k in range(32)]]
 y=sign*(max(sign*p.y for p in samples)+.25)
 p=Vector((95,y,z));mat=Matrix(((u.x,v.x,axis.x,p.x/100),(u.y,v.y,axis.y,p.y/100),(u.z,v.z,axis.z,p.z/100),(0,0,0,1)))
 def local(ob):
  bpy.context.view_layer.update();ob.data.transform(mat@ob.matrix_world);ob.matrix_world=Matrix.Identity(4);clean(ob);return ob
 def cb(pos,dims):return local(box('_R14 box',pos,dims))
 def cy(pos,r,h,n=64):return local(cyl('_R14 bore',pos,r,h,n))
 # Cut only the old projecting tongue, not the side-panel shell/root.
 boolean(cap,cb((-14.925,0,10),(30.15,9.0,50)))
 # Main pad fills the former slot. A .30 mm receiver gap clears the new lap.
 boolean(main,cb((-6.475,0,1.8),(12.65,20,3.6)),'UNION')
 boolean(main,cb((-1,0,6.4),(24.6,14.6,5.6)))
 # Broad root follows the curved side panel and attaches a 5 mm-thick ear.
 root=cb((5.075,0,9),(9.85,20,18));boolean(root,clone(env,'_root envelope'),'INTERSECT');boolean(cap,root,'UNION')
 ear=local(prism('_R14 5mm ear',[(-12,-6),(-11,-7),(8,-7),(8,7),(-11,7),(-12,6)],3.9,8.9))
 boolean(ear,clone(env,'_ear envelope'),'INTERSECT');boolean(cap,ear,'UNION')
 for ob in [main,cap]:boolean(ob,cy((-6,0,5),1.7,70))
 boolean(cap,cy((-6,0,7.6),5.9/math.sqrt(3),3.,6))
 pocketmat=mat@Matrix.Translation(Vector((-6,0,6.1))/100)
 ribs(cap,pocketmat,5.3)
 # 10 mm reaches through the complete nut with ~1 mm protrusion. A 12 mm
 # screw would contact the installed right sensor frame at the front joint.
 shaft=cy((-6,0,4.5),1.45,10);head=cy((-6,0,-2),2.75,3)
 boolean(head,cy((-6,0,-3.1),2.5/math.sqrt(3),1.4,6));boolean(shaft,head,'UNION');shaft.name=f'R14 {jid} | M3x10 screw'
 nut=cy((-6,0,7.3),5.5/math.sqrt(3),2.4,6);boolean(nut,cy((-6,0,7.3),1.4,5));nut.name=f'R14 {jid} | M3 captive nut'
 washer=cy((-6,0,-.25),3,.5);boolean(washer,cy((-6,0,-.25),1.7,2));washer.name=f'R14 {jid} | M3 washer'
 for ob in [shaft,nut,washer]:
  move_collection(ob,C);ob.data.materials.clear();ob.data.materials.append(bpy.data.materials['Machined metal']);ob['Reference only']='Purchased M3 hardware, not a print';hardware.append(ob)
 qb=[mat@(Vector((uu,vv,ww))/100)*100 for uu in [-15.1,10.1] for vv in [-10.1,10.1] for ww in [-16,35.1]]
 joints.append({'id':jid,'main':main.name,'cap':CN,'sign':sign,'local_to_world_matrix':[list(r) for r in mat],
  'seat_mm':list(p+u*(-6)),'axis_inward':list(axis),'screw':'M3x10','washer':'M3 6 mm OD x 0.5 mm',
  'nut':'standard M3 5.5 AF x 2.4 mm','lap_thickness_mm':5.,'pad_thickness_mm':3.6,'receiver_gap_mm':.3,
  'nut_shoulder_mm':2.2,'bore_mm':3.4,'pocket_AF_mm':5.9,'pocket_depth_mm':2.8,
  'edit_bounds_mm':[[min(q[i] for q in qb),max(q[i] for q in qb)] for i in range(3)]})
 retention.append({'id':jid,'part':CN,'nut_object':nut.name,'local_to_world_matrix':[list(r) for r in pocketmat],
  'rib_tip_AF_mm':5.3,'nominal_nut_AF_mm':5.5,'pocket_AF_mm':5.9,'pocket_depth_mm':2.8,'rib_count':6,'profile':'standard'})
 print('R14_JOINT',json.dumps(joints[-1]),flush=True)
remove(env)
for ob in [front,rear,cap]:
 m=metrics(ob);print('R14_SOLID',ob.name,json.dumps(m),flush=True)
 assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0,m
 ob['R14 panel joints']='H7 front and H8 rear replace both thin P15B locating tabs. Remove bolts before sliding panel.'
for a,b in [(front,cap),(rear,cap),(front,rear)]:
 overlap=intersection(a,b);print('R14_PART_CLEARANCE',a.name,b.name,overlap,flush=True);assert overlap<.03
# Find any changed-cover/new-fastener penetration into the nearby fixed frame.
for ob in [front,rear,cap]+hardware:
 overlap=intersection(ob,S.objects['P07R | Sensor side frame'],inset=False)
 print('R14_FRAME_CLEARANCE',ob.name,overlap,flush=True);assert overlap<.03,(ob.name,overlap)

oldmanifest=json.loads((OLDPACK/'manifest.json').read_text());records=[];changed=[]
for row in oldmanifest['parts']:
 if row['id'] in {'P15B','P15C','P15D'}:continue
 path=PACK/row['file'];path.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(OLDPACK/row['file'],path)
 assert HASH(path)==row['sha256'];records.append(row)
ROT=Matrix.Rotation(math.pi,4,'X');SIDE=Matrix.Rotation(math.pi/2,4,'Y')
def emit(pid,label,ob,rot,folder,note):
 path=f'{folder}/{pid}_{label}_qty1.stl';m=export(ob,PACK/path,rot)
 row={'id':pid,'label':label,'sources':[ob.name],'quantity':1,'file':path,'folder':folder,'note':note+' 100%, millimetres; keep supplied orientation.',
  'support':'Inspect local supports at ears, pockets, curved shoulders and camera bridge as applicable. Support/print time not measured.',
  'reimport_check':m,'sha256':HASH(PACK/path),'export_rotation_matrix':[list(r) for r in rot],
  'brim_4mm_fits_K1C':all(d+8<220 for d in m['dimensions_mm'][:2])}
 records.append(row);changed.append(row);dest=CHANGED/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/path,dest)
 print('R14_EXPORT',pid,json.dumps(m),flush=True)
smiles=[o for o in S.objects if o.name.startswith('Smile |')]
pf=clone(front,'_P15C fused smile')
for ob in smiles:boolean(pf,clone(ob,'_smile'),'UNION')
bm=bmesh.new();bm.from_mesh(pf.data);bmesh.ops.triangulate(bm,faces=list(bm.faces));bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=.001/100);bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=.0001/100);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(pf.data);bm.free();pf.data.update()
emit('P15B','side_panel_all_bolted',cap,SIDE,'01_rigid','H1-H3 retained roof nuts plus H7/H8 reinforced end ears and retained nuts. Both thin end tabs removed.')
emit('P15C','front_hood_panel_bolt',pf,ROT,'01_rigid','Front smiling cover with matching H7 clamp pad. Flat roof down, face attached.')
emit('P15D','rear_hood_panel_bolt',rear,ROT,'01_rigid','Rear cover with matching H8 clamp pad. Flat roof down. H4-H6 retained nuts unchanged.')
remove(pf)
for jid,sign,main,pids in [('H7',-1,front,['Q14A','Q14B']),('H8',1,rear,['Q14C','Q14D'])]:
 for ob,pid,rot in [(main,pids[0],ROT),(cap,pids[1],SIDE)]:
  q=clone(ob,pid+' | '+jid+' actual joint crop')
  boolean(q,box('_coupon crop',(94,sign*85,135),(32,40,28)),'INTERSECT')
  emit(pid,jid+('_clamp_fit' if ob==main else '_nut_lap_fit'),q,rot,'04_fit_test','Optional actual '+jid+' coupon; test the pair with M3x10 screw, nut and washer before all three covers.');remove(q)
unexpected=[n for n,h in ORIGINAL.items() if n not in NAMES and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
prior=json.loads((OLD/'geometry_checks.json').read_text())
for j in prior['nut_retention']:j['part']=NAMES.get(j['part'],j['part'])
report={'revision':'R14','source_R13_sha256':SOURCE_HASH,'renamed_hood_objects':NAMES,'panel_joints':joints,
 'roof_joint':prior['roof_joint'],'side_joints':prior['side_joints'],'nut_retention':prior['nut_retention']+retention,
 'roof_plane_mm':190.8,'unexpected_original_changes':unexpected,'unchanged_original_object_count':len(ORIGINAL)-3,
 'design':'Both original P15B thin end tabs replaced by 5 mm bolted laps. New H7/H8 nuts use R13 standard PETG retention ribs.',
 'physical_fit':'Unverified; print Q14A/B and Q14C/D first. R13 retention samples still apply.'}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
manifest={**oldmanifest,'revision':'R14 all hood seam connectors bolted','source':'roam_panel_bolts_r14.blend','parts':records,
 'unique_stl_files':len(records),'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),
 'replaces':['P15B/P15C/P15D must all be updated from R13 for the new H7/H8 panel end joints.'],
 'unchanged_parts_source':str(OLDPACK),'new_optional_coupons':['Q14A','Q14B','Q14C','Q14D'],
 'hardware_change_from_R13':'Add 2 M3x10 screws, 2 standard M3 nuts, 2 M3 washers for H7/H8.',
 'nut_retention':report['nut_retention'],'additional_hardware':oldmanifest['additional_hardware']+joints}
manifest.pop('hardware_change_from_R12',None)
for target,rows in [(PACK,records),(CHANGED,changed)]:
 (target/'manifest.json').write_text(json.dumps({**manifest,'parts':rows,'unique_stl_files':len(rows),'total_physical_pieces_including_optional':sum(r['quantity'] for r in rows)},indent=2))
 with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
  wr=csv.writer(f);wr.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
  for r in rows:wr.writerow([r['id'],r['label'],r['quantity'],r['file'],*r['reimport_check']['dimensions_mm'],r['support'],r['note']])
with (OLDPACK/'HARDWARE.csv').open(newline='') as f:hw=list(csv.DictReader(f))
hw.append({'Modeled screw':'M3x10','Quantity':'2','Notes':'ADDITIONAL R14 H7/H8: also 2 standard M3 nuts + 2 M3 washers. Do not use longer bolts: frame clearance. Reuse all R13 hardware.'})
with (PACK/'HARDWARE.csv').open('w',newline='') as f:
 wr=csv.DictWriter(f,fieldnames=['Modeled screw','Quantity','Notes']);wr.writeheader();wr.writerows(hw)
S['Mechanical revision']='R14 all hood end connectors bolted; R13 nut retention; R11 flat roof; R09 ToF fit'
S['Revision']=S['Mechanical revision'];S['Geometry checks']='panel_bolts_r14/geometry_checks.json and validation.json'
bpy.ops.object.select_all(action='DESELECT');cap.select_set(True);bpy.context.view_layer.objects.active=cap
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_panel_bolts_r14.blend'),compress=True)
S.render.engine='CYCLES';S.cycles.samples=24;S.cycles.device='CPU';S.render.resolution_x=1500;S.render.resolution_y=1100;S.render.resolution_percentage=100
aim((370,-490,335),(0,0,140),330);render('hood_assembled.png')
hood=[front,rear,cap]+smiles+hardware
for ob in S.objects:
 if ob.type not in ['LIGHT','CAMERA'] and ob not in hood:ob.hide_render=True;ob.hide_set(True)
cap.location.x+=.45
for ob in hardware:
 if 'nut' in ob.name:ob.location.x+=.45
 else:ob.location.y+=(-.22 if 'H7' in ob.name else .22)
S['Exploded view']='Side panel +45 mm in X. H7/H8 nuts stay with panel; bolts/washers withdrawn along Y.'
aim((315,-430,285),(25,0,145),330)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_panel_bolts_r14_exploded.blend'),compress=True);render('panel_exploded.png')
for ob in S.objects:
 if ob.type not in ['LIGHT','CAMERA'] and ob!=cap:ob.hide_render=True;ob.hide_set(True)
cap.location.x-=.45
aim((-90,-185,200),(97,0,147),235)
S['Exploded view']='P15B isolated: three retained roof tongues and both reinforced bolted end ears.'
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_panel_bolts_r14_panel_inspection.blend'),compress=True);render('panel_inside.png')
assert HASH(SOURCE)==SOURCE_HASH
print('R14_BUILD_COMPLETE',json.dumps({'new_STLs':len(changed),'unchanged_STLs':len(records)-len(changed)}),flush=True)
