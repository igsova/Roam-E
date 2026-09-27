"""R11: planar bed-contact roof, no raised LiDAR ring; attached smile retained.

Run in Blender background with --disable-autoexec; optional -- --render.
Scene units: 100 mm/unit. Does not modify any earlier revision.
"""
import ast, csv, hashlib, json, math, shutil, sys
from pathlib import Path
import bpy, bmesh
from mathutils import Matrix, Vector

ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/service_hood_r10'
SOURCE=OLD/'roam_service_hood_r10.blend'
OLDPACK=OLD/'ROAM_STL_R10_Service_Hood_K1C'
OUT=ROOT/'mechanical/flat_roof_r11'
PACK=OUT/'ROAM_STL_R11_Flat_Roof_K1C'
CHANGED=OUT/'ROAM_R11_hood_replacement'
OUT.mkdir(parents=True,exist_ok=True)
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
SOURCE_HASH=HASH(SOURCE)
RENDER='--render' in sys.argv
PLANE=190.8
THRESHOLD=190.0
FN='P15C | Front slide-on smiling hood R11'
RN='P15D | Rear slide-on hood R11'
for file,names in [
 ('create_split_shell_r07.py',{'clean','remove','clone','box','boolean','bounds','metrics','snapshot','intersection'}),
 ('create_tof_solder_relief_r08.py',{'export','aim','render','material'})]:
 for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(node,ast.FunctionDef) and node.name in names:
   exec(compile(ast.Module(body=[node],type_ignores=[]),'<CAD helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update()
assert 'R10' in S['Mechanical revision'] and not S.get('Exploded view')
ORIGINAL={o.name:snapshot(o) for o in S.objects}
names={'P15C | Front slide-on smiling hood R10':FN,'P15D | Rear slide-on hood R10':RN}
roof_checks={}
for oldname,newname in names.items():
 ob=S.objects[oldname];ob.name=newname
 assert ob.matrix_world==Matrix.Identity(4)
 # Clip the raised lip first. Moving an intact lip onto the roof would create
 # overlapping/collapsed surfaces. Its XY aperture and the inside skin stay put.
 boolean(ob,box('_remove raised roof ring',(0,0,PLANE+100),(600,600,200)))
 count=0
 for v in ob.data.vertices:
  if v.co.z*100>=THRESHOLD-.0001:
   v.co.z=PLANE/100;count+=1
 clean(ob)
 ob['R11 roof']='Flat exterior Z=190.8 mm. Raised LiDAR ring removed. Rounded shoulders retained.'
 ob['R11 print']='Roof down; smile attached. Local support at camera bridge, rounded edges, vents and joint details.'
 ob.data.calc_loop_triangles()
 flat=[t for t in ob.data.loop_triangles if all(abs(ob.data.vertices[i].co.z*100-PLANE)<.001 for i in t.vertices) and t.normal.z>.99]
 flat_area=sum(t.area*10000 for t in flat)
 thickness=[]
 for x in range(-110,105,2):
  for y in range(-100,101,2):
   hit,p,_,_=ob.ray_cast(Vector((x/100,y/100,3)),Vector((0,0,-1)))
   if not hit or abs(p.z*100-PLANE)>.001:continue
   hit,q,_,_=ob.ray_cast(p-Vector((0,0,.00001)),Vector((0,0,-1)))
   assert hit,(ob.name,x,y)
   thickness.append(float((p.z-q.z)*100))
 m=metrics(ob)
 assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0
 assert min(thickness)>2.3 and flat_area>9000,(ob.name,min(thickness),flat_area)
 roof_checks[ob.name]={'mesh':m,'flattened_vertex_count':count,'plane_z_mm':PLANE,
   'flat_bed_contact_area_mm2':flat_area,'thickness_sample_grid_mm':2,
   'thickness_sample_count':len(thickness),'min_sampled_flat_roof_thickness_mm':min(thickness)}
 print('R11_ROOF',ob.name,json.dumps(roof_checks[ob.name]),flush=True)
front=S.objects[FN];rear=S.objects[RN];cap=S.objects['P15B | Removable hood side panel R07']
assert intersection(front,rear)<.03
for ob in [front,rear]:assert intersection(ob,cap)<.03
coupons=[]
for ob,pid in [(front,'Q11C'),(rear,'Q11D')]:
 q=clone(ob,pid+' | H4 flat roof actual joint coupon')
 boolean(q,box('_coupon boundary',(-85,9,186),(20,28,18)),'INTERSECT');coupons.append(q)
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
 if row['id'] in {'P15C','P15D','Q10C','Q10D'}:continue
 path=PACK/row['file'];path.parent.mkdir(parents=True,exist_ok=True)
 shutil.copy2(OLDPACK/row['file'],path);assert HASH(path)==row['sha256'];records.append(row)
ROT=Matrix.Rotation(math.pi,4,'X')
for pid,label,ob,folder,note in [
 ('P15C','front_slide_on_smiling_hood_flat_roof',printfront,'01_rigid','Front cover; smile stays attached. Flat roof on bed.'),
 ('P15D','rear_slide_on_hood_flat_roof',rear,'01_rigid','Rear cover; insert H4 nut before assembly. Flat roof on bed.'),
 ('Q11C','H4_flat_roof_receiver_test',coupons[0],'04_fit_test','Optional actual R11 H4 receiver crop. Mate with Q11D.'),
 ('Q11D','H4_flat_roof_tongue_test',coupons[1],'04_fit_test','Optional actual R11 H4 tongue crop. Insert M3 nut first.')]:
 path=f'{folder}/{pid}_{label}_qty1.stl';m=export(ob,PACK/path,ROT)
 row={'id':pid,'label':label,'sources':[FN if pid=='P15C' else ob.name],
  'quantity':1,'file':path,'folder':folder,'note':note+' 100%, millimetres; supplied orientation.',
  'support':'Broad roof lies flat. Local support may be needed for camera bridge, rounded shoulders, rear vents and joint details. Inspect sliced preview.',
  'reimport_check':m,'sha256':HASH(PACK/path),'brim_4mm_fits_K1C':all(d+8<220 for d in m['dimensions_mm'][:2]),
  'export_rotation_matrix':[list(r) for r in ROT]}
 records.append(row);changed.append(row)
 dest=CHANGED/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/path,dest)
 print('R11_EXPORT',pid,json.dumps(m),flush=True)
for ob in coupons+[printfront]:remove(ob)
unexpected=[n for n,h in ORIGINAL.items() if n not in names and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
manifest={**oldmanifest,'revision':'R11 flat roof / removed LiDAR rim; attached smiling face; R10 assembly preserved',
 'source':'roam_flat_roof_r11.blend','parts':records,'unique_stl_files':len(records),
 'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),
 'replaces':['R10 P15C + P15D with R11 P15C + P15D; Q11C/Q11D replace optional Q10 tests'],
 'unchanged_parts_source':str(OLDPACK),'new_optional_coupons':['Q11C','Q11D'],
 'hardware_change_from_R10':'None. R10 H4 hardware retained unchanged.'}
for target,rows in [(PACK,records),(CHANGED,changed)]:
 (target/'manifest.json').write_text(json.dumps({**manifest,'parts':rows,'unique_stl_files':len(rows),
   'total_physical_pieces_including_optional':sum(r['quantity'] for r in rows)},indent=2))
 with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
  wr=csv.writer(f);wr.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
  for r in rows:wr.writerow([r['id'],r['label'],r['quantity'],r['file'],*r['reimport_check']['dimensions_mm'],r['support'],r['note']])
shutil.copy2(OLDPACK/'HARDWARE.csv',PACK/'HARDWARE.csv')
prior=json.loads((OLD/'geometry_checks.json').read_text())
report={'revision':'R11 flat roof','source_R10_sha256':SOURCE_HASH,'renamed_hood_objects':names,
 'roof_plane_mm':PLANE,'protected_geometry_below_z_mm':THRESHOLD,
 'roof_checks':roof_checks,'roof_joint':prior['roof_joint'],
 'unexpected_original_changes':unexpected,'unchanged_original_object_count':len(ORIGINAL)-2,
 'unchanged_STL_ids':[r['id'] for r in records if r not in changed],
 'design_choice':'User requested attached face and accepted local support; no new separate face part.',
 'motion_validation':'validate_flat_roof_r11.py must pass before release.'}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
S['Mechanical revision']='R11 flat roof; R10 horizontal service hood; R09 measured ToF / R06 electronics retained'
S['Revision']=S['Mechanical revision'];S['Geometry checks']='flat_roof_r11/geometry_checks.json and validation.json'
S['R11 printing']='No raised LiDAR ring. Roof Z190.8 planar; keep face attached; local support allowed.'
bpy.ops.object.select_all(action='DESELECT');front.select_set(True);bpy.context.view_layer.objects.active=front
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_flat_roof_r11.blend'),compress=True)
S.render.engine='CYCLES';S.cycles.samples=24;S.cycles.device='CPU'
S.render.resolution_x=1500;S.render.resolution_y=1150;S.render.resolution_percentage=100
aim((360,-480,355),(0,0,140),330);render('hood_assembled.png')
# Isolate the actual hood to show the planar outer surface without LiDAR hiding it.
hood=[front,rear,cap]+smiles
for ob in S.objects:
 if ob.type not in {'LIGHT','CAMERA'} and ob not in hood:
  ob.hide_render=True;ob.hide_set(True)
aim((260,-350,420),(0,0,150),280)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_flat_roof_r11_hood_inspection.blend'),compress=True)
render('flat_roof_detail.png')
# Native CAD preview of the delivered print orientation, on two separate beds.
# It is a geometry illustration, NOT a sliced toolpath preview.
cap.hide_render=True;cap.hide_set(True)
for group,bed_x in [([front]+smiles,-118),([rear],118)]:
 for ob in group:ob.matrix_world=ROT@ob.matrix_world
 bpy.context.view_layer.update()
 boxes=[bounds(ob) for ob in group]
 lo=[min(bb[i][0] for bb in boxes) for i in range(3)]
 hi=[max(bb[i][1] for bb in boxes) for i in range(3)]
 shift=Vector((bed_x-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2]))/100
 for ob in group:ob.matrix_world=Matrix.Translation(shift)@ob.matrix_world
bedmat=material('R11 print bed',(.055,.085,.12))
for x in [-118,118]:
 bed=box('Illustration | 220 x 220 print bed',(x,0,-1),(220,220,2))
 bed.data.materials.append(bedmat)
S['Exploded view']='Print orientation illustration: P15C/P15D roof down on separate 220 mm beds, no slicer supports shown.'
aim((400,-520,410),(0,0,30),525)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_flat_roof_r11_print_inspection.blend'),compress=True)
render('roof_down_print_layout.png')
assert HASH(SOURCE)==SOURCE_HASH
print('R11_BUILD_COMPLETE',json.dumps({'new_STLs':len(changed),'unchanged_STLs':len(records)-len(changed)}),flush=True)
