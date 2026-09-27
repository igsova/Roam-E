"""Export only the corrected P15C and retain the other 61 R15 STLs exactly."""
import ast,csv,hashlib,json,math,shutil,zipfile
from pathlib import Path
import bpy,bmesh
from mathutils import Vector,Matrix
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/camera_closure_r16';OLD=ROOT/'mechanical/sensor_enclosure_r15'
OLDPACK=OLD/'ROAM_STL_R15_Sensor_Enclosure_K1C';PACK=OUT/'ROAM_STL_R16_Camera_Closure_K1C';CHANGED=OUT/'ROAM_R16_front_hood_replacement'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','boolean','metrics','bounds'}),('create_tof_solder_relief_r08.py',{'export'}),('validate_panel_bolts_r14.py',{'mesh_arrays'}),('validate_camera_closure_r16.py',{'end_closure'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<export helper>','exec'))
qa=json.loads((OUT/'validation.json').read_text());assert qa['status']=='PASS' and not qa['quick_only']
assert HASH(OUT/'roam_camera_closure_r16.blend')==qa['master_sha256']
assert HASH(OUT/'geometry_checks.json')==qa['geometry_report_sha256']
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_camera_closure_r16.blend'));S=bpy.context.scene
front=S.objects['P15C | Closed camera surround front hood R16'];cap=S.objects['P15B | Raised removable hood side panel R15']
pf=clone(front,'_P15C R16 with attached smile')
for ob in [o for o in S.objects if o.name.startswith('Smile |')]:boolean(pf,clone(ob,'_attached smile'),'UNION')
bm=bmesh.new();bm.from_mesh(pf.data);bmesh.ops.triangulate(bm,faces=list(bm.faces));bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=.001/100);bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=.0001/100);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(pf.data);bm.free();pf.data.update()
rot=Matrix.Rotation(math.pi,4,'X');path='01_rigid/P15C_closed_camera_front_hood_R16_qty1.stl'
points=[rot@(pf.matrix_world@v.co*100) for v in pf.data.vertices]
lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
offset=Vector(((lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]))
check=export(pf,PACK/path,rot)
assert all(d+8<=220 for d in check['dimensions_mm'][:2]) and check['dimensions_mm'][2]<250
# Reimport the actual file, undo its bed placement, and rerun closure coverage.
bpy.ops.object.select_all(action='DESELECT');bpy.ops.wm.stl_import(filepath=str(PACK/path),use_facet_normal=False)
imp=bpy.context.object;imp.data.transform(Matrix.Scale(.01,4)@rot.inverted()@Matrix.Translation(offset));bpy.context.view_layer.update()
closure=end_closure(imp,cap);assert all(c['miss_count']==0 for c in closure.values()),closure
remove(imp);remove(pf)
old=json.loads((OLDPACK/'manifest.json').read_text());rows=[];retained=[]
row={'id':'P15C','label':'closed_camera_front_hood_R16','sources':[front.name,'Smile | attached decoration'],'quantity':1,'file':path,'folder':'01_rigid','revision':'R16','note':'Only replacement relative to R15. Keep existing R15 P15B/P15D and all hardware. Front opening and roof unchanged; camera return walls now continuous. Import mm at 100%, roof down.','support':'Local support may be required at camera bridge, shoulders and joints. Check actual sliced toolpaths.','reimport_check':check,'sha256':HASH(PACK/path),'export_rotation_matrix':[list(r) for r in rot],'brim_4mm_fits_K1C':True}
for r in old['parts']:
 if r['id']=='P15C':rows.append(row);continue
 p=PACK/r['file'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(OLDPACK/r['file'],p);assert HASH(p)==r['sha256'];rows.append(r);retained.append(r['id'])
assert len(retained)==61 and len(rows)==62
dest=CHANGED/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/path,dest)
export_qa={'status':'PASS','master_sha256':qa['master_sha256'],'new_STL_sha256':row['sha256'],'new_STL_metrics':check,'reimported_STL_camera_end_closure':closure,'retained_R15_STL_count':len(retained),'retained_R15_STL_ids':retained}
(OUT/'export_checks.json').write_text(json.dumps(export_qa,indent=2))
manifest={**old,'revision':'R16 camera-surround closure','source':'roam_camera_closure_r16.blend','replaces':['P15C only relative to R15. R15 side/rear hood and all other STLs retained.'],'master_sha256':qa['master_sha256']}
for target,records in [(PACK,rows),(CHANGED,[row])]:
 (target/'manifest.json').write_text(json.dumps({**manifest,'parts':records,'unique_stl_files':len(records),'total_physical_pieces_including_optional':sum(r['quantity'] for r in records)},indent=2))
 with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
  for r in records:w.writerow([r['id'],r['label'],r['quantity'],r['file'],*r['reimport_check']['dimensions_mm'],r['support'],r['note']])
 for name in ['ASSEMBLY_R16.md','validation.json','export_checks.json','geometry_checks.json','camera_closure_checks.json','hood_assembled.png','camera_end_left.png','camera_end_right.png']:
  shutil.copy2(OUT/name,target/('START_HERE.md' if name=='ASSEMBLY_R16.md' else name))
 shutil.copy2(OLD/'ASSEMBLY_R15.md',target/'R15_ASSEMBLY_REFERENCE.md');shutil.copy2(OLD/'MEASUREMENT_REVIEW.md',target/'MEASUREMENT_REVIEW.md')
for p in OLDPACK.iterdir():
 if p.is_file() and (p.name.endswith('_reference.md') or p.name in ['HARDWARE.csv','R14_HARDWARE_reference.csv','R07_JOINTS.csv']):shutil.copy2(p,PACK/p.name)
shutil.copytree(OLDPACK/'historical_R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
for target in [PACK,CHANGED]:
 with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
  for p in sorted(target.rglob('*')):
   if p.is_file():z.write(p,p.relative_to(OUT))
 with zipfile.ZipFile(OUT/(target.name+'.zip')) as z:
  assert z.testzip() is None
  records=json.loads((target/'manifest.json').read_text())['parts'];assert len([n for n in z.namelist() if n.endswith('.stl')])==len(records)
  for r in records:assert hashlib.sha256(z.read(target.name+'/'+r['file'])).hexdigest()==r['sha256']
assert HASH(OLD/'roam_sensor_enclosure_r15.blend')==qa['source_R15_sha256']
print('R16_EXPORT_COMPLETE',json.dumps(export_qa),flush=True)
