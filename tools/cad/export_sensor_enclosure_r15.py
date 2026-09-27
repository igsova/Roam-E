"""Export R15 printable parts and exact fit samples from its checked master.
Run after validate_sensor_enclosure_r15.py passes. Optional -- --render.
"""
import ast,csv,hashlib,json,math,shutil,sys,zipfile
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/sensor_enclosure_r15';OLD=ROOT/'mechanical/panel_bolts_r14'
OLDPACK=OLD/'ROAM_STL_R14_Panel_Bolts_K1C';PACK=OUT/'ROAM_STL_R15_Sensor_Enclosure_K1C';CHANGED=OUT/'ROAM_R15_hood_replacement'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();RENDER='--render' in sys.argv
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','boolean','metrics','bounds','intersection'}),('create_tof_solder_relief_r08.py',{'export','aim','render','material'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<CAD helper>','exec'))
v=json.loads((OUT/'validation.json').read_text());assert v['status']=='PASS' and not v['quick_only']
assert HASH(OUT/'roam_sensor_enclosure_r15.blend')==v['master_sha256'],'Stale model validation'
assert HASH(OUT/'geometry_checks.json')==v['geometry_report_sha256'],'Stale geometry validation'
report=json.loads((OUT/'geometry_checks.json').read_text());d=report['datums']
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_sensor_enclosure_r15.blend'));S=bpy.context.scene;bpy.context.preferences.filepaths.save_version=0
CN='P15B | Raised removable hood side panel R15';FN='P15C | Enclosed camera front hood R15';RN='P15D | Raised rear hood R15'
cap,front,rear=[S.objects[n] for n in [CN,FN,RN]];ROT=Matrix.Rotation(math.pi,4,'X');SIDE=Matrix.Rotation(math.pi/2,4,'Y')
old=json.loads((OLDPACK/'manifest.json').read_text());records=[];changed=[]
for row in old['parts']:
 if row['id'] in {'P15B','P15C','P15D','F03','P11'}:continue
 path=PACK/row['file'];path.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(OLDPACK/row['file'],path);assert HASH(path)==row['sha256'];records.append(row)
def emit(pid,label,ob,rot,folder,note,qty=1):
 path=f'{folder}/{pid}_{label}_qty{qty}.stl';m=export(ob,PACK/path,rot)
 assert m['components']==1 and not m['nonmanifold_edges'] and not m['loose_vertices'] and m['volume_mm3']>0,m
 assert m['dimensions_mm'][0]+8<=220.001 and m['dimensions_mm'][1]+8<=220.001 and m['dimensions_mm'][2]<250,m
 row={'id':pid,'label':label,'sources':[ob.name],'quantity':qty,'file':path,'folder':folder,'note':note+' Import millimetres at 100%; retain supplied orientation.',
 'support':'Check local supports at camera bridge, shoulders, vents, nuts and joint details. Physical slicing not performed.',
 'reimport_check':m,'sha256':HASH(PACK/path),'export_rotation_matrix':[list(r) for r in rot],'brim_4mm_fits_K1C':True}
 records.append(row);changed.append(row);dest=CHANGED/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/path,dest);print('R15_EXPORT',pid,json.dumps(m),flush=True)
pf=clone(front,'_P15C with attached smile')
for ob in [o for o in S.objects if o.name.startswith('Smile |')]:boolean(pf,clone(ob,'_smile'),'UNION')
bm=bmesh.new();bm.from_mesh(pf.data);bmesh.ops.triangulate(bm,faces=list(bm.faces));bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=.001/100);bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=.0001/100);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(pf.data);bm.free();pf.data.update()
emit('P15B','raised_side_panel_retained_nuts',cap,SIDE,'01_rigid','Replace matching R14 side panel; all bolted joints retained.')
emit('P15C','measured_camera_front_hood',pf,ROT,'01_rigid','Raised planar roof; attached smile; close-fit camera surround. Replace together with P15B/P15D.')
emit('P15D','raised_lidar_rear_hood',rear,ROT,'01_rigid','Raised planar roof and locally rerouted LiDAR seam; replace with P15B/P15C.')
remove(pf)
emit('F03','camera_upper_pad_1p20mm',S.objects['P08 | Camera upper compliant pad'],Matrix.Identity(4),'03_flexible_optional','Prefer cut compliant foam of verified compressed thickness. New nominal 1.20 mm upper pad, not a rigid shim.',2)
q=clone(front,'Q15A | Actual camera opening fit ring')
boolean(q,box('_camera coupon',(0,d['camera_front_y_mm']+1.35,161.65),(185,2.6,47)),'INTERSECT')
emit('Q15A','camera_face_fit_ring',q,Matrix.Rotation(math.pi/2,4,'X'),'04_fit_test','Optional exact front surround crop: test against camera face before full hood.');remove(q)
for pid,ob in [('Q15B',front),('Q15C',rear)]:
 q=clone(ob,pid+' | Actual LiDAR roof aperture half')
 boolean(q,cyl('_lidar test radius',(*d['aperture_centre_mm'],d['roof_z_mm']-1.4),45,3.2,192),'INTERSECT')
 emit(pid,'lidar_aperture_'+('front' if ob==front else 'rear')+'_test',q,ROT,'04_fit_test','Optional aperture half. Test both halves together around UNPOWERED rotor; support at proposed roof level; never rest on rotating optics.');remove(q)
manifest={'revision':'R15 measured LiDAR and camera enclosure','source':'roam_sensor_enclosure_r15.blend','parts':records,'unique_stl_files':len(records),
 'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),'measurements':report['measurements'],'datums':d,
 'replaces':['P15B/P15C/P15D together; upper camera pad F03 becomes nominal 1.20 mm.'],
 'omitted':['P11 four 4 mm printed LiDAR spacers: measured brass posts stand directly on P10; do not add spacer height again.'],
 'purchased_hardware':'Reuse R14 H1-H8/K hardware. LiDAR uses actual measured 44.78 mm brass posts, not printed replicas; verify existing sensor fasteners.'}
for target,rows in [(PACK,records),(CHANGED,changed)]:
 (target/'manifest.json').write_text(json.dumps({**manifest,'parts':rows,'unique_stl_files':len(rows),'total_physical_pieces_including_optional':sum(r['quantity'] for r in rows)},indent=2))
 with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
  wr=csv.writer(f);wr.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
  for r in rows:wr.writerow([r['id'],r['label'],r['quantity'],r['file'],*r['reimport_check']['dimensions_mm'],r['support'],r['note']])
 for name in ['ASSEMBLY_R15.md','MEASUREMENT_REVIEW.md','geometry_checks.json','validation.json','hood_assembled.png']:
  shutil.copy2(OUT/name,target/('START_HERE.md' if name=='ASSEMBLY_R15.md' else name))
# Preserve previous complete references for unchanged lower/chassis parts.
for name in ['Split_shell_R07_reference.md','Electronics_R06_reference.md','Geekworm_mount_R05_reference.md','R07_JOINTS.csv','ToF_R09_reference.md']:
 shutil.copy2(OLDPACK/name,PACK/name)
shutil.copytree(OLDPACK/'historical_R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
with (PACK/'HARDWARE.csv').open('w',newline='') as f:
 wr=csv.writer(f);wr.writerow(['Scope','Instruction']);wr.writerow(['Hood','Reuse R14 H1-H6 M3x12 and H7-H8 M3x10, nuts and washers; all retained.'])
 wr.writerow(['LiDAR','Reuse four actual 44.78 mm brass posts; no printed P11 spacers. Base threads/fasteners are unmeasured; do not use old provisional screw lengths as approval.'])
 wr.writerow(['Camera','Replace two upper compliant pads with nominal 1.20 mm material; verify actual compression. Rigid clamps unchanged.'])
 wr.writerow(['Other components','See R14_HARDWARE_reference.csv and earlier component guides for unchanged items only.'])
shutil.copy2(OLDPACK/'HARDWARE.csv',PACK/'R14_HARDWARE_reference.csv')

# Exploded native CAD inspection, not a print-layout claim.
keep={CN,FN,RN}|{o.name for o in S.objects if o.name.startswith('Smile |')}|set(report['added_reference_objects'])|set(n for n in S.objects.keys() if n.startswith(('Housing_','Front_','RGB_','IR_Projector_','Depth_','Rear_Black_','Right_Small_','Left_Small_')))
for o in S.objects:
 if o.type not in ['LIGHT','CAMERA'] and o.name not in keep:o.hide_render=True;o.hide_set(True)
cap.location.x+=.55;front.location.y-=.50;rear.location.y+=.50
for o in S.objects:
 if o.name.startswith('Smile |'):o.location.y-=.50
S['Exploded view']='P15B +55 mm X; P15C -50 mm Y; P15D +50 mm Y. Sensors fixed.'
aim((380,-510,360),(15,0,174),370)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_sensor_enclosure_r15_exploded.blend'),compress=True);render('hood_exploded.png')
assert HASH(OLD/'roam_panel_bolts_r14.blend')==report['source_R14_sha256']
for target in [PACK,CHANGED]:
 if (OUT/'hood_exploded.png').exists():shutil.copy2(OUT/'hood_exploded.png',target/'hood_exploded.png')
 with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
  for p in sorted(target.rglob('*')):
   if p.is_file():z.write(p,p.relative_to(OUT))
 with zipfile.ZipFile(OUT/(target.name+'.zip')) as z:
  assert z.testzip() is None
  rows=json.loads((target/'manifest.json').read_text())['parts'];assert len([n for n in z.namelist() if n.endswith('.stl')])==len(rows)
  for r in rows:assert hashlib.sha256(z.read(target.name+'/'+r['file'])).hexdigest()==r['sha256']
print('R15_EXPORT_COMPLETE',json.dumps({'STLs':len(records),'new_STLs':len(changed),'retained_STLs':len(records)-len(changed)}),flush=True)
