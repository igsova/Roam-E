"""Independent R15 model, joint-profile, rotating-envelope and assembly QA.
--quick omits motion sweeps. Exporter separately checks STLs and ZIP contents.
No physical fit, material strength, optics calibration, or slicing is inferred.
"""
import ast,hashlib,json,math,sys,zipfile
from pathlib import Path
import bpy,bmesh
from mathutils import Vector,Matrix
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/sensor_enclosure_r15';OLD=ROOT/'mechanical/panel_bolts_r14'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for file,names in [('create_split_shell_r07.py',{'snapshot','metrics','bounds','remove','clone','clean','boolean','intersection','box','cyl'}),('validate_panel_bolts_r14.py',{'mesh_arrays','combine','bvh'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<QA helper>','exec'))
report=json.loads((OUT/'geometry_checks.json').read_text());d=report['datums']
master_hash=HASH(OUT/'roam_sensor_enclosure_r15.blend');geometry_hash=HASH(OUT/'geometry_checks.json')
assert HASH(OLD/'roam_panel_bolts_r14.blend')==report['source_R14_sha256']
bpy.ops.wm.open_mainfile(filepath=str(OLD/'roam_panel_bolts_r14.blend'));bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in bpy.context.scene.objects}
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_sensor_enclosure_r15.blend'));bpy.context.view_layer.update();S=bpy.context.scene
CN='P15B | Raised removable hood side panel R15';FN='P15C | Enclosed camera front hood R15';RN='P15D | Raised rear hood R15'
changed=set(report['changed_original_objects'])|{'Presentation camera'};unexpected=[n for n,h in original.items() if n not in changed and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
expected=(set(original)-set(report['renamed_hood_objects']))|set(report['renamed_hood_objects'].values())|set(report['added_reference_objects'])
assert set(S.objects.keys())==expected,(set(S.objects.keys())^expected)
assert all(n in S.objects for n in report['added_reference_objects'])
assert not S.objects['Component | LIDAR'].visible_get() and S.objects['Component | LIDAR'].hide_render
meshes={n:metrics(S.objects[n]) for n in [CN,FN,RN]}
for n,m in meshes.items():assert m['components']==1 and not m['nonmanifold_edges'] and not m['loose_vertices'] and m['volume_mm3']>0,(n,m)
camera_names=[o.name for o in S.objects if o.type=='MESH' and (o.name.startswith(('RGB_','IR_Projector_','Depth_','Front_')) or o.name in ['Housing_Silver_Capsule','Rear_Black_Inset','Right_Small_Front_Aperture','Left_Small_Front_Aperture'])]
cb=[bounds(S.objects[n]) for n in camera_names]
camera_bounds=[(min(b[i][0] for b in cb),max(b[i][1] for b in cb)) for i in range(3)]
camera_dims=[hi-lo for lo,hi in camera_bounds]
assert all(abs(a-b)<.01 for a,b in zip(camera_dims,[161,24.92,22.8])),camera_dims
for n in report['added_reference_objects']:
 if 'Brass post' in n:assert abs(metrics(S.objects[n])['dimensions_mm'][2]-44.78)<.005
for n in [o.name for o in S.objects if o.name.startswith('P08 | Camera upper compliant pad')]:
 assert abs(metrics(S.objects[n])['dimensions_mm'][2]-1.2)<.005
roof=[]
for n in [FN,RN]:
 o=S.objects[n];o.data.calc_loop_triangles()
 flat=sum(t.area*10000 for t in o.data.loop_triangles if t.normal.z>.999 and all(abs(o.data.vertices[i].co.z*100-d['roof_z_mm'])<.005 for i in t.vertices))
 assert flat>9000,(n,flat);roof.append({'part':n,'flat_exterior_roof_area_mm2':flat})
# All eight nut seats are measured from actual faces after the height edit.
ribs=[]
for j in report['nut_retention']:
 o=S.objects[j['part']];mat=Matrix(j['local_to_world_matrix']);axis=mat.to_3x3()@Vector((0,0,1));samples=[]
 for depth in [.2,1.5,2.4,2.75]:
  r=j['rib_tip_AF_mm']/2
  if depth>2:r=min(2.95,r+(2.96-r)*(depth-2)/.8)
  for k in range(6):
   a=math.tau*k/6;start=mat@(Vector((0,0,depth))/100);direction=mat.to_3x3()@Vector((math.cos(a),math.sin(a),0))
   hit,p,_,_=o.ray_cast(start,direction);assert hit
   actual=(p-start).length*100;assert abs(actual-r)<.006,(j['id'],depth,k,actual,r);samples.append(actual)
 start=mat@(Vector((2.1,0,.5))/100);hit,p,_,_=o.ray_cast(start,-axis);assert hit
 assert abs((mat.inverted()@p).z*100)<.006
 ribs.append({'joint':j['id'],'sample_radii_mm':samples})
groups={FN:[S.objects[FN]],RN:[S.objects[RN]],CN:[S.objects[CN]]};removed=[]
for o in S.objects:
 n=o.name
 if n.startswith('Smile |'):groups[FN].append(o)
 if n.startswith(('P12 |','P14 |')):
  bb=bounds(o);key=CN if n.startswith('P12 |') and sum(bb[0])>0 else FN if sum(bb[1])/2<10 else RN;groups[key].append(o)
 if n in ['H1 | M3 captive nut','H2 | M3 captive nut','H3 | M3 captive nut'] or (n.startswith('R14 H') and 'nut' in n):groups[CN].append(o)
 if n=='R10 H4 | M3 captive nut' or (n.startswith('R12 H') and 'nut' in n):groups[RN].append(o)
 if (n.startswith(('K','H1 |','H2 |','H3 |','R10 H4 |','R12 H','R14 H')) and ('screw' in n or 'washer' in n)):removed.append(n)
groupnames={o.name for obs in groups.values() for o in obs}
env=[o for o in S.objects if o.type=='MESH' and not o.hide_render and o.name not in groupnames and o.name not in removed and bounds(o)[2][1]>101.99 and not any(c.name=='06 Studio' for c in o.users_collection)]
holds=['K-1_55 | M3x16 screw','K-1_55 | washer','K-1_-55 | M3x16 screw','K-1_-55 | washer']
arrays={o.name:mesh_arrays(o,inset_contact=o.name.startswith('P12 |')) for o in env+[o for obs in groups.values() for o in obs]+[S.objects[n] for n in removed]}
base=[o.name for o in env];sv,sf,owners=combine(base);fixed=bvh(sv,sf)
static_collisions={}
for n in [CN,FN,RN]:
 pairs=bvh(*arrays[n]).overlap(fixed)
 static_collisions[n]=sorted(set(owners[j] for _,j in pairs))
print('R15_FIXED_COLLISIONS',json.dumps(static_collisions),flush=True)
fastener_collisions={}
for n in removed:
 if n.startswith('K'):continue
 pairs=bvh(*arrays[n]).overlap(fixed)
 if pairs:fastener_collisions[n]=sorted(set(owners[j] for _,j in pairs))
print('R15_FASTENER_COLLISIONS',json.dumps(fastener_collisions),flush=True)
# A full rotating cylinder is authoritative for mechanical rotor clearance.
rc=d['aperture_centre_mm'];r=report['measurements']['lidar_rotor_diameter_mm']/2
swept=cyl('_QA rotating cylinder',(rc[0],rc[1],d['rotor_bottom_z_mm']+10),r,20,192)
rotor_intersections={n:intersection(S.objects[n],swept) for n in [CN,FN,RN]};remove(swept)
assert max(rotor_intersections.values())<.03,rotor_intersections
# Sample aperture at the roof height, including the locally rerouted seam.
radial=[]
for deg in range(360):
 a=math.radians(deg);direc=Vector((math.cos(a),math.sin(a),0));start=Vector((rc[0],rc[1],d['roof_z_mm']-1.4))/100
 hits=[]
 for n in [FN,RN]:
  hit,p,_,_=S.objects[n].ray_cast(start,direc)
  if hit:hits.append((p-start).length*100)
 if hits:
  actual=min(hits);assert actual>=r+1.97,(deg,actual);radial.append(actual-r)
motion=[];collisions=[]
stages=[(CN,Vector((1,0,0)),[FN,RN],holds),(FN,Vector((0,-1,0)),[RN],holds[:2]),(RN,Vector((0,1,0)),[],[])]
if '--quick' not in sys.argv:
 for name,axis,remaining,retained in stages:
  names=base+[o.name for k in remaining for o in groups[k]]+retained
  vs,fs,owners=combine(names);st=bvh(vs,fs);mv,mf,mo=combine([o.name for o in groups[name]]);hits=[]
  for step in range(501):
   dist=step*.5;mt=bvh([p+axis*(dist/100) for p in mv],mf);pairs=mt.overlap(st)
   if pairs:
    by={}
    for i,j in pairs:
     key=mo[i]+' <> '+owners[j];by[key]=by.get(key,0)+1
    hits.append({'offset_mm':dist,'pairs':by})
   if step%100==0:print('R15_PATH',name,dist,'hits',len(hits),flush=True)
  motion.append({'part':name,'direction':list(axis),'sample_step_mm':.5,'distance_mm':250,'sample_count':501,'collisions':hits});collisions+=hits
result={'revision':'R15','status':'PASS' if not collisions and not any(static_collisions.values()) and not fastener_collisions else 'FAIL','quick_only':'--quick' in sys.argv,
 'master_sha256':master_hash,'geometry_report_sha256':geometry_hash,
 'source_R14_sha256':report['source_R14_sha256'],'unchanged_original_objects':len(original)-len(changed),'unexpected_original_changes':unexpected,
 'mesh_checks':meshes,'camera_envelope_mm':camera_dims,'camera_bounds_mm':camera_bounds,'roof_checks':roof,'retained_nut_checks':ribs,'stationary_robot_collisions':static_collisions,'hood_fastener_to_fixed_robot_collisions':fastener_collisions,'rotating_envelope_overlap_mm3':rotor_intersections,'min_sampled_radial_clearance_mm':min(radial),'sampled_motion':motion,
 'limits':['Not physical fit or continuous collision proof. Motion sampled at 0.5 mm.','LiDAR underside shape, optical detail, screw heads and inherited mounting footprint are references, not fully measured CAD.','P12 contact sleeves are shrunk 0.2% in QA only to separate intentional coincident contact.','No wiring, vibration, thermal, material strength, sensor calibration, full optical FOV or sliced support validation.']}
(OUT/('validation_quick.json' if '--quick' in sys.argv else 'validation.json')).write_text(json.dumps(result,indent=2))
assert not any(static_collisions.values()),static_collisions
assert not fastener_collisions,fastener_collisions
assert not collisions,'Inspect validation.json for path collision details'
print('R15_VALIDATION_COMPLETE',result['status'],flush=True)
