"""R17 independent geometry, sensor, switch space and sampled motion tests."""
import ast,hashlib,json,math
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/lower_controls_r17'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','boolean','bounds','metrics','snapshot','intersection'}),('validate_panel_bolts_r14.py',{'mesh_arrays'}),('create_lower_controls_r17.py',{'local','cb','cy'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<QA>','exec'))
report=json.loads((OUT/'geometry_checks.json').read_text())
source=ROOT/'mechanical/camera_closure_r16/roam_camera_closure_r16.blend'
assert HASH(source)==report['source_R16_sha256']
bpy.ops.wm.open_mainfile(filepath=str(source));S=bpy.context.scene;bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in S.objects}
old_arrays={o.name:mesh_arrays(o) for o in S.objects if o.type=='MESH' and not o.hide_render and bounds(o)[2][0]<105 and not any(c.name=='06 Studio' for c in o.users_collection)}
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_lower_controls_r17.blend'));S=bpy.context.scene;bpy.context.view_layer.update()
FN='P16C | Front ToF skirt R17';RN='P16D | Rear ToF skirt R17';CN='P16B | Wheel-switch side panel R17';DN='P05 | Curved switched battery drawer R17'
PN=[FN,RN,CN,DN,'P06L | Curved fascia lock guide R17','P06R | Curved fascia lock guide R17']
parts=[S.objects[n] for n in PN]
front,rear,cap,drawer=parts[:4]
checks={'revision':'R17','status':'RUNNING','source_R16_sha256':HASH(source),'master_sha256':HASH(OUT/'roam_lower_controls_r17.blend'),'limits':['Prototype and sampled geometry checks only; actual print fit, PETG grip, strength, cables and slicer supports not validated.','Switch contact DC rating, snap-clip shape and mounting fit remain unverified. No wiring change made.']}
def save(): (OUT/'validation.json').write_text(json.dumps(checks,indent=2))
changed=[n for n,h in original.items() if n not in report['allowed_original_changes'] and n!='Presentation camera' and (n not in S.objects or snapshot(S.objects[n])!=h)]
checks['unexpected_original_changes']=changed;assert not changed,changed
checks['unchanged_object_count']=len(original)-len(report['allowed_original_changes'])-1
checks['presentation_camera_reframed']=True
checks['solids']={o.name:metrics(o) for o in parts}
assert all(m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 for m in checks['solids'].values())
inverse={v:k for k,v in report['renames'].items()};inverse[RN]='P16A | Main ToF skirt R07'
def old_obj(n):
 n=inverse.get(n,n)
 if n not in old_arrays:return None
 v,f=old_arrays[n];me=bpy.data.meshes.new('_old mesh');me.from_pydata(v,[],f);me.update();o=bpy.data.objects.new('_old '+n,me);S.collection.objects.link(o);return o
def baseline(a,b):
 aa=old_obj(a.name);bb=old_obj(b.name)
 if aa is None or bb is None:
  if aa:remove(aa)
  if bb:remove(bb)
  return 0
 v=contact_volume(aa,bb);remove(aa);remove(bb);return v

def contact_volume(a,b):
 # Small reference-side shrink removes coincident contact faces without
 # shrinking the entire shell inward by tenths of a millimetre. Record this
 # tolerance; real dimensional measurements are tested independently below.
 if not all(min(x[1],y[1])-max(x[0],y[0])>.01 for x,y in zip(bounds(a),bounds(b))):return 0.
 small=clone(b,'_QA contact inset');ps=[v.co for v in small.data.vertices]
 centre=Vector([(min(v[i] for v in ps)+max(v[i] for v in ps))/2 for i in range(3)])
 small.data.transform(Matrix.Translation(centre)@Matrix.Scale(.999,4)@Matrix.Translation(-centre))
 value=intersection(a,small);remove(small);return value

# Static interference volumes; retain reported legacy contacts separately,
# never silently permit a new overlap simply because a mesh is manifold.
env=[o for o in S.objects if o.type=='MESH' and not o.hide_render and o.name not in PN and not any(c.name=='06 Studio' for c in o.users_collection) and bounds(o)[2][0]<105]
collisions=[];contacts=[];pairs=set()
for a in parts:
 for b in parts+env:
  pair=tuple(sorted([a.name,b.name]))
  if a==b or pair in pairs:continue
  pairs.add(pair)
  v=contact_volume(a,b)
  if v<.03:continue
  intended=any(j['part']==a.name and j['nut_object']==b.name for j in report['nut_retention']) and v<3
  if a.name.startswith('P06') and 'drawer lock nut' in b.name and v<3:intended=True
  base=baseline(a,b) if not intended and b not in parts else 0
  row=[a.name,b.name,round(v,5),round(base,5)]
  if intended or v<=base+.04:contacts.append(row)
  else:collisions.append(row)
 print('R17_STATIC_PART',a.name,flush=True)
checks['new_static_collisions']=collisions;checks['intentional_rib_or_legacy_contacts']=contacts;save()
print('R17_STATIC',json.dumps(collisions),flush=True)

# Functional ToF checks: all eight measured sensor datums and 2mm reliefs.
stations=[]
for tag in ['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']:
 pcb=S.objects['ToF '+tag+' | Reference PCB 25x10.66 R09'];mat=pcb.matrix_world.copy();sensor=S.objects['ToF '+tag+' | VL53L0X optical package']
 vv=[mat.inverted()@(sensor.matrix_world@v.co)*100 for v in sensor.data.vertices];edge=min(v.y for v in vv)
 part=S.objects[CN if tag=='Right' else FN if tag.startswith('Front') else RN];inv=part.matrix_world.inverted();normal=mat.to_3x3()@Vector((0,0,1));depths=[];gaps=[]
 for k in range(6):
  pad=S.objects[f'ToF {tag} | Header pad {k}'];vs=[mat.inverted()@(pad.matrix_world@v.co)*100 for v in pad.data.vertices]
  u=(min(v.x for v in vs)+max(v.x for v in vs))/2;v=(min(v.y for v in vs)+max(v.y for v in vs))/2;gaps.append(edge-v)
  for dv in [-.85,0,.85]:
   start=mat@(Vector((u,v+dv,-.85))/100);hit,pt,_,_=part.ray_cast(inv@start,inv.to_3x3()@normal);assert hit
   depths.append((part.matrix_world@pt-start).length*100)
 assert all(abs(d-2)<.006 for d in depths),(tag,depths)
 assert all(abs(d-4.8)<.006 for d in gaps),(tag,gaps)
 stations.append({'station':tag,'relief_samples_mm':depths,'sensor_edge_to_pin_centres_mm':gaps})
checks['ToF_measurements']=stations
rib_checks=[]
for joint in report['nut_retention']:
 ob=S.objects[joint['part']];mat=Matrix(joint['matrix']);readings=[]
 for depth in [.2,1.5,2.4]:
  expected=2.65 if depth<=2 else 2.65+(2.96-2.65)*(depth-2)/.8
  for k in range(6):
   a=k*math.tau/6;origin=mat@(Vector((0,0,depth))/100);direction=mat.to_3x3()@Vector((math.cos(a),math.sin(a),0))
   hit,p,_,_=ob.ray_cast(origin,direction);assert hit
   actual=(p-origin).length*100;assert abs(actual-expected)<.008,(joint['id'],actual,expected)
   readings.append(actual)
 rib_checks.append({'joint':joint['id'],'radius_samples_mm':readings})
checks['nut_retention_surface_checks']=rib_checks
guide_slots=[]
for part,sign in [(parts[4],-1),(parts[5],1)]:
 origin=Vector((80*sign,report['lock_seat_y_mm']-8.4,78))/100
 widths=[]
 for direction in [Vector((1,0,0)),Vector((-1,0,0))]:
  hit,p,_,_=part.ray_cast(origin,direction);assert hit;d=(p-origin).length*100;assert abs(d-2.65)<.008,(part.name,d);widths.append(d)
 hit,p,_,_=part.ray_cast(origin,Vector((0,0,-1)));assert hit;floor_z=p.z*100;assert abs(floor_z-74.825)<.01,(part.name,floor_z)
 guide_slots.append({'part':part.name,'nut_grip_gap_mm':sum(widths),'nut_stop_z_mm':floor_z})
checks['guide_nut_slot_measurements']=guide_slots
floor=[]
for x in [-12,12]:
 hit,pt,_,_=drawer.ray_cast(Vector((x,35,55))/100,Vector((0,0,1)));assert hit
 roof=pt.z*100;hit,pt,_,_=drawer.ray_cast(Vector((x,35,63))/100,Vector((0,0,-1)));assert hit
 top=pt.z*100;assert abs(roof-59.9)<.01 and abs(top-61.5)<.01,(roof,top)
 floor.append({'channel_x_mm':x,'roof_z_mm':roof,'tray_top_z_mm':top,'remaining_floor_mm':top-roof,'caster_head_top_z_mm':59.407,'clearance_mm':roof-59.407})
checks['caster_channel_floor']=floor

# Confirm seat thickness and bore on ACTUAL solid; keep-out reserves 40mm
# behind mounting plane, including insulation/wire exit allowance.
space=[]
for m in report['switch_mounts']:
 part=S.objects[m['part']];mat=Matrix(m['matrix']);normal=mat.to_3x3()@Vector((0,0,1));measured=[]
 for i in range(16):
  a=i*math.tau/16;p=mat@(Vector((10.8*math.cos(a),10.8*math.sin(a),-2))/100)
  hit,q,_,_=part.ray_cast(p,normal);assert hit
  seat_front=(mat.inverted()@q).z*100
  p=mat@(Vector((10.8*math.cos(a),10.8*math.sin(a),5))/100);hit,q,_,_=part.ray_cast(p,-normal);assert hit
  seat_back=(mat.inverted()@q).z*100;measured.append([seat_front,seat_back]);assert abs(seat_front)<.015 and abs(seat_back-2.2)<.015,(m['id'],seat_front,seat_back)
 reserve=cy(mat,(0,0,10.5),11.8,16.0)
 boolean(reserve,cb(mat,(0,0,27),(16,16,26)),'UNION')
 hits=[]
 for other in parts+env:
  if other==part or other.name.startswith('R17 '+m['id']+' rocker'):continue
  v=intersection(reserve,other)
  if v>.03:hits.append([other.name,v])
 remove(reserve);space.append({'switch':m['id'],'seat_surfaces_mm':measured,'reserved_depth_mm':40,'reserved_volume_collisions':hits})
checks['switch_space']=space;save()

# Sample full withdrawal with the real drawer payload and both electrical
# terminals modeled; wires remain an explicit physical verification item.
arrays={o.name:mesh_arrays(o) for o in parts+env}
def combined(names):
 vs=[];fs=[]
 for name in names:
  v,f=arrays[name];off=len(vs);vs+=v;fs += [tuple(i+off for i in tri) for tri in f]
 return vs,fs
def tree(vs,fs):return BVHTree.FromPolygons(vs,fs,all_triangles=True,epsilon=0)
drawer_group=[DN]+[o.name for o in env if o.name.startswith(('Component | Battery','Battery |','R17 BATTERY rocker'))]
removed={o.name for o in env if o.name.startswith(('E-1 |','E1 |')) and 'drawer lock nut' not in o.name}
stationary=[o.name for o in parts+env if o.name not in drawer_group and o.name not in removed]
mv,mf=combined(drawer_group);sv,sf=combined(stationary);st=tree(sv,sf);hits=[]
for i in range(301):
 offset=Vector((0,i*.5/100,0));bvh=tree([p+offset for p in mv],mf)
 if bvh.overlap(st):hits.append(i*.5)
checks['drawer_withdrawal']={'range_mm':[0,150],'step_mm':.5,'sample_count':301,'surface_intersection_offsets_mm':hits,'payload':drawer_group,'fasteners_removed':sorted(removed)}
save();print('R17_DRAWER_MOTION',hits[:12],flush=True)

# Lower shell can be assembled on the bench. Measure seam separation without
# claiming the inherited mounting feet have a straight path past all pylons.
separations=[]
for moving,fixed,axis in [(cap,[front,rear],Vector((1,0,0))),(front,[rear],Vector((0,-1,0)))]:
 mv,mf=arrays[moving.name];fv,ff=combined([o.name for o in fixed]);fixedtree=tree(fv,ff);h=[]
 for i in range(401):
  offset=axis*(i*.5/100)
  if tree([v+offset for v in mv],mf).overlap(fixedtree):h.append(i*.5)
 separations.append({'moving':moving.name,'fixed':[o.name for o in fixed],'axis':list(axis),'step_mm':.5,'distance_mm':200,'surface_intersection_offsets_mm':h})
checks['shell_joint_separation']=separations
checks['status']='PASS' if not collisions and not any(s['reserved_volume_collisions'] for s in space) and not hits and not any(s['surface_intersection_offsets_mm'] for s in separations) else 'FAIL'
save();print('R17_VALIDATION_COMPLETE',checks['status'],flush=True)
