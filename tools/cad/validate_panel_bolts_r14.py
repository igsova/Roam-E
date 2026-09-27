"""Independent R14 identity, rib dimensions, mesh and assembly-path QA.

Collision sampling is 0.5 mm over each 240 mm linear removal path. This is a
digital sampled test, not continuous collision proof or physical validation.
Each moving section's K/H screws and washers are removed; the left K screws
can hold the still-stationary sections. Captive nuts, plugs and P12 sleeves
stay with their respective parts. The camera/frame/LiDAR stay put.
"""
import ast,hashlib,json,shutil,zipfile,math,sys
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/nut_retention_r13'
OUT=ROOT/'mechanical/panel_bolts_r14'
PACK=OUT/'ROAM_STL_R14_Panel_Bolts_K1C'
CHANGED=OUT/'ROAM_R14_hood_replacement'
OLDPACK=OLD/'ROAM_STL_R13_Nut_Retention_K1C'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for n in ast.parse((ROOT/'tools/cad/create_split_shell_r07.py').read_text()).body:
 if isinstance(n,ast.FunctionDef) and n.name in {'snapshot','metrics','remove','bounds','clone','clean','boolean','intersection','box'}:
  exec(compile(ast.Module(body=[n],type_ignores=[]),'<QA>','exec'))
report=json.loads((OUT/'geometry_checks.json').read_text())
assert HASH(OLD/'roam_nut_retention_r13.blend')==report['source_R13_sha256']
assert not report['unexpected_original_changes']
bpy.ops.wm.open_mainfile(filepath=str(OLD/'roam_nut_retention_r13.blend'))
bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in bpy.context.scene.objects}
def mesh_arrays(ob,inset_contact=False):
 me=bpy.data.meshes.new_from_object(ob.evaluated_get(bpy.context.evaluated_depsgraph_get()))
 me.transform(ob.matrix_world);me.calc_loop_triangles()
 vs=[v.co.copy() for v in me.vertices];fs=[tuple(t.vertices) for t in me.loop_triangles]
 if inset_contact:
  c=Vector([(min(p[i] for p in vs)+max(p[i] for p in vs))/2 for i in range(3)])
  vs=[c+(p-c)*.998 for p in vs]
 bpy.data.meshes.remove(me);return vs,fs
old_geometry=[mesh_arrays(bpy.context.scene.objects[n]) for n in report['renamed_hood_objects']]
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_panel_bolts_r14.blend'))
bpy.context.view_layer.update();S=bpy.context.scene
OLDNAMES=set(report['renamed_hood_objects'])
FN='P15C | Front slide-on smiling hood R14';RN='P15D | Rear slide-on hood R14';CN='P15B | Removable hood side panel R14'
NEW_HW=[f'R12 {j} | {item}' for j in ['H5','H6'] for item in ['M3x12 screw','M3 captive nut','M3 washer']]
ADDED_HW=[f'R14 {j} | {item}' for j in ['H7','H8'] for item in ['M3x10 screw','M3 captive nut','M3 washer']]
NEW_HW+=ADDED_HW
assert set(S.objects.keys())==(set(original)-OLDNAMES)|{CN,FN,RN}|set(ADDED_HW)
changed=[n for n,h in original.items() if n not in OLDNAMES and snapshot(S.objects[n])!=h]
assert not changed,changed
# Independently measure the delivered pocket surface, not just parameters.
rib_checks=[]
for joint in report['nut_retention']:
 ob=S.objects[joint['part']];mat=Matrix(joint['local_to_world_matrix']);axis=mat.to_3x3()@Vector((0,0,1))
 readings=[]
 for depth in [.2,1.5,2.4,2.75]:
  radius=joint['rib_tip_AF_mm']/2
  if depth>2:radius=min(2.95,radius+(2.96-radius)*(depth-2)/.8)
  for k in range(6):
   a=k*math.tau/6;origin=mat@(Vector((0,0,depth))/100)
   direction=mat.to_3x3()@Vector((math.cos(a),math.sin(a),0))
   hit,p,_,_=ob.ray_cast(origin,direction);assert hit
   actual=(p-origin).length*100
   assert abs(actual-radius)<.005,(joint['id'],depth,k,actual,radius)
   readings.append({'depth_mm':depth,'flat':k,'radius_mm':actual})
 # The unchanged nut-bearing floor must still lie at local depth zero.
 origin=mat@(Vector((2.1,0,.5))/100)
 hit,p,_,_=ob.ray_cast(origin,-axis);assert hit
 floor_depth=(mat.inverted()@p).z*100
 assert abs(floor_depth)<.005,(joint['id'],floor_depth)
 # Full-size modeled metal nut deliberately overlaps the compressible ribs.
 overlap=intersection(ob,S.objects[joint['nut_object']])
 assert .05<overlap<3.,(joint['id'],overlap)
 rib_checks.append({'joint':joint['id'],'surface_samples':readings,
  'nut_floor_depth_mm':floor_depth,'intentional_nominal_nut_interference_volume_mm3':overlap})

groups={FN:[S.objects[FN]],RN:[S.objects[RN],S.objects['R10 H4 | M3 captive nut']],CN:[S.objects[CN]]}
groups[RN]+=[S.objects[n] for n in NEW_HW if 'nut' in n and n.startswith('R12')]
groups[CN]+=[S.objects[n] for n in ADDED_HW if 'nut' in n]
removed=[n for n in NEW_HW if 'nut' not in n]
for ob in S.objects:
 if ob.name.startswith('Smile |'):groups[FN].append(ob)
 if ob.name.startswith(('P12 |','P14 |')):
  bb=bounds(ob)
  name=CN if ob.name.startswith('P12 |') and sum(bb[0])>0 else (FN if sum(bb[1])/2<10 else RN)
  groups[name].append(ob)
 if ob.name in ['H1 | M3 captive nut','H2 | M3 captive nut','H3 | M3 captive nut']:groups[CN].append(ob)
 if ((ob.name.startswith('K') and ('screw' in ob.name or 'washer' in ob.name)) or
     ob.name in ['H1 | M3x12 screw','H2 | M3x12 screw','H3 | M3x12 screw','H1 | M3 washer','H2 | M3 washer','H3 | M3 washer','R10 H4 | M3x12 screw','R10 H4 | M3 washer']):removed.append(ob.name)
groupnames={ob.name for obs in groups.values() for ob in obs}
env=[o for o in S.objects if o.type=='MESH' and not o.hide_render and o.name not in groupnames and o.name not in removed
     and bounds(o)[2][1]>101.99 and not any(c.name=='06 Studio' for c in o.users_collection)]
rear_hold=['K-1_55 | M3x16 screw','K-1_55 | washer']
front_hold=['K-1_-55 | M3x16 screw','K-1_-55 | washer']
arrays={}
for ob in [S.objects[n] for n in NEW_HW]+env+[o for g in groups.values() for o in g]+[S.objects[n] for n in rear_hold+front_hold]+[S.objects['R10 H4 | M3x12 screw'],S.objects['R10 H4 | M3 washer']]:
 # Only separate intentional sleeve/frame face contact in QA. Delivered sleeve
 # geometry is unchanged. Other purchased hardware remains at full dimensions.
 arrays[ob.name]=mesh_arrays(ob,inset_contact=ob.name.startswith('P12 |'))
def combine(names):
 vs=[];fs=[];owners=[]
 for name in names:
  v,f=arrays[name];base=len(vs);vs+=v;fs.extend(tuple(i+base for i in face) for face in f);owners.extend([name]*len(f))
 return vs,fs,owners
def bvh(vs,fs):return BVHTree.FromPolygons(vs,fs,all_triangles=True,epsilon=0.)
base_env=[o.name for o in env]
motion=[];collisions=[]
stages=[(CN,Vector((1,0,0)),[FN,RN],rear_hold+front_hold),
        (FN,Vector((0,-1,0)),[RN],rear_hold),(RN,Vector((0,1,0)),[],[])]
for name,direction,remaining,retained in stages:
 static_names=base_env+[o.name for k in remaining for o in groups[k]]+retained
 sv,sf,owners=combine(static_names);st=bvh(sv,sf)
 mv,mf,mo=combine([o.name for o in groups[name]])
 hits=[]
 for step in range(0 if '--local-checks-only' in sys.argv else 481):
  offset=step*.5
  mt=bvh([p+direction*(offset/100) for p in mv],mf)
  pairs=mt.overlap(st)
  if pairs:
   by={}
   for i,j in pairs:
    key=mo[i]+' <> '+owners[j];by[key]=by.get(key,0)+1
   hits.append({'offset_mm':offset,'pairs':by})
  if step%80==0:print('R14_PATH',name,offset,'hits',len(hits),flush=True)
 # At the far end the entire group must be beyond the robot along its axis.
 axis=max(range(3),key=lambda i:abs(direction[i]));sign=direction[axis]
 moving_extrema=[(p[axis]+direction[axis]*2.4)*sign*100 for p in mv]
 fixed_extrema=[p[axis]*sign*100 for p in sv]
 exit_gap=min(moving_extrema)-max(fixed_extrema)
 assert exit_gap>10,(name,exit_gap)
 motion.append({'part':name,'direction':list(direction),'offset_range_mm':[0,240],
   'step_mm':.5,'sample_count':481,'moving_objects':[o.name for o in groups[name]],
   'stationary_objects':len(static_names),'retained_stationary_cover_hardware':retained,
   'full_exit_gap_mm':exit_gap,'collisions':hits})
 collisions+=hits

# Only the two end-joint regions may change; protect roof, camera, old joints.
newtrees=[bvh(*arrays[n]) for n in [CN,FN,RN]];distances=[]
def in_edited_region(p):
 xyz=p*100
 return any(all(lo-.05<=xyz[i]<=hi+.05 for i,(lo,hi) in enumerate(j['edit_bounds_mm'])) for j in report['panel_joints'])
for verts,_ in old_geometry:
 for p in verts:
  xyz=p*100
  if in_edited_region(p):continue
  distances.append(min(t.find_nearest(p)[3]*100 for t in newtrees))
assert max(distances)<.01,max(distances)
# Check in the other direction too: all new surface outside the cavities must
# lie on its original part. No pad, seam, camera opening or outer bulge added.
outside_errors=[];outside_locations=[];volume_checks={}
for name,(vs,fs) in zip([CN,FN,RN],old_geometry):
 oldtree=bvh(vs,fs)
 nvs,nfs=arrays[name];newtree=bvh(nvs,nfs)
 def surface_samples(vertices,faces):
  yield from vertices
  for face in faces:
   a,b,c=(vertices[i] for i in face)
   yield (a+b+c)/3
   yield a*.6+b*.2+c*.2
   yield a*.2+b*.6+c*.2
   yield a*.2+b*.2+c*.6
 for p in surface_samples(nvs,nfs):
  if in_edited_region(p):continue
  error=oldtree.find_nearest(p)[3]*100;outside_errors.append(error)
  if error>.01:outside_locations.append((error,name,list(p*100)))
 # Almost identical meshes make a whole-body Boolean difference ill-defined
 # at their coincident faces. Instead check signed nearest-surface containment
 # at original vertices and four barycentric samples per original triangle.
 # Positive signed distance is removed original material; allow 0.01 mm for
 # the 0.005 mm mesh cleanup, and report the measured error, not zero loss.
 containment=[]
 for p in surface_samples(vs,fs):
  if in_edited_region(p):continue
  q,normal,_,_=newtree.find_nearest(p)
  containment.append((p-q).dot(normal)*100)
 assert max(containment)<.01,(name,max(containment))
 me=bpy.data.meshes.new('_QA original');me.from_pydata(vs,[],fs);me.update()
 old=bpy.data.objects.new('_QA original',me);S.collection.objects.link(old)
 ov=metrics(old)['volume_mm3'];nv=metrics(S.objects[name])['volume_mm3']
 delta=nv-ov;assert delta>100,(name,delta)
 volume_checks[name]={'net_added_volume_mm3':delta,'original_surface_containment_samples':len(containment),
  'max_original_surface_outside_new_solid_mm':max(0,max(containment)),
  'surface_tolerance_mm':.01}
 remove(old)
# Re-tessellation of the original non-planar n-gons introduces sub-0.05 mm
# outward surface differences. Keep the stricter 0.01 mm original-material
# containment and vertex limits above; report both, rather than claiming exact
# equality of the two edited meshes. Other objects/STLs remain hash-identical.
assert max(outside_errors)<.05,sorted(outside_locations,reverse=True)[:12]
if '--local-checks-only' in sys.argv:
 print('R14_LOCAL_CHECKS_ONLY',json.dumps(volume_checks),flush=True)
 raise SystemExit(0)

# Full-size vertical screwdriver envelope above H4: 6 mm diameter from the
# head top to Z=255 mm. No stored shell or frame part may block tool access.
seat=report['roof_joint']['seat_mm'];z0=seat[2]+3.6
vs=[];n=32
for z in [z0,255.]:
 vs.extend(Vector((seat[0]+3*math.cos(math.tau*i/n),seat[1]+3*math.sin(math.tau*i/n),z))/100 for i in range(n))
faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
tool=BVHTree.FromPolygons(vs,faces,all_triangles=False)
sv,sf,owners=combine(base_env+list(groupnames));access=sorted(set(owners[j] for _,j in tool.overlap(bvh(sv,sf))))
assert not access,access
sv,sf,owners=combine(base_env);installed_hardware_clashes={}
for name in NEW_HW+['R10 H4 | M3x12 screw','R10 H4 | M3 washer','R10 H4 | M3 captive nut']:
 pairs=bvh(*arrays[name]).overlap(bvh(sv,sf))
 installed_hardware_clashes[name]=sorted(set(owners[j] for _,j in pairs))
assert not any(installed_hardware_clashes.values()),installed_hardware_clashes

# Outside driver access for each side bolt: straight 6 mm-diameter shaft.
side_access={}
for joint in report['side_joints']:
 x,y,z=joint['seat_mm'];vs=[];n=32
 for xx in [x-70,x-3.6]:
  vs.extend(Vector((xx,y+3*math.cos(math.tau*i/n),z+3*math.sin(math.tau*i/n)))/100 for i in range(n))
 faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 tool=BVHTree.FromPolygons(vs,faces,all_triangles=False)
 sv,sf,owners=combine(base_env+list(groupnames))
 side_access[joint['id']]=sorted(set(owners[j] for _,j in tool.overlap(bvh(sv,sf))))
assert not any(side_access.values()),side_access

# H7/H8 screwdriver access from the front/rear exterior, along each bolt axis.
panel_access={};hardware_to_covers={}
for joint in report['panel_joints']:
 mat=Matrix(joint['local_to_world_matrix']);vs=[];n=32
 for w in [-70,-3.6]:
  vs.extend(mat@(Vector((-6+3*math.cos(math.tau*i/n),3*math.sin(math.tau*i/n),w))/100) for i in range(n))
 faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 tool=BVHTree.FromPolygons(vs,faces,all_triangles=False)
 sv,sf,owners=combine(base_env+list(groupnames))
 panel_access[joint['id']]=sorted(set(owners[j] for _,j in tool.overlap(bvh(sv,sf))))
assert not any(panel_access.values()),panel_access
for name in ADDED_HW:
 for cover in [CN,FN,RN]:
  if 'nut' in name and cover==CN:continue # intentional R13 press-fit rib contact
  overlap=intersection(S.objects[name],S.objects[cover],inset=True)
  hardware_to_covers[name+' / '+cover]=overlap
  assert overlap<.03,(name,cover,overlap)

result={'revision':'R14','status':'pending' if collisions else 'digital mesh and sampled assembly-path checks passed',
 'panel_joint_driver_access_6mm_diameter':panel_access,'new_hardware_to_covers_mm3':hardware_to_covers,
 'measured_nut_retention':rib_checks,'material_changes':volume_checks,
 'max_new_surface_error_outside_pockets_mm':max(outside_errors),
 'new_surface_tessellation_tolerance_mm':.05,
 'retention_type':'PETG interference ribs. Retaining force, repeated insertion life and physical print fit untested.',
 'unchanged_R13_objects':len(original)-3,'unexpected_original_changes':changed,
 'removed_for_assembly_screws_and_washers':removed,'sampled_removal_paths':motion,
 'assembly_order':'Preload H4-H6 nuts in P15D; H1-H3 and H7-H8 nuts in P15B. Fit rear P15D, front P15C, then P15B from right. Fit all bolts; remove H7/H8 before withdrawing P15B.',
 'protected_vertex_samples_outside_end_joints':len(distances),'max_protected_vertex_error_mm':max(distances),
 'H4_vertical_tool_access_6mm_diameter':'clear','side_driver_access_6mm_diameter':side_access,'new_hardware_against_fixed_robot':installed_hardware_clashes,
 'contact_QA_tolerance':'P12 sleeve QA copies shrink 0.2% only to separate intentional sleeve/frame face contact; all delivered geometry unchanged.',
 'limits':['Sampled 0.5 mm mesh motion, not continuous collision detection or a physical-fit guarantee.',
  'Unmodeled wires, connector bend radii, manufacturing tolerances and flexible printed parts are not simulated.',
  'Fit Q14 joint pairs and Q13 PETG nut-retention coupons first. Deliberate rib/nut interference is not a measured retention force or lifetime guarantee.',
  'Dry-fit covers with power disconnected. Never force a cover against optics.',
  'Nominal bounding-box bed fit is not sliced toolpath/support validation. Support quantity and print time unmeasured.']}
(OUT/'validation.json').write_text(json.dumps(result,indent=2))
assert not collisions,'Assembly collisions: inspect validation.json'
manifest=json.loads((PACK/'manifest.json').read_text())
prior={r['id']:r for r in json.loads((OLDPACK/'manifest.json').read_text())['parts']}
same=[];new={}
for row in manifest['parts']:
 p=PACK/row['file'];assert HASH(p)==row['sha256']
 if row['id'] not in {'P15B','P15C','P15D'} and not row['id'].startswith('Q14'):
  assert row['sha256']==prior[row['id']]['sha256'];same.append(row['id'])
 else:
  bpy.ops.wm.stl_import(filepath=str(p),use_facet_normal=False)
  ob=bpy.context.object;m=metrics(ob,1)
  ob.data.calc_loop_triangles()
  flat=sum(t.area for t in ob.data.loop_triangles if t.normal.z<-.99 and all(abs(ob.data.vertices[i].co.z)<.001 for i in t.vertices))
  m['flat_bed_contact_area_mm2']=flat
  if row['id'] in {'P15C','P15D'}:assert flat>9000,(row['id'],flat)
  remove(ob)
  assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 and m['volume_mm3']>0 and abs(m['min_z_mm'])<.001,m
  assert all(d+8<220 for d in m['dimensions_mm'][:2]) and m['dimensions_mm'][2]<250
  assert all(abs(a-b)<.002 for a,b in zip(m['dimensions_mm'],row['reimport_check']['dimensions_mm']))
  new[row['id']]=m
assert len(same)==53 and len(new)==7 and len(manifest['parts'])==60
assert len({r['id'] for r in manifest['parts']})==len(manifest['parts'])
assert len(list(PACK.rglob('*.stl')))==len(manifest['parts'])
assert HASH(OLD/'roam_nut_retention_r13.blend')==report['source_R13_sha256']
result.update({'unchanged_STL_ids':same,'new_STL_checks':new,'source_R13_sha256':report['source_R13_sha256']})
(OUT/'validation.json').write_text(json.dumps(result,indent=2))
if '--check-only' in sys.argv:
 print('R14_GEOMETRY_VALIDATION_COMPLETE',json.dumps({'unchanged_STLs':len(same),'new_STLs':len(new),'motion_samples':sum(p['sample_count'] for p in motion)}),flush=True)
 raise SystemExit(0)
assert (OUT/'ASSEMBLY_R14.md').exists()
for name in ['Split_shell_R07_reference.md','Electronics_R06_reference.md','Geekworm_mount_R05_reference.md','R07_JOINTS.csv','ToF_R09_reference.md']:
 shutil.copy2(OLDPACK/name,PACK/name)
shutil.copytree(OLDPACK/'historical_R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
for target in [PACK,CHANGED]:
 shutil.copy2(OUT/'ASSEMBLY_R14.md',target/'START_HERE.md')
 for name in ['geometry_checks.json','validation.json','panel_inside.png','panel_exploded.png','hood_assembled.png']:
  shutil.copy2(OUT/name,target/name)
 with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
  for p in sorted(target.rglob('*')):
   if p.is_file():z.write(p,p.relative_to(OUT))
 with zipfile.ZipFile(OUT/(target.name+'.zip')) as z:
  assert z.testzip() is None
  rows=json.loads((target/'manifest.json').read_text())['parts']
  assert len([n for n in z.namelist() if n.endswith('.stl')])==len(rows)
  for r in rows:assert hashlib.sha256(z.read(target.name+'/'+r['file'])).hexdigest()==r['sha256']
print('R14_VALIDATION_COMPLETE',json.dumps({'unchanged_STLs':len(same),'new_STLs':new,'motion_samples':sum(p['sample_count'] for p in motion)}),flush=True)
