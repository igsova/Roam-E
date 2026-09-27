"""Independent R11 identity, flat-roof, mesh and installed-sensor path QA.

Collision sampling is 0.5 mm over each 240 mm linear removal path. This is a
digital sampled test, not continuous collision proof or physical validation.
Each moving section's K/H screws and washers are removed; the left K screws
can hold the still-stationary sections. Captive nuts, plugs and P12 sleeves
stay with their respective parts. The camera/frame/LiDAR stay put.
"""
import ast,hashlib,json,shutil,zipfile,math
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/service_hood_r10'
OUT=ROOT/'mechanical/flat_roof_r11'
PACK=OUT/'ROAM_STL_R11_Flat_Roof_K1C'
CHANGED=OUT/'ROAM_R11_hood_replacement'
OLDPACK=OLD/'ROAM_STL_R10_Service_Hood_K1C'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for n in ast.parse((ROOT/'tools/cad/create_split_shell_r07.py').read_text()).body:
 if isinstance(n,ast.FunctionDef) and n.name in {'snapshot','metrics','remove','bounds'}:
  exec(compile(ast.Module(body=[n],type_ignores=[]),'<QA>','exec'))
report=json.loads((OUT/'geometry_checks.json').read_text())
assert HASH(OLD/'roam_service_hood_r10.blend')==report['source_R10_sha256']
assert not report['unexpected_original_changes']
bpy.ops.wm.open_mainfile(filepath=str(OLD/'roam_service_hood_r10.blend'))
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
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_flat_roof_r11.blend'))
bpy.context.view_layer.update();S=bpy.context.scene
OLDNAMES=set(report['renamed_hood_objects'])
FN='P15C | Front slide-on smiling hood R11';RN='P15D | Rear slide-on hood R11';CN='P15B | Removable hood side panel R07'
assert set(S.objects.keys())==(set(original)-OLDNAMES)|{FN,RN}
changed=[n for n,h in original.items() if n not in OLDNAMES and snapshot(S.objects[n])!=h]
assert not changed,changed
groups={FN:[S.objects[FN]],RN:[S.objects[RN],S.objects['R10 H4 | M3 captive nut']],CN:[S.objects[CN]]}
removed=[]
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
for ob in env+[o for g in groups.values() for o in g]+[S.objects[n] for n in rear_hold+front_hold]+[S.objects['R10 H4 | M3x12 screw'],S.objects['R10 H4 | M3 washer']]:
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
 for step in range(481):
  offset=step*.5
  mt=bvh([p+direction*(offset/100) for p in mv],mf)
  pairs=mt.overlap(st)
  if pairs:
   by={}
   for i,j in pairs:
    key=mo[i]+' <> '+owners[j];by[key]=by.get(key,0)+1
   hits.append({'offset_mm':offset,'pairs':by})
  if step%80==0:print('R11_PATH',name,offset,'hits',len(hits),flush=True)
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

# Roof-only revision: every old hood vertex strictly below the modified
# roof band must still lie on the delivered mesh. No scaling or joint relocation.
newtrees=[bvh(*arrays[n]) for n in [FN,RN]];distances=[]
for verts,_ in old_geometry:
 for p in verts:
  if p.z*100>=189.9999:continue
  distances.append(min(t.find_nearest(p)[3]*100 for t in newtrees))
assert max(distances)<.01,max(distances)

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
for name in ['R10 H4 | M3x12 screw','R10 H4 | M3 washer','R10 H4 | M3 captive nut']:
 pairs=bvh(*arrays[name]).overlap(bvh(sv,sf))
 installed_hardware_clashes[name]=sorted(set(owners[j] for _,j in pairs))
assert not any(installed_hardware_clashes.values()),installed_hardware_clashes

result={'revision':'R11','status':'pending' if collisions else 'digital mesh and sampled assembly-path checks passed',
 'unchanged_R10_objects':len(original)-2,'unexpected_original_changes':changed,
 'removed_for_assembly_screws_and_washers':removed,'sampled_removal_paths':motion,
 'assembly_order':'Rear P15D, secure its left K screw loosely; front P15C, secure its left K screw loosely; unchanged P15B; remaining screws/washers.',
 'protected_vertex_samples_below_roof':len(distances),'max_protected_vertex_error_mm':max(distances),
 'H4_vertical_tool_access_6mm_diameter':'clear','new_hardware_against_fixed_robot':installed_hardware_clashes,
 'contact_QA_tolerance':'P12 sleeve QA copies shrink 0.2% only to separate intentional sleeve/frame face contact; all delivered geometry unchanged.',
 'limits':['Sampled 0.5 mm mesh motion, not continuous collision detection or a physical-fit guarantee.',
  'Unmodeled wires, connector bend radii, manufacturing tolerances and flexible printed parts are not simulated.',
  'Fit joint coupons first; dry-fit covers with power disconnected. Never force a cover against optics.',
  'Nominal bounding-box bed fit is not sliced toolpath/support validation. Support quantity and print time unmeasured.']}
(OUT/'validation.json').write_text(json.dumps(result,indent=2))
assert not collisions,'Assembly collisions: inspect validation.json'
manifest=json.loads((PACK/'manifest.json').read_text())
prior={r['id']:r for r in json.loads((OLDPACK/'manifest.json').read_text())['parts']}
same=[];new={}
for row in manifest['parts']:
 p=PACK/row['file'];assert HASH(p)==row['sha256']
 if row['id'] not in {'P15C','P15D','Q11C','Q11D'}:
  assert row['sha256']==prior[row['id']]['sha256'];same.append(row['id'])
 else:
  bpy.ops.wm.stl_import(filepath=str(p),use_facet_normal=False)
  ob=bpy.context.object;m=metrics(ob,1)
  ob.data.calc_loop_triangles()
  flat=sum(t.area for t in ob.data.loop_triangles if t.normal.z<-.99 and all(abs(ob.data.vertices[i].co.z)<.001 for i in t.vertices))
  m['flat_bed_contact_area_mm2']=flat
  assert flat>(9000 if row['id'] in {'P15C','P15D'} else 100), (row['id'],flat)
  remove(ob)
  assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 and m['volume_mm3']>0 and abs(m['min_z_mm'])<.001,m
  assert all(d+8<220 for d in m['dimensions_mm'][:2]) and m['dimensions_mm'][2]<250
  assert all(abs(a-b)<.002 for a,b in zip(m['dimensions_mm'],row['reimport_check']['dimensions_mm']))
  new[row['id']]=m
assert len(same)==44 and set(new)=={'P15C','P15D','Q11C','Q11D'} and len(manifest['parts'])==48
result.update({'unchanged_STL_ids':same,'new_STL_checks':new,'source_R10_sha256':report['source_R10_sha256']})
(OUT/'validation.json').write_text(json.dumps(result,indent=2))
assert (OUT/'ASSEMBLY_R11.md').exists()
for name in ['Split_shell_R07_reference.md','Electronics_R06_reference.md','Geekworm_mount_R05_reference.md','R07_JOINTS.csv','ToF_R09_reference.md']:
 shutil.copy2(OLDPACK/name,PACK/name)
shutil.copytree(OLDPACK/'historical_R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
for target in [PACK,CHANGED]:
 shutil.copy2(OUT/'ASSEMBLY_R11.md',target/'START_HERE.md')
 for name in ['geometry_checks.json','validation.json','flat_roof_detail.png','roof_down_print_layout.png','hood_assembled.png']:
  shutil.copy2(OUT/name,target/name)
 with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
  for p in sorted(target.rglob('*')):
   if p.is_file():z.write(p,p.relative_to(OUT))
 with zipfile.ZipFile(OUT/(target.name+'.zip')) as z:
  assert z.testzip() is None
  rows=json.loads((target/'manifest.json').read_text())['parts']
  assert len([n for n in z.namelist() if n.endswith('.stl')])==len(rows)
  for r in rows:assert hashlib.sha256(z.read(target.name+'/'+r['file'])).hexdigest()==r['sha256']
print('R11_VALIDATION_COMPLETE',json.dumps({'unchanged_STLs':len(same),'new_STLs':new,'motion_samples':sum(p['sample_count'] for p in motion)}),flush=True)
