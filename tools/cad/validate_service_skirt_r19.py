"""Independent R19 preservation, solid, connector and sampled motion checks."""
import ast,hashlib,json,math
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/service_skirt_r19'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','boolean','bounds','metrics','snapshot','intersection'}),('create_lower_controls_r17.py',{'local','cb','cy'}),('validate_panel_bolts_r14.py',{'mesh_arrays'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<QA>','exec'))
report=json.loads((OUT/'geometry_checks.json').read_text());source=ROOT/'mechanical/drawer_cleanup_r18/roam_drawer_cleanup_r18.blend'
assert HASH(source)==report['source_R18_sha256']
bpy.ops.wm.open_mainfile(filepath=str(source));bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in bpy.context.scene.objects}
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_service_skirt_r19.blend'));S=bpy.context.scene;bpy.context.view_layer.update()
checks={'revision':'R19','status':'RUNNING','source_R18_sha256':HASH(source),'master_sha256':HASH(OUT/'roam_service_skirt_r19.blend'),'limits':report['limits']+['Paths sampled at 0.5mm; this is not a continuous collision proof or physical fit test. Reverse paths represent assembly.','No entire flexible harness routing or electrical design validation. Jumper terminals end below the chassis.']}
def save():(OUT/'validation.json').write_text(json.dumps(checks,indent=2))
checks['unexpected_original_changes']=[n for n,h in original.items() if n not in report['renames'] and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not checks['unexpected_original_changes']
checks['unchanged_object_count']=len(original)-3
checks['R18_drawer_unchanged']=snapshot(S.objects['P05 | Continuous curved battery drawer R18'])==original['P05 | Continuous curved battery drawer R18']
panels=list(report['renames'].values());cleats=[j['cleat'] for j in report['joints']];parts=[S.objects[n] for n in panels+cleats]
checks['solids']={o.name:metrics(o) for o in parts}
assert all(m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 and m['volume_mm3']>0 for m in checks['solids'].values())

# Physical feature measurements from actual surfaces, not just inputs.
joint_samples=[]
for joint in report['joints']:
 panel=S.objects[joint['panel']];cleat=S.objects[joint['cleat']];mat=Matrix(joint['matrix']);nm=Matrix(joint['nut_matrix'])
 readings=[]
 for d in [.2,1.5,2.4]:
  expected=2.65 if d<=2 else 2.65+.3*(d-2)/.8
  for k in range(6):
   a=k*math.tau/6;o=nm@(Vector((0,0,d))/100);direction=nm.to_3x3()@Vector((math.cos(a),math.sin(a),0))
   hit,p,_,_=cleat.ray_cast(o,direction);assert hit
   distance=(p-o).length*100;assert abs(distance-expected)<.008,(joint['id'],d,distance,expected)
   readings.append(distance)
 # Broad mating lands, away from the bore and original foot.
 origin=mat@(Vector((9,5,3.7))/100);normal=mat.to_3x3()@Vector((0,0,1))
 hit,p,_,_=cleat.ray_cast(origin,normal);assert hit;cf=(mat.inverted()@p).z*100
 hit,p,_,_=panel.ray_cast(origin,-normal);assert hit;pf=(mat.inverted()@p).z*100
 assert abs(cf-pf-.4)<.01,(joint['id'],cf,pf)
 joint_samples.append({'joint':joint['id'],'measured_mating_gap_mm':cf-pf,'retention_radii_mm':readings})
checks['joint_surface_measurements']=joint_samples
stations=[]
for tag in ['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']:
 pcb=S.objects['ToF '+tag+' | Reference PCB 25x10.66 R09'];mat=pcb.matrix_world.copy();part=S.objects[panels[0] if tag=='Right' else panels[1] if tag.startswith('Front') else panels[2]]
 ds=[]
 for i in range(6):
  for dv in [-.85,0,.85]:
   o=mat@(Vector((-6.35+2.54*i,-6+dv,-.85))/100);direction=mat.to_3x3()@Vector((0,0,1))
   hit,p,_,_=part.ray_cast(o,direction);assert hit
   d=(p-o).length*100;assert abs(d-2)<.006,(tag,d);ds.append(d)
 stations.append({'station':tag,'solder_relief_mm':ds})
checks['unchanged_ToF_reliefs']=stations
save();print('R19_DIMENSIONS_PASS',flush=True)

visible=[o for o in S.objects if o.type=='MESH' and not o.hide_render and not any(c.name=='06 Studio' for c in o.users_collection)]
reserves=[o for o in S.objects if o.name.startswith('R19 ToF ') and 'reserve' in o.name]
arrays={o.name:mesh_arrays(o) for o in visible+reserves}
def tree(v,f):return BVHTree.FromPolygons(v,f,all_triangles=True,epsilon=0)
def combined(names):
 vs=[];fs=[];owners=[]
 for n in names:
  v,f=arrays[n];off=len(vs);vs+=v;fs += [tuple(i+off for i in tri) for tri in f];owners += [n]*len(f)
 return vs,fs,owners

# Full-size Boolean intersections catch containment as well as surface clashes.
collisions=[];intentional=[];pairs=set()
for a in parts:
 for b in visible:
  pair=tuple(sorted([a.name,b.name]))
  if a==b or pair in pairs:continue
  pairs.add(pair)
  if not all(min(x[1],y[1])-max(x[0],y[0])>.015 for x,y in zip(bounds(a),bounds(b))):continue
  # Original sensor mounts are unchanged and intentionally meet the shell.
  if b.get('ToF station') and not b.name.startswith('R19'):continue
  # Coincident metal/plastic bearing faces make Boolean intersection
  # ill-conditioned. Separate only NEW exterior metal fasteners by 2 microns
  # along their bolt axis for this static volume test; never shrink the shell.
  joint=next((j for j in report['joints'] if b.name.startswith('R19 '+j['id']+' | External')),None)
  if joint:
   probe=clone(b,'_QA metal contact separation')
   probe.location-=Matrix(joint['matrix']).to_3x3()@Vector((0,0,.002/100))
   bpy.context.view_layer.update();v=intersection(a,probe);remove(probe)
  else:v=intersection(a,b)
  if v<.025:continue
  if any(j['cleat']==a.name and j['nut_object']==b.name for j in report['joints']) and v<3:
   intentional.append([a.name,b.name,v]);continue
  # Existing S1..S4 PETG crush-rib/nut contacts are unchanged by R19.
  if a.name in panels and 'nut' in b.name and b.name.startswith(('S1 |','S2 |','R17 S3 |','R17 S4 |')) and v<3:
   intentional.append([a.name,b.name,v]);continue
  collisions.append([a.name,b.name,v])
 print('R19_STATIC',a.name,'new contacts',len(collisions),flush=True)
checks['new_static_collisions']=collisions;checks['intentional_nut_grip_interferences']=intentional
checks['static_metal_contact_test_tolerance_mm']=.002;save()

cable_hits=[]
for r in reserves:
 hits=[]
 for o in visible:
  # Female housings intentionally contain their header pins; reserve spans all6.
  if o.get('ToF station')==r['ToF station'] and (o.name.startswith('ToF ') or o.name.startswith('R19 ToF ')):continue
  if not all(min(x[1],y[1])-max(x[0],y[0])>.015 for x,y in zip(bounds(r),bounds(o))):continue
  v=intersection(r,o)
  if v>.025:hits.append([o.name,v])
 cable_hits.append({'reserve':r.name,'hits':hits})
checks['female_connector_and_bend_reserves']=cable_hits;save();print('R19_CABLES',json.dumps([r for r in cable_hits if r['hits']]),flush=True)

groups={p:[p] for p in panels};removed=[]
for o in visible+reserves:
 tag=o.get('ToF station','')
 if tag:
  p=panels[0] if tag=='Right' else panels[1] if tag.startswith('Front') else panels[2]
  groups[p].append(o.name)
 if o.name.startswith('R17 WHEELS rocker'):groups[panels[0]].append(o.name)
 if o.name.startswith(('S1 |','S2 |','R17 S3 |','R17 S4 |')):
  if 'nut' in o.name:groups[panels[0] if o.name.startswith(('S1 |','S2 |')) else panels[2]].append(o.name)
  else:removed.append(o.name)
 if o.name.startswith('R19 M') and 'Retained' not in o.name:removed.append(o.name)
all_group={n for g in groups.values() for n in g}
env=[o.name for o in visible if o.name not in all_group and o.name not in removed]
motions=[]
for k,(p,axis) in enumerate(zip(panels,[(1,0,0),(0,-1,0),(0,1,0)])):
 fixed=env+[n for q in panels[k+1:] for n in groups[q]]
 sv,sf,so=combined(fixed);st=tree(sv,sf);mv,mf,mo=combined(groups[p]);d=Vector(axis);hits=[]
 for step in range(481):
  mm=step*.5;result=tree([v+d*(mm/100) for v in mv],mf).overlap(st)
  if result:
   by={}
   for a,b in result:
    key=mo[a]+' <> '+so[b];by[key]=by.get(key,0)+1
   hits.append({'mm':mm,'pairs':by})
  if step%120==0:print('R19_MOTION',p,mm,'hits',len(hits),flush=True)
 dominant=max(range(3),key=lambda i:abs(d[i]));sign=d[dominant]
 gap=min((v[dominant]+d[dominant]*2.4)*sign*100 for v in mv)-max(v[dominant]*sign*100 for v in sv)
 assert gap>10,(p,gap)
 motions.append({'panel':p,'direction':list(d),'step_mm':.5,'range_mm':[0,240],'samples':481,'moving_objects':groups[p],'fixed_objects':fixed,'removed_service_screws_and_washers':removed,'exit_gap_mm':gap,'collisions':hits})
 checks['assembly_and_disassembly']=motions;save()
checks['status']='PASS' if not collisions and not any(r['hits'] for r in cable_hits) and not any(r['collisions'] for r in motions) else 'FAIL'
save();print('R19_VALIDATION_COMPLETE',checks['status'],flush=True)
