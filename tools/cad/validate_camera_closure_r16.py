"""R16 regression: check camera-end closure, R15 identity, then inherited QA.
Reuses R15's unchanged sensor/joint/motion tests with explicit path adaptation.
The closure rays must FAIL on R15 and PASS on the delivered R16 surface.
"""
import ast,hashlib,json,math,sys
from pathlib import Path
import bpy,bmesh
from mathutils import Vector,Matrix
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/camera_closure_r16';OLD15=ROOT/'mechanical/sensor_enclosure_r15'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for file,names in [('create_split_shell_r07.py',{'snapshot','bounds','metrics'}),('validate_panel_bolts_r14.py',{'mesh_arrays'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<QA helper>','exec'))
def end_closure(front,cap):
 vertices=[];faces=[]
 for ob in [front,cap]:
  v,f=mesh_arrays(ob);base=len(vertices);vertices+=v;faces += [tuple(base+i for i in p) for p in f]
 tree=BVHTree.FromPolygons(vertices,faces,all_triangles=True);summary={}
 for sign,label in [(-1,'left'),(1,'right')]:
  misses=[];n=0;minimum_margin=1e9;excluded=0
  for iz in range(104):
   z=135.9+.5*iz;dz=abs(z-161.65)
   outer=99
   if dz>11:
    # Compare with the actual 16-segment quarter-circle, not an ideal arc.
    # Close to its top/bottom a horizontal arc/chord difference exceeds 0.04.
    points=[(84+15*math.cos(k*math.pi/32),11+15*math.sin(k*math.pi/32)) for k in range(17)]
    for (x0,z0),(x1,z1) in zip(points,points[1:]):
     if z0<=dz<=z1:outer=x0+(x1-x0)*(dz-z0)/(z1-z0);break
   expected=outer if sign<0 else min(94.85,outer)
   for iy in range(41):
    y=-91+.5*iy
    # Existing H7 receiver has a deliberate sliding-lap clearance. It is not
    # an enclosure wall and must remain open for the retained side panel.
    if sign>0 and z<142.31 and -87.76<y<-82.14:excluded+=1;continue
    n+=1
    hit,normal,face,dist=tree.ray_cast(Vector((sign*150,y,z))/100,Vector((-sign,0,0)))
    margin=sign*hit.x*100-expected if hit is not None else -999
    minimum_margin=min(minimum_margin,margin)
    if margin<-.04:misses.append({'y':y,'z':z,'expected_x_abs_mm':expected,'first_hit_x_mm':hit.x*100 if hit is not None else None})
  summary[label]={'sample_count':n,'excluded_existing_H7_receiver_samples':excluded,'miss_count':len(misses),'minimum_silhouette_margin_mm':minimum_margin,'example_misses':misses[:12]}
 return summary
bpy.ops.wm.open_mainfile(filepath=str(OLD15/'roam_sensor_enclosure_r15.blend'));bpy.context.view_layer.update();S=bpy.context.scene
original15={o.name:snapshot(o) for o in S.objects}
old_closure=end_closure(S.objects['P15C | Enclosed camera front hood R15'],S.objects['P15B | Raised removable hood side panel R15'])
assert sum(c['miss_count'] for c in old_closure.values())>0,'Regression test must detect the R15 defect'
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_camera_closure_r16.blend'));bpy.context.view_layer.update();S=bpy.context.scene
front16='P15C | Closed camera surround front hood R16';front15='P15C | Enclosed camera front hood R15'
changed15=[n for n,h in original15.items() if n!=front15 and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not changed15,changed15
assert set(S.objects.keys())==(set(original15)-{front15})|{front16}
new_closure=end_closure(S.objects[front16],S.objects['P15B | Raised removable hood side panel R15'])
closure_result={'R15_baseline':old_closure,'R16_result':new_closure,'criterion':'First side-ray hit covers the polygonal rear-bezel silhouette, sampled every 0.5 mm in Y/Z across each end. The removable-panel ownership clips the right side at X=94.85 mm. The existing H7 sliding-lap receiver zone is explicitly excluded and retains its clearance.','limits':'Sampled coverage test, not a watertight/dustproof assembly or continuous-surface proof.'}
(OUT/'camera_closure_checks.json').write_text(json.dumps(closure_result,indent=2))
print('R16_END_CLOSURE',json.dumps(closure_result),flush=True)
assert all(c['miss_count']==0 for c in new_closure.values()),new_closure
r=json.loads((OUT/'geometry_checks.json').read_text())
assert HASH(OLD15/'roam_sensor_enclosure_r15.blend')==r['source_R15_sha256']
r16_source_hash=r['source_R15_sha256']
# Unchanged R15 validation suite remains the source of the sensor, bolt,
# roof-planarity, nut-seat and three 250 mm sampled removal-path checks.
code=(ROOT/'tools/cad/validate_sensor_enclosure_r15.py').read_text()
adaptations={"mechanical/sensor_enclosure_r15":"mechanical/camera_closure_r16","roam_sensor_enclosure_r15.blend":"roam_camera_closure_r16.blend",front15:front16,"'revision':'R15'":"'revision':'R16'","'validation.json'":"'validation_baseline.json'","'validation_quick.json'":"'validation_baseline_quick.json'"}
for a,b in adaptations.items():
 assert a in code,a
 code=code.replace(a,b)
exec(compile(code,'<inherited R15 sensor/joint/motion QA>','exec'))
result=json.loads((OUT/('validation_baseline_quick.json' if '--quick' in sys.argv else 'validation_baseline.json')).read_text())
result.update({'revision':'R16','camera_end_closure':closure_result,'source_R15_sha256':r16_source_hash,'R15_unchanged_objects':len(original15)-1,'R15_unexpected_original_changes':changed15})
(OUT/('validation_quick.json' if '--quick' in sys.argv else 'validation.json')).write_text(json.dumps(result,indent=2))
print('R16_VALIDATION_COMPLETE',result['status'],flush=True)
