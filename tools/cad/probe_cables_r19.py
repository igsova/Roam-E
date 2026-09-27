"""Provisional straight Dupont plug and R6 wire-bend packaging probe."""
import ast,json,math
from pathlib import Path
import bpy,bmesh
from mathutils import Vector,Matrix
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/service_skirt_r19'
for file,names in [('create_split_shell_r07.py',{'bounds','box','cyl','clean','remove','clone','boolean','intersection','metrics'}),('create_lower_controls_r17.py',{'local','cb','cy'}),('validate_panel_bolts_r14.py',{'mesh_arrays'})]:
 for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(node,ast.FunctionDef) and node.name in names:exec(compile(ast.Module(body=[node],type_ignores=[]),'<helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'mechanical/drawer_cleanup_r18/roam_drawer_cleanup_r18.blend'))
S=bpy.context.scene;bpy.context.view_layer.update()
env=[o for o in S.objects if o.type=='MESH' and not o.hide_render and bounds(o)[2][0]<80 and not any(c.name=='06 Studio' for c in o.users_collection)]
trees={o.name:BVHTree.FromPolygons(*mesh_arrays(o),all_triangles=True) for o in env}
report=[]
for tag in ['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']:
 pcb=S.objects['ToF '+tag+' | Reference PCB 25x10.66 R09'];mat=pcb.matrix_world.copy()
 tools=[('housing',cb(mat,(0,-6,-9.65),(15.8,3.2,14.0))),('wire bend',cb(mat,(0,-2.8,-20.5),(15.8,9.6,8.4))),('upward bundle',cb(mat,(0,15,-22.45),(15.8,28,2.4)))]
 for label,probe in tools:
  tr=BVHTree.FromPolygons(*mesh_arrays(probe),all_triangles=True);hits=[]
  for o in env:
   # Ignore plug pins physically inserted into housings, not retainer/frame.
   if o.get('ToF station')==tag and o.name.startswith('ToF '):continue
   if not all(min(a[1],b[1])>max(a[0],b[0]) for a,b in zip(bounds(o),bounds(probe))):continue
   if tr.overlap(trees[o.name]):hits.append(o.name)
  report.append({'station':tag,'probe':label,'bounds':bounds(probe),'surface_intersections':hits})
  remove(probe)
 print('CABLE_PROBE',tag,[r['surface_intersections'] for r in report[-3:]],flush=True)
(OUT/'R18_cable_probe.json').write_text(json.dumps(report,indent=2))
