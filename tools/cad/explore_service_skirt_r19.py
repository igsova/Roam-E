"""Read-only R18 assembly exploration. Coordinates are mm, scene is 100mm/BU."""
import ast, json, math, sys
from pathlib import Path
import bpy, bmesh
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'mechanical/service_skirt_r19';OUT.mkdir(exist_ok=True)
for file,names in [('create_split_shell_r07.py',{'bounds'}),('validate_panel_bolts_r14.py',{'mesh_arrays'})]:
 for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(node,ast.FunctionDef) and node.name in names:exec(compile(ast.Module(body=[node],type_ignores=[]),'<helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'mechanical/drawer_cleanup_r18/roam_drawer_cleanup_r18.blend'))
S=bpy.context.scene;bpy.context.view_layer.update()
PANELS=['P16B | Wheel-switch side panel R17','P16C | Front ToF skirt R17','P16D | Rear ToF skirt R17']
obs=[o for o in S.objects if o.type=='MESH' and not o.hide_render and not any(c.name=='06 Studio' for c in o.users_collection)]
inventory=[{'name':o.name,'bounds':bounds(o),'properties':{k:str(v) for k,v in o.items()}} for o in obs if bounds(o)[2][0]<105]
(OUT/'R18_lower_inventory.json').write_text(json.dumps(inventory,indent=2))
groups={p:[p] for p in PANELS};removed=[];sensor_names=[]
for o in obs:
 tag=o.get('ToF station','')
 if not tag:
  for t in ['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']:
   if t in o.name and o.name.startswith(('ToF ','P17 |','P18 |')):tag=t;break
 if tag:
  p=PANELS[0] if tag=='Right' else PANELS[1] if tag.startswith('Front') else PANELS[2]
  groups[p].append(o.name);sensor_names.append(o.name)
 if o.name.startswith(('M1 |','M2 |','M3 |','M4 |','S1 |','S2 |','R17 S3 |','R17 S4 |')):
  removed.append(o.name)
 if o.name.startswith('R17 WHEELS rocker'):groups[PANELS[0]].append(o.name)
all_group={n for g in groups.values() for n in g}
env=[o.name for o in obs if o.name not in all_group and o.name not in removed]
arrays={o.name:mesh_arrays(o) for o in obs}
def combined(names):
 vs=[];fs=[];owners=[]
 for n in names:
  v,f=arrays[n];off=len(vs);vs+=v;fs += [tuple(i+off for i in tri) for tri in f];owners += [n]*len(f)
 return vs,fs,owners
def tree(v,f):return BVHTree.FromPolygons(v,f,all_triangles=True,epsilon=0)
results=[]
for index,(p,direction) in enumerate(zip(PANELS,[(1,0,0),(0,-1,0),(0,1,0)])):
 fixed=env+[n for q in PANELS[index+1:] for n in groups[q]]
 sv,sf,so=combined(fixed);st=tree(sv,sf);mv,mf,mo=combined(groups[p]);d=Vector(direction)
 result={'panel':p,'moving':groups[p],'removed':removed,'paths':[]}
 for axis in [d,Vector((0,0,1)),(d+Vector((0,0,1))).normalized(),(d+Vector((-1,0,0))).normalized() if index>0 else Vector((1,0,1)).normalized()]:
  summary={};samples=[]
  for mm in range(0,241,2):
   pairs=tree([v+axis*(mm/100) for v in mv],mf).overlap(st)
   seen={mo[a]+' <> '+so[b] for a,b in pairs}
   if seen:samples.append(mm)
   for pair in seen:
    if pair not in summary:summary[pair]=[mm,mm,0]
    summary[pair][1]=mm;summary[pair][2]+=1
  row={'axis':list(axis),'hits_mm':samples,'pairs_first_last_count':summary}
  result['paths'].append(row);print('R18_PATH',p,list(axis),json.dumps(summary),flush=True)
 results.append(result)
(OUT/'R18_path_exploration.json').write_text(json.dumps(results,indent=2))
print('R18_EXPLORE_DONE',flush=True)
