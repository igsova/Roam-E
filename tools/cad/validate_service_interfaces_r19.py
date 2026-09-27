"""R19 complementary checks: STL reimports, bolt driver access, drawer sweep."""
import ast,json,hashlib
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/service_skirt_r19'
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','boolean','bounds','metrics','intersection'}),('create_lower_controls_r17.py',{'local','cb','cy'}),('validate_panel_bolts_r14.py',{'mesh_arrays'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<QA>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_service_skirt_r19.blend'));S=bpy.context.scene;bpy.context.view_layer.update()
report=json.loads((OUT/'geometry_checks.json').read_text());checks={'status':'RUNNING','master_sha256':hashlib.sha256((OUT/'roam_service_skirt_r19.blend').read_bytes()).hexdigest()}
visible=[o for o in S.objects if o.type=='MESH' and not o.hide_render and not any(c.name=='06 Studio' for c in o.users_collection)]
access=[]
for j in report['joints']:
 probe=cy(Matrix(j['matrix']),(9,0,-24),3.0,39.8)
 hits=[]
 for o in visible:
  v=intersection(probe,o)
  if v>.025:hits.append([o.name,v])
 remove(probe);access.append({'joint':j['id'],'driver_diameter_mm':6,'outside_access_depth_mm':39.8,'hits':hits})
checks['external_driver_access']=access
rows=json.loads((OUT/'new_parts.json').read_text());stls=[]
for row in rows:
 path=OUT/'ROAM_R19_skirt_replacement'/row['file'];assert hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256']
 bpy.ops.wm.stl_import(filepath=str(path));o=bpy.context.object;m=metrics(o,factor=1)
 assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 and m['volume_mm3']>0,(row['id'],m)
 assert abs(m['min_z_mm'])<.001,m
 assert all(abs(a-b)<.02 for a,b in zip(m['dimensions_mm'],row['reimport_check']['dimensions_mm']))
 assert all(d+4<220 for d in m['dimensions_mm'][:2]),(row['id'],m)
 stls.append({'id':row['id'],'solid':m,'nominal_2mm_brim_fits_220mm_square':True});remove(o)
checks['STL_reimports']=stls
moving=[o for o in visible if o.name.startswith(('P05 |','Component | Battery','Battery |','R17 BATTERY rocker'))]
removed=[o.name for o in visible if o.name.startswith(('E-1 |','E1 |')) and 'drawer lock nut' not in o.name]
fixed=[o for o in visible if o not in moving and o.name not in removed]
def combined(obs):
 vs=[];fs=[];owners=[]
 for o in obs:
  v,f=mesh_arrays(o);off=len(vs);vs+=v;fs += [tuple(i+off for i in tri) for tri in f];owners += [o.name]*len(f)
 return vs,fs,owners
def tree(v,f):return BVHTree.FromPolygons(v,f,all_triangles=True,epsilon=0)
mv,mf,mo=combined(moving);sv,sf,so=combined(fixed);st=tree(sv,sf);hits=[]
for step in range(301):
 mm=step*.5;result=tree([v+Vector((0,mm/100,0)) for v in mv],mf).overlap(st)
 if result:hits.append({'mm':mm,'pairs':sorted({mo[a]+' <> '+so[b] for a,b in result})})
 if step%100==0:print('R19_DRAWER',mm,'hits',len(hits),flush=True)
checks['drawer_withdrawal']={'samples':301,'step_mm':.5,'range_mm':[0,150],'collisions':hits,'removed':removed}
checks['status']='PASS' if not hits and not any(a['hits'] for a in access) else 'FAIL'
(OUT/'interface_checks.json').write_text(json.dumps(checks,indent=2));print('R19_INTERFACES_COMPLETE',checks['status'],flush=True)
