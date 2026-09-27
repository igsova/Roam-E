"""R19: three horizontally removable skirt panels with fixed chassis cleats.

Preserves R18 drawer, chassis, sensors and hood. Female jumper references are
provisional mechanical envelopes, not an electrical pin-assignment diagram.
"""
import ast,hashlib,json,math
from pathlib import Path
import bpy,bmesh
from mathutils import Vector,Matrix
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'mechanical/service_skirt_r19';OUT.mkdir(exist_ok=True)
PACK=OUT/'ROAM_R19_skirt_replacement'
SOURCE=ROOT/'mechanical/drawer_cleanup_r18/roam_drawer_cleanup_r18.blend'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','mesh_object','prism','boolean','bounds','metrics','snapshot','intersection','ray_surface','move_collection'}),('create_lower_controls_r17.py',{'frame','local','cb','cy'}),('create_nut_retention_r13.py',{'ribs'}),('create_panel_bolts_r14.py',{'freeze'}),('create_tof_solder_relief_r08.py',{'export','material'})]:
 for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(node,ast.FunctionDef) and node.name in names:exec(compile(ast.Module(body=[node],type_ignores=[]),'<helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE));bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in S.objects}
renames={'P16B | Wheel-switch side panel R17':'P16B | Outward-service wheel panel R19','P16C | Front ToF skirt R17':'P16C | Outward-service front skirt R19','P16D | Rear ToF skirt R17':'P16D | Outward-service rear skirt R19'}
for old,new in renames.items():freeze(S.objects[old]);S.objects[old].name=new
cap,front,rear=[S.objects[n] for n in renames.values()]
C=bpy.data.collections.new('25 R19 chassis cleats - PRINT');S.collection.children.link(C)
HC=bpy.data.collections.new('26 R19 external skirt bolts - NOT FOR PRINT');S.collection.children.link(HC)
WC=bpy.data.collections.new('27 R19 female jumper reference - NOT FOR PRINT');S.collection.children.link(WC)
RC=bpy.data.collections.new('28 R19 cable keepouts - HIDDEN NOT FOR PRINT');S.collection.children.link(RC)
metal=bpy.data.materials['Machined metal']
blue=material('R19 fixed cleats',(.12,.30,.46));plugmat=material('R19 jumper housing',(.025,.028,.032))
colors=[material('R19 jumper '+str(i),c) for i,c in enumerate([(.55,.025,.02),(.07,.07,.07),(.6,.42,.015),(.015,.38,.18)])]
def reference(o,name,col,mat):
 o.name=name;o.data.materials.clear();o.data.materials.append(mat);move_collection(o,col)
 o['Reference only']='NOT FOR PRINT';return o

def retained_hex_cut(mat):
 # Single closed analytic cavity, avoiding six coincident rib-union seams.
 verts=[]
 for depth,tip in [(0,2.65),(2,2.65),(2.8,2.95),(3.1,2.95)]:
  for i in range(6):
   a=i*math.tau/6;n=Vector((math.cos(a),math.sin(a),0));t=Vector((-n.y,n.x,0))
   for r,s in [(2.95,-2.95/math.sqrt(3)),(2.95,-.6),(tip,-.1),(tip,.1),(2.95,.6)]:
    verts.append(tuple(mat@((n*r+t*s+Vector((0,0,depth)))/100)*100))
 faces=[tuple(reversed(range(30))),tuple(range(90,120))]
 for k in range(3):
  for i in range(30):faces.append((k*30+i,k*30+(i+1)%30,(k+1)*30+(i+1)%30,(k+1)*30+i))
 return mesh_object('_analytic retained hex cavity',verts,faces)

cleats=[];joints=[];edits=[];new_hardware=[]
for jid,part,anchor,normal,u in [('M1',front,(-68,-91),(0,1,0),(1,0,0)),('M2',front,(68,-91),(0,1,0),(-1,0,0)),('M3',rear,(-90,42),(1,0,0),(0,-1,0)),('M4',cap,(90,42),(-1,0,0),(0,-1,0))]:
 # Locate the real curved skin; retain the chassis bolt axes and hardware.
 x,y=anchor;z=73. if jid in ['M1','M2'] else 63.5;n=Vector(normal)
 origin=(x,-200,z) if jid in ['M1','M2'] else (-200 if x<0 else 200,y,z)
 p=ray_surface(part,origin,normal);mat=frame(p,u,(0,0,1),normal)
 roi_pos=(x,y,57.90676);roi_dims=(16,42,4.20) if jid in ['M1','M2'] else (64,16,4.20)
 foot=clone(part,'P19'+jid[-1]+' | Fixed skirt cleat R19')
 boolean(foot,box('_original mounting foot',roi_pos,roi_dims),'INTERSECT')
 boolean(foot,box('_flat chassis seat',(0,0,155.90676),(500,500,200)),'INTERSECT')
 boolean(foot,cb(mat,(0,-15,103.9),(120,120,200)),'INTERSECT')
 # Detach at a plane, with 0.40mm clearance to the removable panel stub.
 cut=box('_foot detachment',roi_pos,tuple(d+.8 for d in roi_dims))
 boolean(cut,cb(mat,(0,-15,103.5),(120,120,200)),'INTERSECT')
 boolean(part,cut)
 # Offset the service bolt 9mm tangentially from the old vertical anchor.
 # This leaves a straight hex-key access column for that existing screw.
 bottom=55.90676-z;top=7.
 boolean(foot,cb(mat,(9,(bottom+top)/2,7.15),(14,top-bottom,6.5)),'UNION')
 boolean(foot,cyl('_anchor head and driver access',(x,y,109.95338),3.6,100.09324))
 # Complete exterior washer land; flat at local w=-0.5.
 boolean(part,cb(mat,(9,0,1.5),(14,14,4)),'UNION')
 boolean(part,cb(mat,(9,0,-10.5),(14,14,20)))
 for ob in [part,foot]:boolean(ob,cy(mat,(9,0,5),1.7,32,96))
 nutframe=mat@Matrix.Translation(Vector((9,0,7.6))/100)
 boolean(foot,retained_hex_cut(nutframe))
 foot.data.materials.clear();foot.data.materials.append(blue);move_collection(foot,C)
 foot['Manufacture']='PRINT';foot['Assembly']='Stays on the chassis; reuse '+jid+' original M3x12, washers and underside nut.'
 part['R19 assembly']='Horizontal withdrawal only, service screws removed; unplug harness first.'
 shaft=cy(mat,(9,0,5),1.45,12,64);head=cy(mat,(9,0,-2.5),2.75,3,64)
 boolean(head,cy(mat,(9,0,-3.9),2.5/math.sqrt(3),1.4,6));boolean(shaft,head,'UNION')
 screw=reference(shaft,'R19 '+jid+' | External M3x12 screw',HC,metal)
 washer=cy(mat,(9,0,-.75),3,.5,64);boolean(washer,cy(mat,(9,0,-.75),1.7,2))
 washer=reference(washer,'R19 '+jid+' | External washer',HC,metal)
 nut=cy(mat,(9,0,8.9),5.5/math.sqrt(3),2.4,6);boolean(nut,cy(mat,(9,0,8.9),1.4,4))
 nut=reference(nut,'R19 '+jid+' | Retained M3 nut',HC,metal)
 new_hardware += [screw,washer,nut];cleats.append(foot)
 edits.append({'part':part.name,'foot_removal_box_mm':[roi_pos,roi_dims],'service_pad_frame':[list(r) for r in mat]})
 joints.append({'id':jid,'panel':part.name,'cleat':foot.name,'matrix':[list(r) for r in mat],'nut_matrix':[list(r) for r in nutframe],'nut_object':nut.name,'bolt_offset_u_mm':9,'bolt':'M3x12','gap_mm':.4,'nut_pocket_AF_mm':5.9,'retention_tip_AF_mm':5.3,'existing_anchor_xy_mm':anchor})
 print('R19_CLEAT',jid,metrics(foot),flush=True)

# Keep the measured ToF PCB, solder reliefs and optical datums exactly fixed.
# Four 2.5mm female housings, 14mm long; pin locations illustrative only.
# Reserve ALL SIX pin positions, so clearance does not depend on which four
# pins the real harness uses. Wire bends stay below the chassis, not through it.
wire_rows=[]
for tag in ['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']:
 pcb=S.objects['ToF '+tag+' | Reference PCB 25x10.66 R09'];mat=pcb.matrix_world.copy()
 for i in range(4):
  u=-6.35+i*2.54
  housing=cb(mat,(u,-6,-9.65),(2.5,2.5,14))
  housing=reference(housing,f'R19 ToF {tag} | Female jumper {i+1}',WC,plugmat)
  housing['ToF station']=tag;housing['Dimensions provisional']='2.5 x 2.5 x 14mm; user confirmed female jumpers, not housing length.'
  # Swept circular insulation, 1.6mm OD, bend centreline radius 6mm.
  points=[]
  for k in range(25):
   a=k*math.pi/2/24;points.append(Vector((u,-6+6*(1-math.cos(a)),-16.65-6*math.sin(a))))
  points += [Vector((u,t,-22.65)) for t in [1.,2.,3.]]
  verts=[]
  for k,pt in enumerate(points):
   tangent=(points[min(k+1,len(points)-1)]-points[max(k-1,0)]).normalized()
   e=Vector((1,0,0));f=tangent.cross(e).normalized()
   for s in range(12):verts.append(tuple(mat@((pt+.8*(e*math.cos(s*math.tau/12)+f*math.sin(s*math.tau/12)))/100)*100))
  count=len(points);faces=[tuple(reversed(range(12))),tuple(range((count-1)*12,count*12))]
  for k in range(count-1):
   for s in range(12):faces.append((k*12+s,k*12+(s+1)%12,(k+1)*12+(s+1)%12,(k+1)*12+s))
  wire=reference(mesh_object('_wire bend',verts,faces),f'R19 ToF {tag} | Wire bend {i+1}',WC,colors[i]);wire['ToF station']=tag
  wire['Reference only']='Local pigtail only. Harness tails intentionally not modeled. Disconnect before panel removal.'
 for label,pos,dims in [('housing reserve',(0,-6,-9.65),(16.2,3.4,14.4)),('bend reserve',(0,-2,-20.75),(16.2,12.4,9.0))]:
  keep=cb(mat,pos,dims);keep.name=f'R19 ToF {tag} | '+label;move_collection(keep,RC)
  keep['ToF station']=tag;keep['Reference only']='CLEARANCE ENVELOPE, NOT FOR PRINT';keep.hide_render=True;keep.hide_set(True)
 wire_rows.append({'station':tag,'housing_mm':[2.5,2.5,14],'wire_OD_mm':1.6,'bend_radius_mm':6,'reserved_row_width_mm':16.2,'highest_bend_keepout_Z_mm':44.2,'pin_assignment':'Illustrative first four; keepout covers all six. Not a wiring diagram.'})

parts=[cap,front,rear]+cleats
solids={o.name:metrics(o) for o in parts}
assert all(m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 and m['volume_mm3']>0 for m in solids.values()),solids
unexpected=[n for n,h in original.items() if n not in renames and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
report={'revision':'R19 prototype','source_R18_sha256':HASH(SOURCE),'renames':renames,'unchanged_R18_objects':len(original)-len(renames),'unexpected_original_changes':unexpected,'solid_checks':solids,'joints':joints,'edit_regions':edits,'ToF_jumpers':wire_rows,'fixed_chassis_anchor_hardware':'Existing M1..M4 screws, washers and nuts unchanged; now hold separate P191..P194 cleats.','limits':['Housing size and insulation diameter are assumptions. Actual plug engagement, flexible tails and PETG fit require bench check.','Unplug sensor harness tails and wheel switch wiring before removal, with all power disconnected. No live removal.','Only modeled rigid reference and local wire envelopes are evaluated. No flexible cable, load/strength or thermal simulation.']}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
rows=[]
for ob in parts:
 pid=ob.name.split(' |')[0];path=PACK/'01_rigid'/f'{pid}_service_skirt_R19_qty1.stl'
 m=export(ob,path,Matrix.Identity(4));rows.append({'id':pid,'object':ob.name,'file':str(path.relative_to(PACK)),'sha256':HASH(path),'reimport_check':m})
# Actual joint coupons, isolated from the corresponding panel and cleat.
joint=joints[0];mat=Matrix(joint['matrix'])
for pid,name in [('Q19A',joint['panel']),('Q19B',joint['cleat'])]:
 q=clone(S.objects[name],pid+' | Service joint fit sample')
 boolean(q,cb(mat,(9,0,5),(15,14.1,15)),'INTERSECT')
 path=PACK/'04_fit_test'/f'{pid}_M3_PETG_joint_sample_R19_qty1.stl';m=export(q,path,Matrix.Identity(4));remove(q)
 rows.append({'id':pid,'file':str(path.relative_to(PACK)),'sha256':HASH(path),'reimport_check':m})
(OUT/'new_parts.json').write_text(json.dumps(rows,indent=2))
S['Mechanical revision']='R19 serviceable lower skirt; R18 drawer and R16/R15 upper shell unchanged'
S['R19 cable routing']='Female jumper bodies + R6 bends below chassis. Harness tails omitted; unplug before service. Confirm actual plugs.'
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_service_skirt_r19.blend'),compress=True)
print('R19_BUILD_COMPLETE',flush=True)
