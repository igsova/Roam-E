"""R17 prototype: three bolted skirt sections, curved drawer, two rocker references.
Background Blender; does not modify R16. Dimensions in mm, scene 100 mm/BU.
Switch mounting dimensions are provisional pending the actual Hanibos switch.
"""
import ast,csv,hashlib,json,math,shutil,sys,zipfile
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/camera_closure_r16';SOURCE=OLD/'roam_camera_closure_r16.blend'
OUT=ROOT/'mechanical/lower_controls_r17';OUT.mkdir(parents=True,exist_ok=True)
PACK=OUT/'ROAM_STL_R17_Lower_Controls_K1C';CHANGED=OUT/'ROAM_R17_lower_replacement'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
RENDER='--render' in sys.argv
for file,names in [
 ('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','mesh_object','prism','boolean','bounds','metrics','snapshot','intersection','ray_surface','move_collection'}),
 ('create_tof_solder_relief_r08.py',{'export','aim','render','material'}),
 ('create_nut_retention_r13.py',{'ribs'}),('create_panel_bolts_r14.py',{'freeze'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<R17 helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE));bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update();source_hash=HASH(SOURCE)
original={o.name:snapshot(o) for o in S.objects}
FN='P16C | Front ToF skirt R17';RN='P16D | Rear ToF skirt R17';CN='P16B | Wheel-switch side panel R17'
DN='P05 | Curved switched battery drawer R17'
oldmain=S.objects['P16A | Main ToF skirt R07'];cap=S.objects['P16B | Removable ToF side panel R07'];drawer=S.objects['P05 | Battery drawer with integral fascia']
changed_original={oldmain.name,cap.name,drawer.name,'P06L | Battery guide and lock receiver','P06R | Battery guide and lock receiver'}
for o in [oldmain,cap,drawer,S.objects['P06L | Battery guide and lock receiver'],S.objects['P06R | Battery guide and lock receiver']]:freeze(o)
cap.name=CN;drawer.name=DN
front=clone(oldmain,FN);rear=clone(oldmain,RN)
# New seam crosses the left wall above the wheel relief, away from ToF mounts.
SEAM=-4.;GAP=.4
boolean(front,box('_front half',(0,SEAM-GAP/2-250,60),(500,500,300)),'INTERSECT')
boolean(rear,box('_rear half',(0,SEAM+GAP/2+250,60),(500,500,300)),'INTERSECT')
remove(oldmain)
hardware=[];retention=[];joints=[];coupons=[];switches=[]
HC=bpy.data.collections.new('22 R17 lower shell metal hardware - NOT FOR PRINT');S.collection.children.link(HC)
SC=bpy.data.collections.new('23 R17 Hanibos rocker references - NOT FOR PRINT');S.collection.children.link(SC)
metal=bpy.data.materials['Machined metal'];black=material('R17 rocker black',(.018,.021,.025));white=material('R17 switch markings',(.85,.87,.89))

def frame(p,u,v,w):
 return Matrix(((u[0],v[0],w[0],p[0]/100),(u[1],v[1],w[1],p[1]/100),(u[2],v[2],w[2],p[2]/100),(0,0,0,1)))
def local(o,mat):
 bpy.context.view_layer.update();o.data.transform(mat@o.matrix_world);o.matrix_world=Matrix.Identity(4);clean(o);return o
def cb(mat,p,d):return local(box('_R17 box',p,d),mat)
def cy(mat,p,r,h,n=64):return local(cyl('_R17 cylinder',p,r,h,n),mat)
def hw(o,name):
 o.name=name;o.data.materials.clear();o.data.materials.append(metal);move_collection(o,HC);o['Reference only']='Purchased hardware, not for printing';hardware.append(o);return o

# Existing S1/S2 already have reinforced 5 mm bolted tongues. Retain these
# proven interfaces and add the same tapered PETG retention used on the hood.
for jid,sign in [('S1',-1),('S2',1)]:
 nut=S.objects[jid+' | M3 captive nut'];bb=bounds(nut);p=Vector([sum(b)/2 for b in bb]);w=Vector((0,-sign,0));p-=w*1.3
 mat=frame(p,(1,0,0),(0,0,-1),w)
 ribs(cap,mat,5.3)
 retention.append({'id':jid,'part':CN,'nut_object':nut.name,'matrix':[list(r) for r in mat],'tip_AF_mm':5.3})

# Two broad, bolted overlaps at the new left seam. Front receives the outside
# bolt; rear owns the 5 mm nut ear. They separate horizontally along Y.
for jid,z in [('S3',78.0),('S4',93.0)]:
 mat=frame((-116.5,SEAM,z),(0,1,0),(0,0,1),(1,0,0))
 boolean(front,cb(mat,(-5.9,0,2.75),(11.4,13,5.5)),'UNION')
 boolean(front,cb(mat,(-1,0,8.2),(25.6,13.6,5.6)))
 boolean(rear,cb(mat,(5.2,0,5.5),(10,13,10.6)),'UNION')
 ear=local(prism('_5mm S3 S4 rear ear',[(-12,-5.5),(-11,-6.5),(10,-6.5),(10,6.5),(-11,6.5),(-12,5.5)],5.7,10.7),mat)
 boolean(rear,ear,'UNION')
 for o in [front,rear]:boolean(o,cy(mat,(-6,0,6),1.7,35))
 boolean(rear,cy(mat,(-6,0,9.35),5.9/math.sqrt(3),2.9,6))
 pocket=mat@Matrix.Translation(Vector((-6,0,7.9))/100);ribs(rear,pocket,5.3)
 shaft=cy(mat,(-6,0,5.5),1.45,12);head=cy(mat,(-6,0,-2),2.75,3)
 boolean(head,cy(mat,(-6,0,-3.1),2.5/math.sqrt(3),1.4,6));boolean(shaft,head,'UNION');hw(shaft,f'R17 {jid} | M3x12 screw')
 nut=cy(mat,(-6,0,9.2),5.5/math.sqrt(3),2.4,6);boolean(nut,cy(mat,(-6,0,9.2),1.4,4));hw(nut,f'R17 {jid} | M3 captive nut')
 washer=cy(mat,(-6,0,-.25),3,.5);boolean(washer,cy(mat,(-6,0,-.25),1.7,2));hw(washer,f'R17 {jid} | washer')
 retention.append({'id':jid,'part':RN,'nut_object':nut.name,'matrix':[list(r) for r in pocket],'tip_AF_mm':5.3})
 joints.append({'id':jid,'screw':'M3x12','ear_mm':5,'bore_mm':3.4,'receiver_clearance_mm':.3,'matrix':[list(r) for r in mat]})
 if jid=='S3':
  for part,pid in [(front,'Q17A'),(rear,'Q17B')]:
   q=clone(part,pid+' | Lower seam fit sample');boolean(q,cb(mat,(-1,0,5.4),(26,13.4,16)),'INTERSECT');coupons.append((pid,q,Matrix.Identity(4)))

# Rear fascia uses the same superellipse profile as the lower skirt. Small
# planar seats for the two lock screws and rocker are intentional exceptions.
def curve_y(x,z):
 if z<=61:w,d=116.,104.
 elif z<=98:t=(z-61)/37;w,d=116-2*t,104-t
 else:t=(z-98)/2;w,d=114-t,103-t
 return d*max(0,1-(abs(x)/w)**4)**.25
LOCK_SEAT=curve_y(80,78)
def fascia_y(x,z):
 y=curve_y(x,z)
 # Lock washers bear on complete flat 10mm pads, blended to surrounding skin.
 for xx in [-80,80]:
  r=math.hypot(x-xx,z-78);a=max(0,min(1,(8-r)/3));y=y*(1-a)+LOCK_SEAT*a
 return y
def curved_volume(name,back=False,clearance=False):
 xs=[-87+i*174/116 for i in range(117)];zs=[54.5+i*45/30 for i in range(31)]
 verts=[]
 for side in [0,1]:
  for z in zs:
   for x in xs:
    y=(fascia_y(x,z)-3.95 if side==0 else 200) if clearance else (fascia_y(x,z)-(3.6 if side==0 else 0) if not back else (-50 if side==0 else fascia_y(x,z)))
    verts.append((x,y,z))
 nx=len(xs);nz=len(zs);N=nx*nz;faces=[]
 for k in range(nz-1):
  for i in range(nx-1):
   a=k*nx+i;faces.extend([(a,a+nx,a+nx+1,a+1),(a+N,a+1+N,a+nx+1+N,a+nx+N)])
 for i in range(nx-1):
  faces.extend([(i,i+1,i+1+N,i+N),((nz-1)*nx+i,(nz-1)*nx+i+N,(nz-1)*nx+i+1+N,(nz-1)*nx+i+1)])
 for k in range(nz-1):
  for i in [0,nx-1]:
   a=k*nx+i;faces.append((a,a+N,a+nx+N,a+nx))
 return mesh_object(name,verts,faces)
boolean(drawer,box('_remove original flat fascia',(0,147,78),(220,100,80)))
boolean(drawer,curved_volume('_trim tray to rounded front',back=True),'INTERSECT')
boolean(drawer,curved_volume('_new curved fascia'),'UNION')
# Accessible finger recess, closed behind rather than exposing wiring.
handle=box('_handle back',(-22,94.5,76),(42,10,17));boolean(drawer,handle,'UNION')
boolean(drawer,box('_finger recess',(-22,111,76),(34,28,9)))
# The inherited caster cap heads reach Z59.407 while the drawer underside
# is Z58.5. Two longitudinal channels clear their entire withdrawal sweep;
# retain 1.6mm of tray floor below the unchanged Z61.5 battery/liner datum.
for x in [-12,12]:boolean(drawer,box('_caster head travel clearance',(x,38,55.2),(7,78,9.4)))

# Rework only the rear lock ends of P06L/R. Rails, fastening axes, battery
# envelope and guide channel section stay unchanged. Trim overlapping guide
# material so the now curved fascia can close without force.
guides=[]
for sign,label in [(-1,'L'),(1,'R')]:
 g=S.objects[f'P06{label} | Battery guide and lock receiver'];g.name=f'P06{label} | Curved fascia lock guide R17';guides.append(g)
 x=80*sign
 boolean(g,box('_old lock block',(x,105,79),(17,42,26)))
 end=LOCK_SEAT-3.9
 block=box('_new rear lock receiver',(x,(80+end)/2,78),(9,end-80,13.6));boolean(g,block,'UNION')
 # Outer vertical cheek ties the lock boss into the existing guide wall,
 # clear of the inner sliding tongue/channel.
 boolean(g,box('_guide lock outer cheek',(sign*83,(80+end)/2,71.9),(4.0,end-80,25.8)),'UNION')
 # A top-loaded rectangular nut trap: width prevents hex rotation; open top
 # permits loading without glue. Two local crush ribs retain a nominal nut.
 nut_y=LOCK_SEAT-8.4
 mat=frame((x,LOCK_SEAT,78),(1,0,0),(0,0,1),(0,-1,0))
 boolean(g,cy(mat,(0,0,8),1.7,40))
 boolean(g,box('_top load nut slot',(x,nut_y,85),(5.9,2.8,20.35)))
 for side in [-1,1]:
  boolean(g,box('_guide nut grip rib',(x+side*2.8,nut_y,78),(0.3,1.6,3.6)),'UNION')
 boolean(g,curved_volume('_curved fascia guide-end clearance',clearance=True))
 boolean(drawer,cy(mat,(0,0,1),1.7,40))
 # Hardware translated to matching lock datums; retain M3x12 and washers.
 for suffix in ['M3x12 screw','washer','drawer lock nut']:
  name=('E-1' if sign<0 else 'E1')+' | '+suffix;o=S.objects[name];changed_original.add(name)
  o.location.y+=(LOCK_SEAT-101)/100

def switch_mount(part,mat,jid):
 # Flat annular 2.2mm snap seat, wide enough to support the entire Ø23 bezel.
 # The model uses a provisional Ø20.5 cutout; coupons are the release gate.
 pad=cy(mat,(0,0,1),15,5)
 boolean(part,pad,'UNION')
 boolean(part,cy(mat,(0,0,-5),16,10)) # complete front flat at local w=0
 boolean(part,cy(mat,(0,0,13.6),12.2,22.8)) # back counterbore starts at 2.2
 boolean(part,cy(mat,(0,0,0),10.25,55))
 # Switch body and terminals are purchased reference shapes, not printables.
 def ref(ob,label,ma=black):
  ob.name=f'R17 {jid} rocker | '+label;ob.data.materials.clear();ob.data.materials.append(ma);move_collection(ob,SC)
  ob['Reference only']='Hanibos KCD1-style, photo/drawing approximation. NOT FOR PRINT; confirm cutout/clip geometry and DC rating.';switches.append(ob);return ob
 ref(cy(mat,(0,0,-.75),11.5,1.5,96),'23mm bezel')
 ref(cy(mat,(0,0,3.5),10.15,7,80),'20.3mm upper body')
 ref(cy(mat,(0,0,12),9,10,80),'lower housing')
 rocker=cy(mat,(0,0,-2.1),9.6,2.5,80);ref(rocker,'tilting actuator envelope')
 for side in [-1,1]:
  ref(cb(mat,(side*10.4,0,7.8),(.55,4,8)),'snap clip approximation')
  terminal=cb(mat,(side*3,0,21.5),(.8,4.8,9));ref(terminal,'4.8mm terminal',metal)
 # I/O labels placed on exterior of actuator as separate visual references.
 ref(cb(mat,(0,4.0,-3.4),(.65,3.4,.10)),'I marking',white)
 ring=cy(mat,(0,-4,-3.4),1.7,.10,48);boolean(ring,cy(mat,(0,-4,-3.4),1.15,.4,48));ref(ring,'O marking',white)
 return {'id':jid,'part':part.name,'matrix':[list(r) for r in mat],'cutout_diameter_mm':20.5,'snap_seat_thickness_mm':2.2,'bezel_diameter_mm':23,'terminal_depth_mm':26,'reserved_depth_mm':40,'status':'PROVISIONAL: print fit coupon and verify actual clip/body dimensions before main part'}

wheelmat=frame((115.6,-24,84),(0,1,0),(0,0,1),(-1,0,0))
batterymat=frame((40,curve_y(40,78)+.2,78),(1,0,0),(0,0,1),(0,-1,0))
mounts=[switch_mount(cap,wheelmat,'WHEELS'),switch_mount(drawer,batterymat,'BATTERY')]
# Cut battery snap-seat reinforcement back to the original fascia extent at
# its perimeter only. Switch islands stay flat; other surfaces follow contour.
for mat,part,pid in [(batterymat,drawer,'Q17C'),(wheelmat,cap,'Q17D')]:
 q=clone(part,pid+' | Actual rocker mount fit sample');boolean(q,cb(mat,(0,0,1),(32,32,12)),'INTERSECT')
 bm=bmesh.new();bm.from_mesh(q.data);bmesh.ops.triangulate(bm,faces=list(bm.faces));bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=.001/100);bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=.0001/100);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(q.data);bm.free();q.data.update()
 # Match the upright parent part: hole and clip seat print horizontally.
 coupons.append((pid,q,Matrix.Identity(4)))
for diameter,pid in [(20.3,'Q17E'),(20.7,'Q17F')]:
 q=cyl(pid+' | Alternate rocker cutout test',(0,0,1.1),15,2.2,96);boolean(q,cyl('_cutout',(0,0,1),diameter/2,5,96));coupons.append((pid,q,Matrix.Identity(4)))

parts=[front,rear,cap,drawer]+guides
for o in parts:o['Manufacture']='PRINT';o['R17 status']='Prototype: fit coupon required; electrical switching rating not verified'
solids={o.name:metrics(o) for o in parts}
print('R17_SOLIDS',json.dumps(solids),flush=True)
for o in guides:
 bm=bmesh.new();bm.from_mesh(o.data);todo=set(bm.verts);info=[]
 while todo:
  v=todo.pop();group={v};stack=[v]
  while stack:
   for e in stack.pop().link_edges:
    for v in e.verts:
     if v in todo:todo.remove(v);group.add(v);stack.append(v)
  info.append({'verts':len(group),'bb':[(min(v.co[i] for v in group)*100,max(v.co[i] for v in group)*100) for i in range(3)]})
 bm.free();print('R17_GUIDE_COMPONENTS',o.name,info,flush=True)
assert all(m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 and m['volume_mm3']>0 for m in solids.values()),solids
old_names={'P16A | Main ToF skirt R07':FN,'P16B | Removable ToF side panel R07':CN,'P05 | Battery drawer with integral fascia':DN,'P06L | Battery guide and lock receiver':guides[0].name,'P06R | Battery guide and lock receiver':guides[1].name}
unexpected=[n for n,h in original.items() if n not in changed_original and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
report={'revision':'R17 prototype','source_R16_sha256':source_hash,'renames':old_names,'allowed_original_changes':sorted(changed_original),'unchanged_original_objects':len(original)-len(changed_original),'unexpected_original_changes':unexpected,'solid_checks':solids,'new_joints':joints,'nut_retention':retention,'switch_mounts':mounts,'lock_seat_y_mm':LOCK_SEAT,'split_plane_y_mm':SEAM,'split_gap_mm':GAP,'switch_rating':'AC-only 10A/125VAC, 6A/250VAC provided; NO verified DC rating; mechanical mounting only','limits':['No wiring/power changes made. Do not switch battery/motor current until exact DC load/inrush rating is verified.','Switch cutout and snap thickness are provisional, not certified by the ambiguous supplier drawing.','Digital checks do not establish physical fit, strength, PETG creep, cable routing or sliced support requirements.']}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
S['Mechanical revision']='R17 three-piece bolted lower shell and curved battery drawer; R16 hood unchanged';S['Revision']=S['Mechanical revision']
S['R17 switch fit']='Prototype Ø20.5 x 2.2mm seat. Test Q17C/D before printing; cutout/clip profile and DC rating unverified.'
S['R17 electrical']='NO wiring changes/approval. Battery master and wheel branch switching require verified DC current/inrush ratings and source isolation.'
# Samples are not installed robot geometry; export then remove before saving.
rows=[]
for pid,ob,rot,note in [
 ('P16C',front,Matrix.Identity(4),'Front skirt, bottom down. Replaces part of P16A. New S3/S4 bolts connect to rear section.'),
 ('P16D',rear,Matrix.Identity(4),'Rear skirt, bottom down. Includes left ToF station and two retained M3 nuts. Fits with 2mm brim geometrically; inspect exclusions.'),
 ('P16B',cap,Matrix.Identity(4),'Right panel, bottom down; WHEELS switch. Retains S1/S2 axes with PETG nut ribs. Local support may be required; check wheel/ToF and switch details.'),
 ('P05',drawer,Matrix.Identity(4),'Curved battery drawer; BATTERY switch. Reprint both matching P06 guides. Tray/rail axes unchanged.'),
 *[(('P06L' if i==0 else 'P06R'),o,Matrix.Identity(4),'Matching guide rear lock end for curved P05; original rail/channel and mounting axes retained.') for i,o in enumerate(guides)],
 *[(pid,q,rot,'R17 fit sample, not installed. Q17A/B test M3 seam. Q17C/D actual Ø20.5 switch seat; E/F alternatives Ø20.3/20.7 at 2.2mm.') for pid,q,rot in coupons]]:
 folder='04_fit_test' if pid.startswith('Q') else '01_rigid';label={'P16C':'front_skirt','P16D':'rear_skirt','P16B':'wheel_switch_panel','P05':'curved_battery_drawer','P06L':'curved_drawer_guide_L','P06R':'curved_drawer_guide_R'}.get(pid,'fit_sample')
 path=f'{folder}/{pid}_{label}_R17_qty1.stl';print('R17_BEFORE_EXPORT',pid,metrics(ob),flush=True);check=export(ob,PACK/path,rot)
 assert check['dimensions_mm'][2]<250,check
 row={'id':pid,'label':label,'quantity':1,'file':path,'folder':folder,'sources':[ob.name],'revision':'R17 prototype','reimport_check':check,'sha256':HASH(PACK/path),'export_rotation_matrix':[list(r) for r in rot],'note':note,'support':'Inspect sliced supports, especially ToF recesses, wheel arch, guide/nut pockets and rocker counterbores. No support-free claim.','brim_2mm_fits_nominal_K1C':True,'brim_4mm_fits_K1C':all(d+8<220 for d in check['dimensions_mm'][:2])};rows.append(row)
 print('R17_EXPORT',pid,check,flush=True)
for _,q,_ in coupons:remove(q)
(OUT/'new_parts.json').write_text(json.dumps(rows,indent=2))
aim((390,490,330),(0,10,70),340)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_lower_controls_r17.blend'),compress=True)
render('robot_rear_controls.png')
for o in S.objects:
 if o.type not in ['CAMERA','LIGHT'] and o not in parts and o not in switches and o not in hardware and not o.name.startswith(('S1 |','S2 |','Component | Battery','Battery |','E1 |','E-1 |')):o.hide_render=True
aim((320,460,350),(0,10,63),320);render('lower_assembled.png')
# Exploded drawing communicates parts, not a tested assembly trajectory.
for ob,vec in [(front,(0,-40,0)),(rear,(0,25,0)),(cap,(45,0,0)),(drawer,(0,70,0))]:ob.location+=Vector(vec)/100
for o in switches:o.hide_render=True
for o in hardware:o.hide_render=True
aim((350,480,440),(0,10,64),390);render('lower_exploded.png')
assert HASH(SOURCE)==source_hash
print('R17_BUILD_COMPLETE',flush=True)
