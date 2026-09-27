"""R15 measured sensor references and enclosing three-piece hood.
Blender background --disable-autoexec; -- --render for native CAD previews.
Previous releases are immutable. Scene unit = 100 mm. Never scales nut joints.
"""
import ast,csv,hashlib,json,math,shutil,sys
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/panel_bolts_r14';SOURCE=OLD/'roam_panel_bolts_r14.blend'
OLDPACK=OLD/'ROAM_STL_R14_Panel_Bolts_K1C'
OUT=ROOT/'mechanical/sensor_enclosure_r15';OUT.mkdir(parents=True,exist_ok=True)
PACK=OUT/'ROAM_STL_R15_Sensor_Enclosure_K1C';CHANGED=OUT/'ROAM_R15_hood_replacement'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
SOURCE_HASH=HASH(SOURCE);RENDER='--render' in sys.argv
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','mesh_object','prism','boolean','bounds','metrics','snapshot','intersection','envelope','ray_surface','move_collection'}),('create_tof_solder_relief_r08.py',{'export','aim','render','material'}),('create_panel_bolts_r14.py',{'freeze'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<CAD helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE));bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update()
ORIGINAL={o.name:snapshot(o) for o in S.objects}
CN='P15B | Raised removable hood side panel R15';FN='P15C | Enclosed camera front hood R15';RN='P15D | Raised rear hood R15'
NAMES={'P15B | Removable hood side panel R14':CN,'P15C | Front slide-on smiling hood R14':FN,'P15D | Rear slide-on hood R14':RN}
for a,b in NAMES.items():S.objects[a].name=b;freeze(S.objects[b])
cap,front,rear=[S.objects[n] for n in [CN,FN,RN]]
CHANGED_NAMES=set(NAMES);ADDED=[]
ROTOR_X=0.;ROTOR_Y=-8.10129;DIAMETER=70.26;PLATE_TOP=163.75
POST=44.78;BASE_TO_ROTOR=5.57;ROTOR_BOTTOM=PLATE_TOP+POST+BASE_TO_ROTOR
ROOF=217.9;RISE=ROOF-190.8;HOLE_R=DIAMETER/2+2.
CAM_W=161.;CAM_H=22.8;CAM_D=24.92;CAM_BACK=-79.225;CAM_FRONT=CAM_BACK-CAM_D
CAM_BOTTOM=150.25;CAM_Z=CAM_BOTTOM+CAM_H/2

def announce(label,*items):print('R15_'+label,*items,flush=True)
def paint(o,m):
 o.data.materials.clear();o.data.materials.append(m)
 for p in o.data.polygons:p.material_index=0
def exact(o,t,op='DIFFERENCE'):
 label=t.name
 result=boolean(o,t,op,solver='EXACT')
 m=metrics(o)
 if m['nonmanifold_edges']:announce('TOPOLOGY_WARNING',o.name,label,op,m)
 return result
def freeze_and_map(o,fn):
 freeze(o)
 for v in o.data.vertices:v.co=Vector(fn(v.co*100))/100
 clean(o)
def shift_z(o,d):o.matrix_world=Matrix.Translation(Vector((0,0,d))/100)@o.matrix_world

# Measured camera envelope, anchored at the existing rear support and bottom.
# Internal optical offsets are inherited visual references, not calibration data.
camera_names=[o.name for o in S.objects if o.type=='MESH' and (o.name.startswith(('RGB_','IR_Projector_','Depth_','Front_')) or o.name in ['Housing_Silver_Capsule','Rear_Black_Inset','Right_Small_Front_Aperture','Left_Small_Front_Aperture'])]
for name in camera_names:
 ob=S.objects[name]
 freeze_and_map(ob,lambda p:(p.x*CAM_W/180,CAM_BACK+(p.y-CAM_BACK)*CAM_D/28.495,CAM_BOTTOM+(p.z-CAM_BOTTOM)*CAM_H/23.5))
 ob['R15 reference']='Measured overall envelope 161 W x 22.80 H x 24.92 D. Optical detail positions inherited, not metrology.'
 CHANGED_NAMES.add(name)
# Accommodate the 0.70 mm lower camera top without moving the rigid clamps.
pad_names=[o.name for o in S.objects if o.name.startswith('P08 | Camera upper compliant pad')]
for name in pad_names:
 freeze_and_map(S.objects[name],lambda p:(p.x,p.y,174.25-(174.25-p.z)*1.2/.5));CHANGED_NAMES.add(name)
 S.objects[name]['R15 fit']='1.20 mm nominal compliant pad, replacing 0.50 mm; test actual foam compression.'

# Replace inaccurate imported mounting stack by separate measured references.
# Preserve the old sensor mesh hidden in an archive collection.
archive=bpy.data.collections.new('20 R15 superseded sensor reference - hidden');S.collection.children.link(archive)
refs=bpy.data.collections.new('21 R15 measured LiDAR reference - NOT FOR PRINT');S.collection.children.link(refs)
black=material('R15 LiDAR black',(.022,.027,.032));brass=material('R15 brass posts',(.43,.27,.075));green=material('R15 LiDAR PCB',(.025,.18,.08))
oldlidar=S.objects['Component | LIDAR'];CHANGED_NAMES.add(oldlidar.name)
me=bpy.data.meshes.new_from_object(oldlidar.evaluated_get(bpy.context.evaluated_depsgraph_get()));me.transform(oldlidar.matrix_world)
bm=bmesh.new();bm.from_mesh(me);pending=set(bm.verts);groups=[]
while pending:
 stack=[pending.pop()];g=[]
 while stack:
  v=stack.pop();g.append(v.index)
  for e in v.link_edges:
   q=e.other_vert(v)
   if q in pending:pending.remove(q);stack.append(q)
 groups.append(g)
bm.free()
rotor_group=next(g for g in groups if len(g)==74630)
kept=set(rotor_group)
# Keep the upper optical internals from the detailed source, but not its old
# spindle, belt plate or lower mount. They are not used as the rotating keepout.
for g in groups:
 if len(g)>2000:
  zmin=min(me.vertices[i].co.z*100 for i in g)
  if zmin>=200.8:kept.update(g)
verts=sorted(kept);lookup={i:k for k,i in enumerate(verts)}
mesh=bpy.data.meshes.new('R15 measured rotor');mesh.from_pydata([me.vertices[i].co for i in verts],[],[tuple(lookup[i] for i in p.vertices) for p in me.polygons if all(i in kept for i in p.vertices)]);mesh.update()
rotor=bpy.data.objects.new('R15 LiDAR | Rotating body 70.26 mm',mesh);refs.objects.link(rotor)
for v in rotor.data.vertices:
 p=v.co*100;v.co=Vector(((p.x-.00046)*DIAMETER/69.98132,ROTOR_Y+(p.y-ROTOR_Y)*DIAMETER/69.99989,ROTOR_BOTTOM+(p.z-201.75)))/100
paint(rotor,black);clean(rotor);ADDED.append(rotor.name)
oldlidar.hide_render=True;oldlidar.hide_set(True);move_collection(oldlidar,archive)
for name in [o.name for o in S.objects if o.name.startswith('P11.')]:
 ob=S.objects[name];ob.hide_render=True;ob.hide_set(True);move_collection(ob,archive);CHANGED_NAMES.add(name)
for name in [o.name for o in S.objects if o.name.startswith('G') and ('screw' in o.name or 'washer' in o.name)]:
 # Legacy hardware is only archived if located at a LiDAR foot, not a chassis joint.
 bb=bounds(S.objects[name])
 if bb[2][0]>155 and bb[0][0]>-40 and bb[0][1]<40:
  o=S.objects[name];o.hide_render=True;o.hide_set(True);move_collection(o,archive);CHANGED_NAMES.add(name)
def reference(o,mat,description):
 move_collection(o,refs);paint(o,mat);o['Reference only']=description;ADDED.append(o.name);return o
feet=[(-28,-36.10129),(28,-36.10129),(-20,33.89871),(20,33.89871)]
for i,(x,y) in enumerate(feet,1):reference(cyl(f'R15 LiDAR | Brass post {i} 44.78 mm',(x,y,PLATE_TOP+POST/2),2.5/math.cos(math.pi/6),POST,6),brass,'Measured body length. Foot XY reference retained from old model; 5 mm AF approximate; thread details unverified.')
# Stationary outline retained from the old A1 base envelope; underside details
# deliberately simplified. Measured datum is its bottom-to-rotor distance.
base=cyl('_base round',(0,ROTOR_Y,PLATE_TOP+POST+1.9),35.9,3.8,128)
boolean(base,box('_tail',(0,26,PLATE_TOP+POST+1.9),(46,46,3.8)),'UNION')
for x,y in feet:boolean(base,cyl('_base corner',(x,y,PLATE_TOP+POST+1.9),5,3.8,32),'UNION')
base.name='R15 LiDAR | Fixed base';reference(base,black,'3.8 mm plate and underside details approximate; top bolts kept below measured rotating-body bottom.')
reference(box('R15 LiDAR | Underside PCB',(0,0,PLATE_TOP+POST-1.5),(58,63,1.6)),green,'Simplified visual/clearance reference, not measured PCB layout.')
reference(cyl('R15 LiDAR | Fixed rear motor',(0,36,PLATE_TOP+POST-11),15.8,21,64),black,'Conservative inherited motor envelope. Cable shape not modeled.')
reference(cyl('R15 LiDAR | Lower rotating rim',(0,ROTOR_Y,ROTOR_BOTTOM-.55),DIAMETER/2,.9,128),black,'Inferred transition rim under measured rotating cylinder; allow full radial clearance.')
for i,(x,y) in enumerate(feet,1):reference(cyl(f'R15 LiDAR | Base screw head {i}',(x,y,PLATE_TOP+POST+4.55),2.4,1.5,32),bpy.data.materials['Machined metal'],'Visual only: thread/engagement unverified; reuse actual sensor hardware.')
bpy.data.meshes.remove(me)

# Fill the old oversized camera window with matching shell material before
# extending the upper walls. Original mounting holes and lower shell stay put.
rings=[(113,102,102,0,.5),(113,102,107,0,.5),(111,101,157,0,.5),(109,99,176,0,.5),(105,94,185,0,.5),(97,85,190,0,.5),(52,52,191,5,1)]
patch=envelope(rings)
inner=envelope([(w-3.0,d-3.0,z,cy,p) for w,d,z,cy,p in rings]);exact(patch,inner)
exact(patch,box('_old window fill extent',(0,-111,163),(218,102,50)),'INTERSECT')
for target,is_cap in [(front,False),(cap,True)]:
 p=clone(patch,'_window skin')
 exact(p,box('_window ownership',((95.15+250) if is_cap else (94.85-250),0,160),(500,600,200)),'INTERSECT')
 exact(target,p,'UNION')
remove(patch)
# Restore the original H7 receiver immediately after filling the old window.
# Its upper corner intersects the fill skin; later camera cuts do not touch it.
h7=next(j for j in json.loads((OLD/'geometry_checks.json').read_text())['panel_joints'] if j['id']=='H7')
t=box('_restore H7 receiver',(-1,0,6.4),(24.6,14.6,5.6));bpy.context.view_layer.update()
t.data.transform(Matrix(h7['local_to_world_matrix'])@t.matrix_world);t.matrix_world=Matrix.Identity(4);clean(t);boolean(front,t)
# Insert wall height below intact roof details: joint pockets translate rigidly.
for o in [front,rear,cap]:
 for v in o.data.vertices:
  z=v.co.z*100;t=max(0,min(1,(z-164)/12));v.co.z=(z+RISE*t)/100
 clean(o)
for o in S.objects:
 if o.name.startswith(('H1 |','H2 |','H3 |','R10 H4 |','P14 |')):
  shift_z(o,RISE);CHANGED_NAMES.add(o.name)

# Close the old large LiDAR opening with a planar insert. Locally jog the roof
# seam through the rotor centre so both halves can slide horizontally free.
for o in [front,rear]:
 # Remove central roof where the new joint allocation replaces it.
 exact(o,box('_central roof replacement',(0,0,230),(120,140,60)))
disc=cyl('_flat roof annulus',(ROTOR_X,ROTOR_Y,ROOF-1.4),65,2.8,192)
exact(disc,cyl('_rotor clearance',(ROTOR_X,ROTOR_Y,ROOF),HOLE_R,60,192))
# Rectangular insert gives a complete roof beside the circular opening and
# bridges the former Y=10 seam without leaving a gap between separate solids.
slab=box('_roof central slab',(0,0,ROOF-1.4),(119.6,140,2.8))
exact(slab,cyl('_rotor clearance',(ROTOR_X,ROTOR_Y,ROOF),HOLE_R,60,192))
exact(disc,slab,'UNION')
for target,forward in [(front,True),(rear,False)]:
 p=clone(disc,'_split roof insert')
 edge=ROTOR_Y+(-.2 if forward else .2)
 exact(p,box('_new roof seam',(0,edge+(-200 if forward else 200),ROOF),(400,400,20)),'INTERSECT')
 # Trim the overlap annulus outside the replaced 120 mm strip back to each
 # original fore/aft ownership. The 0.2 mm side jog clearance prevents binding.
 for sign in [-1,1]:
  exact(p,box('_outer ownership',(sign*159.8,(10.2-200 if not forward else 9.8+200),ROOF),(200,400,20)))
 exact(target,p,'UNION')
remove(disc)

def rounded_rect(w,h,r,n=16):
 pts=[]
 for cx,cz,start in [(w/2-r,h/2-r,0),(-w/2+r,h/2-r,90),(-w/2+r,-h/2+r,180),(w/2-r,-h/2+r,270)]:
  for j in range(n+1):
   a=math.radians(start+j*90/n);pts.append((cx+r*math.cos(a),cz+r*math.sin(a)))
 return pts
def camera_loft(name,sections):
 # Each section: Y, width, height, corner radius; vertical centre fixed.
 verts=[]
 for y,w,h,r in sections:verts += [(x,y,CAM_Z+z) for x,z in rounded_rect(w,h,r)]
 n=len(rounded_rect(*sections[0][1:]));faces=[tuple(reversed(range(n))),tuple(range((len(sections)-1)*n,len(sections)*n))]
 faces += [(k*n+i,k*n+(i+1)%n,(k+1)*n+(i+1)%n,(k+1)*n+i) for k in range(len(sections)-1) for i in range(n)]
 assert all(0<=i<len(verts) for f in faces for i in f)
 return mesh_object(name,verts,faces)
# Bezel blends back into the existing rounded front, leaving the measured
# face exposed. Inner flare clears camera clamps and avoids an optical tunnel.
bezel=camera_loft('_camera surround',[(CAM_FRONT+.15,182,35.2,12),(-91,198,52,15)])
exact(bezel,box('_respect removable side seam',(-155.15,0,160),(500,600,200)),'INTERSECT')
exact(front,bezel,'UNION')
clearance=camera_loft('_camera optical mouth',[(CAM_FRONT-20,162.2,24,11.8),(CAM_FRONT+.16,162.2,24,11.8),(-100.8,165.2,28,12),(-97,178,37,13),(-75,184,60,15)])
exact(front,clearance)
# Remove hidden excess surround material behind the original skin, maintaining
# a 2.6 mm front wall except the outward-flared mouth.
inner_bezel=camera_loft('_surround back cavity',[(CAM_FRONT+2.75,178,30,10),(-88,184,48,13),(-70,184,60,13)])
exact(front,inner_bezel)
boolean(front,box('_rear camera bracket clearance',(0,-86.5,145),(44,23,32)))
for side in [-1,1]:
 boolean(front,box('_camera lower ledge clearance',(side*60,-86.5,143.5),(22,25,5)))
# Join the bezel's right cut edge back to the curved original skin. Without
# this return wall the side seam looks closed from the front but exposes a
# narrow diagonal view into the robot. It stays wholly on the front's side.
boolean(front,box('_camera bezel right return wall',(93.425,-86.5,163),(2.85,11,40)),'UNION')

for o in [front,rear,cap]:
 clean(o);m=metrics(o);announce('SOLID',o.name,json.dumps(m))
 assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0,m
 o['R15 design']='Raised flat roof; measured sensor envelopes; retain all R14 bolted nut-retaining joints.'
for a,b in [(front,rear),(front,cap),(rear,cap)]:
 v=intersection(a,b);announce('HOOD_OVERLAP',a.name,b.name,v)
 if v>=.03:
  t=clone(a,'_QA overlap');exact(t,clone(b,'_QA other'),'INTERSECT');announce('OVERLAP_BOUNDS',bounds(t));remove(t)
 assert v<.03
S['Mechanical revision']='R15 measured LiDAR/camera, raised three-piece enclosed sensor hood'
S['Revision']=S['Mechanical revision'];S['R15 print release']='Pending independent validation and physical fit samples.'
report={'revision':'R15','source_R14_sha256':SOURCE_HASH,'renamed_hood_objects':NAMES,'changed_original_objects':sorted(CHANGED_NAMES),'added_reference_objects':ADDED,
 'measurements':{'lidar_post_mm':POST,'lidar_post_to_rotor_mm':BASE_TO_ROTOR,'lidar_rotor_diameter_mm':DIAMETER,'camera_width_mm':CAM_W,'camera_height_mm':CAM_H,'camera_depth_mm':CAM_D},
 'datums':{'lidar_support_plate_top_z_mm':PLATE_TOP,'rotor_bottom_z_mm':ROTOR_BOTTOM,'roof_z_mm':ROOF,'roof_rise_mm':RISE,'roof_insert_thickness_mm':2.8,'aperture_radius_mm':HOLE_R,'aperture_centre_mm':[ROTOR_X,ROTOR_Y],'local_roof_seam_y_mm':ROTOR_Y,'camera_front_y_mm':CAM_FRONT,'camera_rear_y_mm':CAM_BACK,'camera_bottom_z_mm':CAM_BOTTOM},
 'assumptions':['Measured camera dimensions treated as maximum body/optical envelope, anchored to existing rear and lower supports.','LiDAR post footprint inherited from old reference; P11 printed spacers omitted because measured brass posts stand directly on P10.','Rotor height/optical details inherited visual reference; fixed-base thickness subdivisions, underside PCB, motor and screw details approximate.','No calibration/URDF changes; actual sensor scan height should be measured before updating software transforms.']}
unexpected=[n for n,h in ORIGINAL.items() if n not in CHANGED_NAMES and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected;report['unexpected_original_changes']=unexpected
prior=json.loads((OLD/'geometry_checks.json').read_text())
for key in ['nut_retention','roof_joint','side_joints','panel_joints']:report[key]=prior[key]
for j in report['nut_retention']:
 j['part']=NAMES.get(j['part'],j['part'])
 if j['id'] in ['H1','H2','H3','H4']:j['local_to_world_matrix'][2][3]+=RISE/100
report['roof_joint']['seat_mm'][2]+=RISE
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
S.render.engine='CYCLES';S.cycles.samples=32;S.cycles.device='CPU';S.render.resolution_x=1600;S.render.resolution_y=1200;S.render.resolution_percentage=100
aim((370,-490,360),(0,0,163),340)
bpy.ops.object.select_all(action='DESELECT');front.select_set(True);bpy.context.view_layer.objects.active=front
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_sensor_enclosure_r15.blend'),compress=True)
render('hood_assembled.png')
announce('GEOMETRY_BUILT',json.dumps(report['datums']))
