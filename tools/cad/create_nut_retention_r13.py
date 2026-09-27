"""R13 PETG press-in M3 nut retention, derived only from immutable R12.

Blender --background --disable-autoexec --python THIS.py -- [--render]
Ribs deliberately interfere with the nominal nuts; physical fit is unverified.
All authored geometry is millimetres, converted to the existing 100 mm/BU scene.
"""
import ast, csv, hashlib, json, math, shutil, sys
from pathlib import Path
import bpy, bmesh
from mathutils import Matrix, Vector

ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/bolted_sides_r12'
SOURCE=OLD/'roam_bolted_sides_r12.blend'
OLDPACK=OLD/'ROAM_STL_R12_Bolted_Sides_K1C'
OUT=ROOT/'mechanical/nut_retention_r13'
PACK=OUT/'ROAM_STL_R13_Nut_Retention_K1C'
CHANGED=OUT/'ROAM_R13_hood_replacement'
OUT.mkdir(parents=True,exist_ok=True)
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
SOURCE_HASH=HASH(SOURCE)
RENDER='--render' in sys.argv
PROFILES={'light':5.4,'standard':5.3,'firm':5.2}
PROFILE='standard'
if '--fit-profile' in sys.argv: PROFILE=sys.argv[sys.argv.index('--fit-profile')+1]
assert PROFILE in PROFILES
BN='P15B | Removable hood side panel R13'
DN='P15D | Rear slide-on hood R13'
FN='P15C | Front slide-on smiling hood R12'
for file,names in [
 ('create_split_shell_r07.py',{'clean','remove','clone','box','cyl','mesh_object','boolean','bounds','metrics','snapshot','intersection'}),
 ('create_tof_solder_relief_r08.py',{'export','aim','render','material'})]:
 for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(node,ast.FunctionDef) and node.name in names:
   exec(compile(ast.Module(body=[node],type_ignores=[]),'<CAD helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update()
assert 'R12' in S['Mechanical revision'] and not S.get('Exploded view')
ORIGINAL={o.name:snapshot(o) for o in S.objects}
NAMES={'P15B | Removable hood side panel R07':BN,'P15D | Rear slide-on hood R12':DN}
for a,b in NAMES.items():S.objects[a].name=b
oldparts={n:clone(S.objects[n],'_unchanged R12 '+n) for n in [BN,DN]}
for n in [BN,DN]:
 # Freeze Blender's ORIGINAL evaluated tessellation. R12 contains a few
 # non-planar n-gons: allowing a Boolean to re-triangulate those would change
 # the old surface away from the new nut features.
 old=oldparts[n];source_object=S.objects[n]
 source_mesh=bpy.data.meshes.new_from_object(source_object.evaluated_get(bpy.context.evaluated_depsgraph_get()))
 source_mesh.transform(source_object.matrix_world);source_mesh.calc_loop_triangles()
 tris=[tuple(t.vertices) for t in source_mesh.loop_triangles]
 settings=[(source_mesh.polygons[t.polygon_index].material_index,source_mesh.polygons[t.polygon_index].use_smooth) for t in source_mesh.loop_triangles]
 me=bpy.data.meshes.new(n+' original tessellation');me.from_pydata([v.co[:] for v in source_mesh.vertices],[],tris);me.update()
 for m in source_mesh.materials:me.materials.append(m)
 for p,(index,smooth) in zip(me.polygons,settings):p.material_index=index;p.use_smooth=smooth
 old.data=me
 bpy.data.meshes.remove(source_mesh)
 # Bake the evaluated surface before booleans, exactly as for STL export.
 ob=S.objects[n];ob.data=oldparts[n].data.copy();ob.modifiers.clear();ob.matrix_world=Matrix.Identity(4)
prior=json.loads((OLD/'geometry_checks.json').read_text())
oldmanifest=json.loads((OLDPACK/'manifest.json').read_text())
priorrows={r['id']:r for r in oldmanifest['parts']}
roofrot=Matrix.Rotation(math.pi,4,'X')
caprot=Matrix(priorrows['P15B']['export_rotation_matrix'])

def frame(jid):
 name=(jid if jid in ['H1','H2','H3'] else ('R10 '+jid if jid=='H4' else 'R12 '+jid))+' | M3 captive nut'
 bb=bounds(S.objects[name]);c=Vector([(a+b)/2 for a,b in bb])
 if jid in ['H1','H2','H3']:
  u,v,w=Vector((1,0,0)),Vector((0,-1,0)),Vector((0,0,-1))
 elif jid=='H4':u,v,w=Vector((0,1,0)),Vector((-1,0,0)),Vector((0,0,-1))
 else:u,v,w=Vector((0,0,-1)),Vector((0,1,0)),Vector((1,0,0))
 origin=c-w*(1.3 if jid in ['H1','H2','H3','H4'] else 1.2)
 m=Matrix(((u.x,v.x,w.x,origin.x/100),(u.y,v.y,w.y,origin.y/100),
           (u.z,v.z,w.z,origin.z/100),(0,0,0,1)))
 return name,m

def ribs(ob,mat,tip_af):
 """Six tapered trapezoidal crush ribs; no undercut or separate retainer.

 Pocket flat radius 2.95. Root embeds 0.15 into the existing wall, base width
 1.2, tip width .20. Ridge stops .05 above floor; .80 axial lead-in reaches
 the original wall at the mouth. They compress locally when the nut enters.
 """
 for i in range(6):
  # Blender's six-vertex cylinder starts at +Y: flat normals are 0,60,...
  angle=i*math.tau/6
  n=Vector((math.cos(angle),math.sin(angle),0));t=Vector((-n.y,n.x,0))
  verts=[]
  # End .01 mm behind the old flat at the mouth: avoids coincident zero-area
  # boolean slivers. The last ~.03 mm of the entrance is simply the old hex.
  for z,tip in [(.05,tip_af/2),(2.,tip_af/2),(2.8,2.96)]:
   for r,s in [(3.10,-.60),(3.10,.60),(tip,.10),(tip,-.10)]:
    p=n*r+t*s+Vector((0,0,z));verts.append(tuple(mat@(p/100)*100))
  faces=[(3,2,1,0),(8,9,10,11)]
  faces += [(4*k+j,4*k+(j+1)%4,4*(k+1)+(j+1)%4,4*(k+1)+j) for k in range(2) for j in range(4)]
  boolean(ob,mesh_object('_integral press rib',verts,faces),'UNION',solver='EXACT')
  bm=bmesh.new();bm.from_mesh(ob.data)
  bmesh.ops.triangulate(bm,faces=list(bm.faces))
  inv=mat.inverted()
  local_vertices=[v for v in bm.verts if (lambda p:p.x*p.x+p.y*p.y<3.6**2 and -.1<p.z<2.9)(inv@v.co*100)]
  bmesh.ops.remove_doubles(bm,verts=local_vertices,dist=.005/100)
  bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=.0001/100)
  bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
  bm.to_mesh(ob.data);bm.free();ob.data.update()

joints=[]
for jid in ['H1','H2','H3','H4','H5','H6']:
 nut,mat=frame(jid);ob=S.objects[BN if jid in ['H1','H2','H3'] else DN]
 # Verify the inferred coordinate frame against the actual R12 pocket walls.
 before=[]
 for i in range(6):
  a=i*math.tau/6;n=Vector((math.cos(a),math.sin(a),0))
  o=mat@(Vector((0,0,1.2))/100);d=mat.to_3x3()@n
  hit,p,_,_=ob.ray_cast(o,d);assert hit
  before.append((p-o).length*100)
 assert all(abs(d-2.95)<.003 for d in before),(jid,before)
 ribs(ob,mat,PROFILES[PROFILE])
 joints.append({'id':jid,'part':ob.name,'nut_object':nut,'local_to_world_matrix':[list(r) for r in mat],
  'original_flat_radii_mm':before,'pocket_AF_mm':5.9,'pocket_depth_mm':2.8,
  'nominal_nut_AF_mm':5.5,'nominal_nut_thickness_mm':2.4,'rib_tip_AF_mm':PROFILES[PROFILE],
  'nominal_radial_interference_mm':(5.5-PROFILES[PROFILE])/2,
  'rib_count':6,'rib_base_width_mm':1.2,'rib_tip_width_mm':.2,'rib_leadin_depth_mm':.8,
  'floor_clearance_mm':.05,'profile':PROFILE})
 print('R13_POCKET',jid,json.dumps(joints[-1]),flush=True)
for n in [BN,DN]:
 ob=S.objects[n];m=metrics(ob)
 if m['nonmanifold_edges']:
  bm=bmesh.new();bm.from_mesh(ob.data)
  print('R13_BAD_EDGES',n,[(len(e.link_faces),[list(v.co*100) for v in e.verts]) for e in bm.edges if not e.is_manifold],flush=True);bm.free()
 assert m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0,m
 ob['R13 nut retention']='Six tapered integral press-fit ribs per hood nut pocket. PETG prototype; fit Q13 coupons first.'

records=[];changed=[]
# Keep old receiver coupons, but supersede open nut-pocket coupons by Q13.
superseded={'Q07B','Q11D','Q12D'}
for row in oldmanifest['parts']:
 if row['id'] in {'P15B','P15D'}|superseded:continue
 row=dict(row)
 if row['id'] in {'Q07A','Q11C','Q12C'}:
  row['note']+=' R13: unchanged receiver sample; pair with the matching Q13 standard nut-bearing sample.'
 path=PACK/row['file'];path.parent.mkdir(parents=True,exist_ok=True)
 shutil.copy2(OLDPACK/row['file'],path);assert HASH(path)==row['sha256'];records.append(row)

def emit(pid,label,ob,rot,folder,note):
 path=f'{folder}/{pid}_{label}_qty1.stl';m=export(ob,PACK/path,rot)
 row={'id':pid,'label':label,'quantity':1,'sources':[ob.name],'file':path,'folder':folder,
  'note':note+' 100%, millimetres; keep supplied orientation.',
  'support':'Use the same PETG profile as the cover. Inspect local supports at laps, bore and pocket; clear support without cutting off the retention ribs.',
  'reimport_check':m,'sha256':HASH(PACK/path),'export_rotation_matrix':[list(r) for r in rot],
  'brim_4mm_fits_K1C':all(d+8<220 for d in m['dimensions_mm'][:2])}
 records.append(row);changed.append(row)
 dest=CHANGED/path;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/path,dest)
 print('R13_EXPORT',pid,json.dumps(m),flush=True)

emit('P15B','hood_side_panel_retained_nuts',S.objects[BN],caprot,'01_rigid','R13 H1-H3 press-in nut pockets; no other change to panel.')
emit('P15D','rear_hood_retained_nuts',S.objects[DN],roofrot,'01_rigid','R13 H4-H6 press-in nut pockets; flat roof and 5 mm side laps unchanged.')
coupons=[]
# These are exact cropped joint solids (same crop as the previous tests), not
# idealized horizontal test blocks: each keeps its cover's layer orientation.
specs=[('B','H1',BN,(94,-35,187),(26,22,14),caprot,'Q07A'),
       ('R','H4',DN,(-85,9,186),(20,28,18),roofrot,'Q11C'),
       ('D','H5',DN,(-108,8,118),(30,36,26),roofrot,'Q12C')]
for code,jid,part,pos,dims,rot,mate in specs:
 j=next(j for j in joints if j['id']==jid);mat=Matrix(j['local_to_world_matrix'])
 for profile,af in PROFILES.items():
  q=clone(oldparts[part],f'Q13{code}_{profile} | {jid} actual nut retention coupon')
  boolean(q,box('_crop',pos,dims),'INTERSECT');ribs(q,mat,af)
  emit(f'Q13{code}_{profile}',f'{jid}_nut_fit_{profile}',q,rot,'04_fit_test',
       f'Actual {jid} nut-bearing joint sample, {profile} grip, ridge tip AF {af:.2f} mm. Optional; not installed. Pair with {mate}. Standard matches delivered covers.')
  coupons.append(q)
for o in coupons+list(oldparts.values()):remove(o)
unexpected=[n for n,h in ORIGINAL.items() if n not in NAMES and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
manifest={**oldmanifest,'revision':'R13 PETG press-in nut retention at H1-H6',
 'source':'roam_nut_retention_r13.blend','parts':records,'unique_stl_files':len(records),
 'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),
 'replaces':['Only P15B and P15D change from R12. P15C and the rest of the robot remain unchanged.'],
 'unchanged_parts_source':str(OLDPACK),'hardware_change_from_R12':'None. Reuse all R12 standard M3 nuts, bolts and washers.',
 'new_optional_coupons':[r['id'] for r in changed if r['folder']=='04_fit_test'],
 'superseded_nut_coupons':sorted(superseded),'retention_profile':PROFILE,'retention_profiles_tip_AF_mm':PROFILES,
 'nut_retention':joints}
manifest.pop('hardware_change_from_R11',None)
for target,rows in [(PACK,records),(CHANGED,changed)]:
 (target/'manifest.json').write_text(json.dumps({**manifest,'parts':rows,'unique_stl_files':len(rows),
   'total_physical_pieces_including_optional':sum(r['quantity'] for r in rows)},indent=2))
 with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
  wr=csv.writer(f);wr.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
  for r in rows:wr.writerow([r['id'],r['label'],r['quantity'],r['file'],*r['reimport_check']['dimensions_mm'],r['support'],r['note']])
shutil.copy2(OLDPACK/'HARDWARE.csv',PACK/'HARDWARE.csv')
report={'revision':'R13','source_R12_sha256':SOURCE_HASH,'renamed_hood_objects':NAMES,
 'nut_retention':joints,'roof_joint':prior['roof_joint'],'side_joints':prior['side_joints'],
 'roof_plane_mm':190.8,'unexpected_original_changes':unexpected,'unchanged_original_object_count':len(ORIGINAL)-2,
 'intentional_interference':'The uncompressed rib tips intersect nominal metal nut flats by 0.10 mm per flat (standard). This is the retaining press fit, not clearance.',
 'physical_retention_test':'Not performed. PETG coupon fitting is required; retention force and lifetime are not certified.',
 'superseded_coupon_ids':sorted(superseded),'profile':PROFILE}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
S['Mechanical revision']='R13 PETG retained hood nuts; R12 bolted sides; R11 flat roof and R09 measured ToF retained'
S['Revision']=S['Mechanical revision'];S['Geometry checks']='nut_retention_r13/geometry_checks.json and validation.json'
S['R13 nut retention']='H1-H6 press-fit ribs; physical coupon fit required. No adhesive, new hardware, or external change.'
bpy.ops.object.select_all(action='DESELECT');S.objects[DN].select_set(True);bpy.context.view_layer.objects.active=S.objects[DN]
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_nut_retention_r13.blend'),compress=True)

# Native CAD renders, not slicer toolpaths. Keep the delivered full master intact.
S.render.engine='CYCLES';S.cycles.samples=32;S.cycles.device='CPU'
S.render.resolution_x=1500;S.render.resolution_y=1050;S.render.resolution_percentage=100
aim((-360,-480,355),(0,0,140),330);render('hood_assembled.png')
keep={BN,DN}
for ob in S.objects:
 if ob.type not in {'LIGHT','CAMERA'} and ob.name not in keep:ob.hide_render=True;ob.hide_set(True)
aim((35,-58,145),(-105,2,136),70)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_nut_retention_r13_pockets_inspection.blend'),compress=True)
render('retained_side_pockets.png')

# Magnified actual H5 sample, open nut recess facing camera; compare old/new.
for ob in list(S.objects):
 if ob.type not in ['LIGHT','CAMERA']:remove(ob)
white=material('R13 label',(.82,.9,.94));teal=material('R13 PETG preview',(.04,.32,.38))
grey=material('R12 reference',(.32,.36,.40))
j=next(j for j in joints if j['id']=='H5');mat=Matrix(j['local_to_world_matrix'])
for source,offset,paint,caption in [(OLDPACK/priorrows['Q12D']['file'],-19,grey,'R12 / open pocket'),
 (PACK/next(r['file'] for r in changed if r['id']=='Q13D_standard'),19,teal,'R13 / press-in ribs')]:
 bpy.ops.wm.stl_import(filepath=str(source),use_facet_normal=False)
 ob=bpy.context.object;ob.data.transform(Matrix.Scale(.01,4))
 # Export-centering is immaterial here: rotate each identical sample face-up,
 # then center its actual pocket using known crop extrema and export transform.
 ob.data.transform(Matrix.Rotation(-math.pi/2,4,'Y'))
 bpy.context.view_layer.update()
 bb=bounds(ob);ob.location=Vector((offset-(bb[0][0]+bb[0][1])/2,-(bb[1][0]+bb[1][1])/2,-bb[2][0]))/100
 ob.data.materials.clear();ob.data.materials.append(paint)
 curve=bpy.data.curves.new(caption,'FONT');curve.body=caption;curve.size=.016;curve.align_x='CENTER'
 lab=bpy.data.objects.new(caption,curve);S.collection.objects.link(lab);lab.location=Vector((offset,-21,0))/100;curve.materials.append(white)
S['Exploded view']='Magnified print-coupon comparison. No supports illustrated.'
aim((0,-60,100),(0,0,0),84)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_nut_retention_r13_comparison.blend'),compress=True)
render('nut_retention_comparison.png')
assert HASH(SOURCE)==SOURCE_HASH
print('R13_BUILD_COMPLETE',json.dumps({'new_STLs':len(changed),'unchanged_STLs':len(records)-len(changed),'superseded_coupons':sorted(superseded)}),flush=True)
