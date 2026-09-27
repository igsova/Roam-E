"""R16 local camera-surround closure; preserve the released R15 source.
Run in Blender background --disable-autoexec [-- --render].
Only P15C is modified. Wall additions stop within the original hood skin.
"""
import ast,hashlib,json,math,sys
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/sensor_enclosure_r15';OUT=ROOT/'mechanical/camera_closure_r16';OUT.mkdir(parents=True,exist_ok=True)
SOURCE=OLD/'roam_sensor_enclosure_r15.blend';HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();RENDER='--render' in sys.argv
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','box','mesh_object','boolean','bounds','metrics','snapshot','intersection','envelope'}),('create_tof_solder_relief_r08.py',{'aim','render'}),('create_sensor_enclosure_r15.py',{'rounded_rect','camera_loft'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<CAD helper>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(SOURCE));S=bpy.context.scene;bpy.context.preferences.filepaths.save_version=0;bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in S.objects};source_hash=HASH(SOURCE)
OLD_FN='P15C | Enclosed camera front hood R15';FN='P15C | Closed camera surround front hood R16'
CN='P15B | Raised removable hood side panel R15';RN='P15D | Raised rear hood R15'
front=S.objects[OLD_FN]
# Freeze Blender's existing tessellation before cutting its few non-planar
# polygons. Otherwise Boolean recomputation can choose a different diagonal.
old_mesh=front.data;old_mesh.calc_loop_triangles()
tris=list(old_mesh.loop_triangles)
tri_mesh=bpy.data.meshes.new('R16 preserved R15 surface triangles')
tri_mesh.from_pydata([v.co for v in old_mesh.vertices],[],[tuple(t.vertices) for t in tris]);tri_mesh.update()
for mat in old_mesh.materials:tri_mesh.materials.append(mat)
for p,t in zip(tri_mesh.polygons,tris):
 source_face=old_mesh.polygons[t.polygon_index];p.material_index=source_face.material_index;p.use_smooth=source_face.use_smooth
front.data=tri_mesh
before=clone(front,'_R15 front reference');front.name=FN
CAM_Z=161.65
# Continue the original rear bezel section into the curved hood. The hollow
# sleeve overlaps the old bezel by 0.3 mm, maintains its outer silhouette and
# is clipped to the front cover's ownership at the removable panel seam.
closure=camera_loft('_continuous camera return', [(-91.30,198,52,15),(-55,198,52,15)])
inner=camera_loft('_return hollow', [(-92,184,46,12),(-54,184,46,12)])
boolean(closure,inner,'DIFFERENCE')
boolean(closure,box('_P15C seam boundary',(-155.15,0,160),(500,600,200)),'INTERSECT')
# Clip additions inside the original enclosure cavity. This avoids extending
# the new sleeve into the camera ledges, rear bracket or wiring volume.
rings=[(113,102,102,0,.5),(113,102,107,0,.5),(111,101,157,0,.5),(109,99,176,0,.5),(105,94,185,0,.5),(97,85,190,0,.5),(52,52,191,5,1)]
room=envelope([(w-2,d-2,z,cy,p) for w,d,z,cy,p in rings])
for v in room.data.vertices:
 z=v.co.z*100;t=max(0,min(1,(z-164)/12));v.co.z=(z+27.1*t)/100
clean(room);boolean(closure,room)
# The lower right return meets the existing H7 lap. Keep its receiver empty
# so the retained side panel can still slide in; that overlap closes when the
# side panel is fitted. Do not recut or alter the original receiver itself.
h7=next(j for j in json.loads((OLD/'geometry_checks.json').read_text())['panel_joints'] if j['id']=='H7')
t=box('_keep H7 receiver clear',(-1,0,6.4),(24.6,14.6,5.6));bpy.context.view_layer.update()
t.data.transform(Matrix(h7['local_to_world_matrix'])@t.matrix_world);t.matrix_world=Matrix.Identity(4);clean(t);boolean(closure,t)
print('R16_ADDITION',json.dumps(metrics(closure)),bounds(closure),flush=True)
addition_bounds=bounds(closure);source_volume=metrics(before)['volume_mm3']
boolean(front,closure,'UNION',solver='EXACT');clean(front)
# Boolean face unions can leave zero-area wire remnants at coincident trim
# boundaries. Remove only vertices/edges with no incident faces, never faces.
bm=bmesh.new();bm.from_mesh(front.data)
loose=[e for e in bm.edges if not e.link_faces]
if loose:bmesh.ops.delete(bm,geom=loose,context='EDGES')
loose=[v for v in bm.verts if not v.link_faces]
if loose:bmesh.ops.delete(bm,geom=loose,context='VERTS')
bm.to_mesh(front.data);bm.free();front.data.update()
checks=metrics(front);assert checks['components']==1 and not checks['nonmanifold_edges'] and not checks['loose_vertices'],checks
# A Boolean subtraction of two nearly identical meshes is unreliable around
# coincident triangulated faces. Independently sample their actual surfaces:
# surface samples outside the local edit bounds must remain coincident. Local
# intersections may be retriangulated/reshaped by the closure and are checked
# by end-coverage, solid, hardware and motion tests instead of a volume proxy.
def surface(o):
 o.data.calc_loop_triangles();v=[o.matrix_world@p.co*100 for p in o.data.vertices];f=[tuple(t.vertices) for t in o.data.loop_triangles]
 samples=v+[sum((v[i] for i in tri),Vector())/3 for tri in f]
 return BVHTree.FromPolygons(v,f,all_triangles=True),samples
old_tree,old_samples=surface(before);new_tree,new_samples=surface(front)
outside=[];local_changes=[]
for p in old_samples:
 nearest,n,_,dist=new_tree.find_nearest(p)
 if dist>.025 and (p-nearest).dot(n)>.025:
  item=(list(p),dist)
  if all(lo-.1<=p[i]<=hi+.1 for i,(lo,hi) in enumerate(addition_bounds)):local_changes.append(item)
  else:outside.append(item)
assert not outside,outside[:10]
errors=[];error_locations=[]
for p in new_samples:
 if not all(lo-.1<=p[i]<=hi+.1 for i,(lo,hi) in enumerate(addition_bounds)):
  e=old_tree.find_nearest(p)[3];errors.append(e)
  if e>.04:error_locations.append((list(p),e))
# Preserve the existing polygonal cavity within 0.06 mm. The report exposes
# the measured sub-0.051 mm retriangulation at its lower rear mouth edge; all
# exact opening, joint and assembly clearances are checked independently.
assert max(errors)<.06,error_locations[:12]
surface_check={'old_surface_sample_count':len(old_samples),'old_outside_edit_region_samples_lost':len(outside),'local_surface_retriangulation_samples':local_changes,'new_samples_outside_edit_region':len(errors),'max_outside_region_surface_error_mm':max(errors),'samples_exceeding_0p04mm':error_locations,'method':'Nearest oriented surface; vertices and triangle centroids outside the local edit box. 0.025 mm old-surface / 0.06 mm new-surface tolerance. Local edits independently tested for end coverage and collisions. Sampled, not exact volume containment.'}
added_volume=checks['volume_mm3']-source_volume;assert 0<added_volume<7000,added_volume
remove(before)
for name in [CN,RN]:
 overlap=intersection(front,S.objects[name]);print('R16_PAIR_OVERLAP',name,overlap,flush=True);assert overlap<.03,(name,overlap)
unchanged=[n for n,h in original.items() if n!=OLD_FN and snapshot(S.objects[n])!=h];assert not unchanged,unchanged
front['R16 correction']='Continuous camera bezel return walls into curved hood, both ends. Only P15C changes; preserves camera opening and all joints.'
S['Mechanical revision']='R16 camera-surround closure; R15 measured sensors and bolted three-piece hood retained';S['Revision']=S['Mechanical revision']
S['R15 print release']='Superseded by R16 camera closure. See validation.json; physical fit testing remains required.'
report=json.loads((OLD/'geometry_checks.json').read_text())
report.update({'revision':'R16','source_R15_sha256':source_hash,'source_R15_validation_sha256':HASH(OLD/'validation.json'),'renamed_from_R15':{OLD_FN:FN},'only_changed_R15_geometry':OLD_FN,'R16_net_added_volume_mm3':added_volume,'R16_addition_bounds_mm':addition_bounds,'R16_surface_preservation':surface_check,'R16_front_solid':checks,'R16_unchanged_object_count':len(original)-1,'R16_unexpected_original_changes':unchanged})
report['renamed_hood_objects']['P15C | Front slide-on smiling hood R14']=FN
for j in report['nut_retention']:j['part']=FN if j['part']==OLD_FN else j['part']
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
bpy.ops.object.select_all(action='DESELECT');front.select_set(True);bpy.context.view_layer.objects.active=front
aim((370,-490,360),(0,0,163),340)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_camera_closure_r16.blend'),compress=True)
render('hood_assembled.png')
# Close-up native views of the actual printable front, not a cosmetic overlay.
keep={FN}|{o.name for o in S.objects if o.name.startswith('Smile |')}
for o in S.objects:
 if o.type not in ['LIGHT','CAMERA'] and o.name not in keep:o.hide_render=True
for side,label in [(-1,'left'),(1,'right')]:
 aim((side*280,-260,186),(side*78,-90,162),105);render('camera_end_'+label+'.png')
assert HASH(SOURCE)==source_hash
print('R16_BUILD_COMPLETE',json.dumps({'front':checks,'net_added_volume_mm3':added_volume,'surface_preservation':surface_check}),flush=True)
