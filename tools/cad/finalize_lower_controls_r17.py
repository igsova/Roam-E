"""Final orientation exports and guide-slot measurement; preserve approved master."""
import ast,hashlib,json,math
from pathlib import Path
import bpy,bmesh
from mathutils import Matrix,Vector
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/lower_controls_r17';PACK=OUT/'ROAM_STL_R17_Lower_Controls_K1C'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for file,names in [('create_split_shell_r07.py',{'clean','remove','clone','box','boolean','metrics'}),('create_tof_solder_relief_r08.py',{'export'}),('create_lower_controls_r17.py',{'local','cb'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<finalize>','exec'))
qa=json.loads((OUT/'validation.json').read_text());assert qa['status']=='PASS'
master=OUT/'roam_lower_controls_r17.blend';assert HASH(master)==qa['master_sha256']
report=json.loads((OUT/'geometry_checks.json').read_text());rows=json.loads((OUT/'new_parts.json').read_text())
bpy.ops.wm.open_mainfile(filepath=str(master));S=bpy.context.scene;bpy.context.view_layer.update()
checks=[]
for label,sign in [('L',-1),('R',1)]:
 part=S.objects[f'P06{label} | Curved fascia lock guide R17'];origin=Vector((80*sign,report['lock_seat_y_mm']-8.4,78))/100;ds=[]
 for direction in [Vector((1,0,0)),Vector((-1,0,0))]:
  hit,p,_,_=part.ray_cast(origin,direction);assert hit;d=(p-origin).length*100;assert abs(d-2.65)<.008,(label,d);ds.append(d)
 hit,p,_,_=part.ray_cast(origin,Vector((0,0,-1)));assert hit;stop=p.z*100;assert abs(stop-74.825)<.01,(label,stop)
 checks.append({'part':part.name,'nut_grip_gap_mm':sum(ds),'nut_stop_z_mm':stop})
qa['guide_nut_slot_measurements']=checks
changed=[]
for row in rows:
 pid=row['id']
 if pid not in ['P16B','Q17A','Q17B','Q17C','Q17D']:continue
 if pid=='P16B':ob=clone(S.objects['P16B | Wheel-switch side panel R17'],'_upright panel')
 elif pid in ['Q17A','Q17B']:
  part=S.objects['P16C | Front ToF skirt R17' if pid=='Q17A' else 'P16D | Rear ToF skirt R17'];mat=Matrix(report['new_joints'][0]['matrix'])
  ob=clone(part,'_actual seam coupon');boolean(ob,cb(mat,(-1,0,5.4),(26,13.4,16)),'INTERSECT')
 else:
  mount=next(m for m in report['switch_mounts'] if m['id']==('BATTERY' if pid=='Q17C' else 'WHEELS'));mat=Matrix(mount['matrix'])
  ob=clone(S.objects[mount['part']],'_actual switch coupon');boolean(ob,cb(mat,(0,0,1),(32,32,12)),'INTERSECT')
  bm=bmesh.new();bm.from_mesh(ob.data);bmesh.ops.triangulate(bm,faces=list(bm.faces));bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=.001/100);bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=.0001/100);bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces));bm.to_mesh(ob.data);bm.free();ob.data.update()
 row['reimport_check']=export(ob,PACK/row['file'],Matrix.Identity(4));remove(ob)
 row['sha256']=HASH(PACK/row['file']);row['export_rotation_matrix']=[list(r) for r in Matrix.Identity(4)]
 row['note']=('Right panel, bottom down; WHEELS switch. Retained S1/S2 axes with PETG nut ribs. Inspect wheel/ToF and switch supports.' if pid=='P16B' else 'Actual joint/rocker fit sample, upright in the SAME orientation as its parent lower part. Print with the same PETG profile and local support settings; Q17C/D use Ø20.5 x 2.2mm seats.')
 changed.append(pid)
(OUT/'new_parts.json').write_text(json.dumps(rows,indent=2))
qa['upright_export_finalization']={'ids':changed,'source_master_unchanged':True,'all_reimported':True,'guide_nut_slots':checks}
(OUT/'validation.json').write_text(json.dumps(qa,indent=2))
assert HASH(master)==qa['master_sha256'];print('R17_FINALIZE_COMPLETE',changed,checks,flush=True)
