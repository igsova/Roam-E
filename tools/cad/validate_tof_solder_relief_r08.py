"""Independently reopen R07/R08, verify changed STLs and package R08 release."""
import ast
import hashlib
import json
import shutil
import zipfile
from pathlib import Path
import bpy
import bmesh
from mathutils import Matrix,Vector

ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/split_shell_r07'
OUT=ROOT/'mechanical/tof_solder_relief_r08'
PACK=OUT/'ROAM_STL_R08_ToF_Solder_Relief_K1C'
CHANGED=OUT/'ROAM_R08_changed_parts'
OLDPACK=OLD/'ROAM_STL_R07_Split_Shell_K1C'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for node in ast.parse((ROOT/'tools/cad/create_split_shell_r07.py').read_text()).body:
    if isinstance(node,ast.FunctionDef) and node.name in {'metrics','snapshot','remove'}:
        exec(compile(ast.Module(body=[node],type_ignores=[]),'<geometry checks>','exec'))
report=json.loads((OUT/'geometry_checks.json').read_text())
assert not any(report[k] for k in ['unexpected_original_changes','addition_clashes','optical_ray_obstructions'])
assert report['assumptions']['clearance_from_PCB_face_mm']==2
assert HASH(OLD/'roam_split_shell_r07.blend')==report['source_sha256']
bpy.ops.wm.open_mainfile(filepath=str(OLD/'roam_split_shell_r07.blend'))
bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in bpy.context.scene.objects}
allowed={'P16A | Main ToF skirt R07','P16B | Removable ToF side panel R07','P19 | ToF fit-test pocket'}
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_tof_solder_relief_r08.blend'))
bpy.context.view_layer.update();S=bpy.context.scene
changed=[n for n,h in original.items() if n not in S.objects or snapshot(S.objects[n])!=h]
assert set(changed)==allowed,changed
assert set(original)=={o.name for o in S.objects}
assert 'R08' in S['Mechanical revision']
measurements=[]
for tag in ['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']:
    mat=S.objects['ToF '+tag+' | Reference PCB 25x12.6'].matrix_world.copy()
    part=S.objects['P16B | Removable ToF side panel R07' if tag=='Right' else 'P16A | Main ToF skirt R07']
    inv=part.matrix_world.inverted();n=mat.to_3x3()@Vector((0,0,1));distances=[]
    for u in [-6.35,-3.81,-1.27,1.27,3.81,6.35]:
        p=mat@(Vector((u,-7.9,-.85))/100)
        hit,at,_,_=part.ray_cast(inv@p,(inv.to_3x3()@n).normalized())
        assert hit
        d=(part.matrix_world@at-p).length*100;assert abs(d-2)<.005
        distances.append(d)
    measurements.append({'station':tag,'clearance_at_six_header_pins_mm':distances})

manifest=json.loads((PACK/'manifest.json').read_text())
previous={r['id']:r for r in json.loads((OLDPACK/'manifest.json').read_text())['parts']}
assert len(manifest['parts'])==45
same=[];new={}
for row in manifest['parts']:
    path=PACK/row['file'];assert HASH(path)==row['sha256']
    if row['id'] not in ['P16A','P16B','P19']:
        assert row['sha256']==previous[row['id']]['sha256'];same.append(row['id'])
    else:
        assert row['sha256']!=previous[row['id']]['sha256']
        bpy.ops.wm.stl_import(filepath=str(path),use_facet_normal=False)
        obj=bpy.context.object;m=metrics(obj,1)
        assert m['components']==1 and m['nonmanifold_edges']==0 and m['volume_mm3']>0 and abs(m['min_z_mm'])<.001,m
        assert all(abs(a-b)<.002 for a,b in zip(m['dimensions_mm'],row['reimport_check']['dimensions_mm']))
        assert all(d+4<220 for d in m['dimensions_mm'][:2])
        new[row['id']]=m;remove(obj)
assert len(same)==42
result={'revision':'R08','status':'digital checks passed; physical solder fit and slicer preview required',
    'unchanged_original_objects':len(original)-3,'only_changed_original_objects':changed,
    'unchanged_STL_ids':same,'new_STL_checks':new,'independent_pin_clearances':measurements,
    'source_R07_sha256':report['source_sha256'],
    'limits':['User pin projection is approximate, not a caliper measurement.',
        'Bounding-box fit is not slicer toolpath validation.',
        'Print P19 with the actual soldered module before printing either skirt half.']}
(OUT/'validation.json').write_text(json.dumps(result,indent=2))
shutil.copy2(OLD/'ASSEMBLY_R07.md',PACK/'Split_shell_R07_reference.md')
for name in ['Electronics_R06_reference.md','Geekworm_mount_R05_reference.md']:
    shutil.copy2(OLDPACK/name,PACK/name)
shutil.copytree(OLDPACK/'historical_R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
for target in [PACK,CHANGED]:
    shutil.copy2(OUT/'ASSEMBLY_R08.md',target/'START_HERE.md')
    guide=target/'START_HERE.md'
    guide.write_text(guide.read_text().replace('../split_shell_r07/ASSEMBLY_R07.md','Split_shell_R07_reference.md'))
    # Make the archive's inherited-instructions link work inside the archive too.
    shutil.copy2(OLD/'ASSEMBLY_R07.md',target/'Split_shell_R07_reference.md')
    for name in ['geometry_checks.json','validation.json','solder_relief_comparison.png']:
        shutil.copy2(OUT/name,target/name)
    with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(target.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(OUT))
    with zipfile.ZipFile(OUT/(target.name+'.zip')) as z:
        assert z.testzip() is None
        rows=json.loads((target/'manifest.json').read_text())['parts']
        for r in rows:assert hashlib.sha256(z.read(target.name+'/'+r['file'])).hexdigest()==r['sha256']
        assert len([n for n in z.namelist() if n.endswith('.stl')])==len(rows)
print('R08_RELEASE_COMPLETE',json.dumps(result),flush=True)
