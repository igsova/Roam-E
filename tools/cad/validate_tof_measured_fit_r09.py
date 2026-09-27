"""Independent measurement/identity/STL checks and release packaging for R09."""
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
OLD=ROOT/'mechanical/tof_solder_relief_r08'
OUT=ROOT/'mechanical/tof_measured_fit_r09'
PACK=OUT/'ROAM_STL_R09_Measured_ToF_K1C'
CHANGED=OUT/'ROAM_R09_changed_parts'
OLDPACK=OLD/'ROAM_STL_R08_ToF_Solder_Relief_K1C'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for node in ast.parse((ROOT/'tools/cad/create_split_shell_r07.py').read_text()).body:
    if isinstance(node,ast.FunctionDef) and node.name in {'metrics','snapshot','remove'}:
        exec(compile(ast.Module(body=[node],type_ignores=[]),'<geometry checks>','exec'))
report=json.loads((OUT/'geometry_checks.json').read_text())
assert not any(report[k] for k in ['unexpected_original_changes','addition_clashes','reference_clashes','optical_ray_obstructions'])
assert HASH(OLD/'roam_tof_solder_relief_r08.blend')==report['source_R08_sha256']
assert HASH(ROOT/'mechanical/split_shell_r07/roam_split_shell_r07.blend')==report['base_R07_sha256']
bpy.ops.wm.open_mainfile(filepath=str(OLD/'roam_tof_solder_relief_r08.blend'))
bpy.context.view_layer.update()
original={o.name:snapshot(o) for o in bpy.context.scene.objects}
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_tof_measured_fit_r09.blend'))
bpy.context.view_layer.update();S=bpy.context.scene
allowed=set(report['allowed_original_changes']);renames=report['renamed_reference_PCBS']
expected_names=(set(original)-set(renames))|set(renames.values())
assert expected_names=={o.name for o in S.objects},expected_names.symmetric_difference({o.name for o in S.objects})
changed=[name for name,h in original.items() if snapshot(S.objects[renames.get(name,name)])!=h]
assert set(changed)==allowed,(set(changed)^allowed)
assert len(allowed)==107 # 3 prints + 8 boards + 48 pads + 48 header pins
assert 'R09' in S['Mechanical revision']

def local_bounds(obj,mat):
    vs=[mat.inverted()@(obj.matrix_world@v.co)*100 for v in obj.data.vertices]
    return [(min(v[i] for v in vs),max(v[i] for v in vs)) for i in range(3)]
measurements=[]
for tag in ['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']:
    pcb=S.objects['ToF '+tag+' | Reference PCB 25x10.66 R09'];mat=pcb.matrix_world.copy()
    pb=local_bounds(pcb,mat);assert abs(pb[1][1]-pb[1][0]-10.66)<.005
    sensor=S.objects['ToF '+tag+' | VL53L0X optical package'];sb=local_bounds(sensor,mat)
    part=S.objects['P16B | Removable ToF side panel R07' if tag=='Right' else 'P16A | Main ToF skirt R07']
    inv=part.matrix_world.inverted();n=mat.to_3x3()@Vector((0,0,1));distances=[];edgegaps=[]
    for i in range(6):
        pad=S.objects['ToF '+tag+' | Header pad '+str(i)];bb=local_bounds(pad,mat)
        u=sum(bb[0])/2;v=sum(bb[1])/2;edgegap=sb[1][0]-v
        assert abs(edgegap-4.8)<.005,(tag,i,edgegap);edgegaps.append(edgegap)
        for dv in [-.85,0,.85]:
            p=mat@(Vector((u,v+dv,-.85))/100)
            hit,at,_,_=part.ray_cast(inv@p,(inv.to_3x3()@n).normalized());assert hit
            d=(part.matrix_world@at-p).length*100;assert abs(d-2)<.005,(tag,i,dv,d)
            distances.append(d)
    measurements.append({'station':tag,'PCB_short_dimension_mm':pb[1][1]-pb[1][0],
        'sensor_edge_to_six_pin_centres_mm':edgegaps,'solder_clearances_18_samples_mm':distances})

manifest=json.loads((PACK/'manifest.json').read_text())
previous={r['id']:r for r in json.loads((OLDPACK/'manifest.json').read_text())['parts']}
assert len(manifest['parts'])==45 and sum(r['quantity'] for r in manifest['parts'])==120
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
result={'revision':'R09','status':'digital checks passed; physical P19 fit and slicer preview required',
    'unchanged_original_objects':len(original)-len(changed),'intentional_changed_object_count':len(changed),
    'unchanged_STL_ids':same,'new_STL_checks':new,'independent_measured_fit':measurements,
    'source_R08_sha256':report['source_R08_sha256'],'base_R07_sha256':report['base_R07_sha256'],
    'remaining_assumptions':report['remaining_assumptions'],
    'limits':['New width and pin spacing are user-measured; other board datums remain assumptions.',
        'Bounding-box fit is not slicer toolpath validation. Print R09 P19 before either skirt half.']}
(OUT/'validation.json').write_text(json.dumps(result,indent=2))
for name in ['Split_shell_R07_reference.md','Electronics_R06_reference.md','Geekworm_mount_R05_reference.md']:
    shutil.copy2(OLDPACK/name,PACK/name)
shutil.copytree(OLDPACK/'historical_R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
for target in [PACK,CHANGED]:
    guide=target/'START_HERE.md'
    guide.write_text((OUT/'ASSEMBLY_R09.md').read_text().replace('../split_shell_r07/ASSEMBLY_R07.md','Split_shell_R07_reference.md'))
    shutil.copy2(OLDPACK/'Split_shell_R07_reference.md',target/'Split_shell_R07_reference.md')
    for name in ['geometry_checks.json','validation.json','measured_fit_comparison.png']:
        shutil.copy2(OUT/name,target/name)
    with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(target.rglob('*')):
            if p.is_file():z.write(p,p.relative_to(OUT))
    with zipfile.ZipFile(OUT/(target.name+'.zip')) as z:
        assert z.testzip() is None
        rows=json.loads((target/'manifest.json').read_text())['parts']
        for r in rows:assert hashlib.sha256(z.read(target.name+'/'+r['file'])).hexdigest()==r['sha256']
        assert len([n for n in z.namelist() if n.endswith('.stl')])==len(rows)
print('R09_RELEASE_COMPLETE',json.dumps({'changed_STLs':new,'unchanged_STL_count':len(same),'unchanged_original_objects':result['unchanged_original_objects']}),flush=True)
