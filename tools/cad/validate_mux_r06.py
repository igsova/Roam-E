"""Read-only artifact audit, run with Blender after merge_mux_r06.py."""
import bpy
import hashlib
import json
import zipfile
from pathlib import Path
from mathutils import Vector

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'mechanical/geekworm_mux_r06'
OLD=ROOT/'mechanical/geekworm_r05'
hashfile=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
report=json.loads((OUT/'geometry_checks.json').read_text())
assert hashfile(Path(report['source']))==report['source_sha256']
assert hashfile(Path(report['donor']))==report['donor_sha256']
for key in ('assembly_clashes','jumper_keepout_clashes','board_mount_clashes','sampled_drawer_sweep_clashes','unexpected_changes','donor_geometry_changes'):
    assert report[key]==[],key
for pack,count,pieces in (('ROAM_STL_R06_Geekworm_MUX_K1C',41,116),('ROAM_R06_changed_parts',4,6)):
    folder=OUT/pack
    manifest=json.loads((folder/'manifest.json').read_text())
    assert len(manifest['parts'])==count
    assert sum(row['quantity'] for row in manifest['parts'])==pieces
    ids=[row['id'] for row in manifest['parts']]
    assert len(ids)==len(set(ids))
    actual={str(p.relative_to(folder)) for p in folder.rglob('*.stl')}
    expected={r['file'] for r in manifest['parts']}
    assert actual==expected,(actual-expected,expected-actual)
    for r in manifest['parts']:
        assert r['id'] not in ('C04','S06') and 'mcu' not in r['file'].lower()
        assert hashfile(folder/r['file'])==r['sha256'],r['id']
    with zipfile.ZipFile(OUT/(pack+'.zip')) as z:
        assert z.testzip() is None
        for r in manifest['parts']:
            assert hashlib.sha256(z.read(pack+'/'+r['file'])).hexdigest()==r['sha256']
    print('VERIFIED_PACK',pack,count,pieces,flush=True)
full=json.loads((OUT/'ROAM_STL_R06_Geekworm_MUX_K1C/manifest.json').read_text())
parts={r['id']:r for r in full['parts']}
assert parts['S01']['quantity']==12
oldparts={r['id']:r for r in json.loads((OLD/'ROAM_STL_R05_K1C/manifest.json').read_text())['parts']}
for id in ('C02','C08','F05','P16','P17','P18','C03'):
    assert parts[id]['sha256']==oldparts[id]['sha256'],id
for filename in ('roam_geekworm_mux_r06.blend','roam_geekworm_mux_r06_inspection.blend','roam_geekworm_mux_r06_exploded.blend','tca9548a_mount_reference.blend'):
    bpy.ops.wm.open_mainfile(filepath=str(OUT/filename));bpy.context.view_layer.update()
    scene=bpy.context.scene
    assert not any('MCU' in o.name for o in scene.objects),filename
    assert 'C09 | TCA9548A removable carrier' in scene.objects
    if not filename.startswith('tca9548a'):
        assert 'Component | Arduino_Nano' in scene.objects
        assert 'C08 | Geekworm base edge retainer front' in scene.objects
        assert 'Geekworm | 30x30x7 fan frame' in scene.objects
    board=bpy.data.objects['MUX reference | TCA9548A HW-617 PCB 21.4 x 31.5']
    points=[board.matrix_world@Vector(v)*100 for v in board.bound_box]
    dims=[max(p[i] for p in points)-min(p[i] for p in points) for i in range(3)]
    assert all(abs(a-b)<.005 for a,b in zip(dims,(21.4,31.5,1.6))),dims
    print('VERIFIED_BLEND',filename,'objects',len(scene.objects),'mux PCB mm',dims,flush=True)
print('R06_AUDIT_PASSED',flush=True)
