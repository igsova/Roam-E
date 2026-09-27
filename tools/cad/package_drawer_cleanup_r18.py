"""Publish only the validated P05 replacement; never overwrite R17 assets."""
import hashlib
import json
import shutil
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'mechanical/drawer_cleanup_r18'
PACK = OUT / 'ROAM_R18_drawer_replacement'
HASH = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
qa = json.loads((OUT/'validation.json').read_text())
build = json.loads((OUT/'geometry_checks.json').read_text())
assert qa['status'] == 'PASS', qa
assert qa['master_sha256'] == build['master_sha256'] == HASH(OUT/'roam_drawer_cleanup_r18.blend')
assert HASH(ROOT/'mechanical/lower_controls_r17/roam_lower_controls_r17.blend') == build['source_R17_sha256']
row = build['parts'][0]
assert HASH(PACK/row['file']) == row['sha256']
slice_result = json.loads((OUT/'slicer_check/result.json').read_text())
assert slice_result['returncode'] == 0 and slice_result['stl_sha256'] == row['sha256']
with zipfile.ZipFile(OUT/'slicer_check/R18_drawer_diagnostic_NOT_PRINT_READY.3mf') as z:
    assert z.testzip() is None and 'Metadata/plate_1.gcode' in z.namelist()
    assert len(z.read('Metadata/plate_1.gcode')) > 100000

build['status'] = 'BUILD_COMPLETE; independent validation PASS'
build['validation_report'] = 'validation.json'
(OUT/'geometry_checks.json').write_text(json.dumps(build,indent=2))

manifest = {'revision':'R18 drawer-only mechanical correction',
    'master_sha256':build['master_sha256'], 'source_R17_sha256':build['source_R17_sha256'],
    'replaces':['P05_curved_battery_drawer_R17_qty1.stl'],
    'retain':'All other R17 parts, including P06L/P06R and all lock hardware.',
    'parts':build['parts'],'unique_stl_files':1,'physical_pieces':1,
    'fit_status':'Digital checks passed; physical PETG and switch fit unverified.',
    'printing':'Supports required as determined in slicer; not a support-free redesign.',
    'validation':'validation.json','slicer_check':slice_result}
(PACK/'manifest.json').write_text(json.dumps(manifest,indent=2))
guide = (OUT/'README.md').read_text()
guide = guide.replace('ROAM_R18_drawer_replacement/01_rigid/','01_rigid/')
guide = guide.replace('(ROAM_R18_drawer_replacement.zip)','(../ROAM_R18_drawer_replacement.zip)')
guide = guide.replace('(roam_drawer_cleanup_r18.blend)','(../roam_drawer_cleanup_r18.blend)')
guide = guide.replace('(roam_drawer_cleanup_r18_inspection.blend)','(../roam_drawer_cleanup_r18_inspection.blend)')
(PACK/'START_HERE.md').write_text(guide)
for name in ['geometry_checks.json','validation.json','drawer_front.png',
             'drawer_inside.png','drawer_bottom.png','washer_seat_detail.png']:
    shutil.copy2(OUT/name,PACK/name)
shutil.copy2(OUT/'slicer_check/result.json',PACK/'slicing_smoke_test.json')
archive = OUT/(PACK.name+'.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(PACK.rglob('*')):
        if p.is_file():
            z.write(p,p.relative_to(OUT))
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    assert len([n for n in z.namelist() if n.endswith('.stl')]) == 1
    assert hashlib.sha256(z.read(PACK.name+'/'+row['file'])).hexdigest() == row['sha256']
release = {'status':'PASS','zip':archive.name,'zip_sha256':HASH(archive),
    'master_sha256':build['master_sha256'],'stl_sha256':row['sha256'],
    'preview_sha256':{n:HASH(OUT/n) for n in ['drawer_front.png','drawer_inside.png','drawer_bottom.png','washer_seat_detail.png']},
    'unchanged_other_R17_objects':qa['unchanged_R17_objects']}
(OUT/'release_checks.json').write_text(json.dumps(release,indent=2))
print('R18_PACKAGE_COMPLETE',archive,flush=True)
