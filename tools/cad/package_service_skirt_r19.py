"""Publish only a hash-matched, fully checked R19 replacement package."""
import hashlib,json,shutil,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/service_skirt_r19';PACK=OUT/'ROAM_R19_skirt_replacement'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
qa=json.loads((OUT/'validation.json').read_text());extra=json.loads((OUT/'interface_checks.json').read_text());build=json.loads((OUT/'geometry_checks.json').read_text());slices=json.loads((OUT/'slicer_check/result.json').read_text());rows=json.loads((OUT/'new_parts.json').read_text())
assert qa['status']==extra['status']==slices['status']=='PASS'
assert qa['master_sha256']==extra['master_sha256']==HASH(OUT/'roam_service_skirt_r19.blend')
assert build['source_R18_sha256']==HASH(ROOT/'mechanical/drawer_cleanup_r18/roam_drawer_cleanup_r18.blend')
assert len(rows)==9
for row in rows:
 assert HASH(PACK/row['file'])==row['sha256']
 assert next(r for r in slices['parts'] if r['id']==row['id'])['stl_sha256']==row['sha256']
manifest={'revision':'R19 service skirt prototype','parts':rows,'installed_printed_pieces':7,'fit_sample_pieces':2,'replaces':['P16B R17','P16C R17','P16D R17'],'retain':'R18 P05 drawer, R17 P06 guides, measured ToFs/retainers/pads, chassis, wheels, upper robot and original hardware.','additional_hardware':{'M3x12_socket_screws':4,'M3_standard_nuts_5.5AF_2.4thick':4,'M3_washers':4},'source_R18_sha256':build['source_R18_sha256'],'master_sha256':qa['master_sha256'],'status':'DIGITAL_CHECKS_PASS; physical prototype fit still unverified','limits':qa['limits']}
(PACK/'manifest.json').write_text(json.dumps(manifest,indent=2))
shutil.copy2(OUT/'README.md',PACK/'START_HERE.md')
for n in ['validation.json','interface_checks.json','geometry_checks.json','R18_path_exploration.json','R18_cable_probe.json','R19_lower_assembled.png','R19_lower_exploded.png','R19_fixed_cleat_detail.png','R19_four_female_jumpers.png']:shutil.copy2(OUT/n,PACK/n)
shutil.copy2(OUT/'slicer_check/result.json',PACK/'slicing_smoke_test.json')
archive=OUT/(PACK.name+'.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
 for p in sorted(PACK.rglob('*')):
  if p.is_file():z.write(p,p.relative_to(OUT))
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 assert len([n for n in z.namelist() if n.endswith('.stl')])==9
 for row in rows:assert hashlib.sha256(z.read(PACK.name+'/'+row['file'])).hexdigest()==row['sha256']
release={'status':'PASS','archive':archive.name,'archive_sha256':HASH(archive),'master_sha256':qa['master_sha256'],'demo_sha256':HASH(OUT/'roam_service_skirt_r19_assembly_demo.blend'),'unchanged_R18_objects':qa['unchanged_object_count'],'parts':9,'installed_pieces':7,'fit_samples':2}
(OUT/'release_checks.json').write_text(json.dumps(release,indent=2));print('R19_RELEASE_COMPLETE',json.dumps(release),flush=True)
