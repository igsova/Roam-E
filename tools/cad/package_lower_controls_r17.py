"""Package R17 only after the independent validator passes."""
import csv,hashlib,json,shutil,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/lower_controls_r17'
OLD=ROOT/'mechanical/camera_closure_r16/ROAM_STL_R16_Camera_Closure_K1C'
PACK=OUT/'ROAM_STL_R17_Lower_Controls_K1C';CHANGED=OUT/'ROAM_R17_lower_replacement'
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
qa=json.loads((OUT/'validation.json').read_text());assert qa['status']=='PASS',qa['status']
assert qa['master_sha256']==HASH(OUT/'roam_lower_controls_r17.blend')
new=json.loads((OUT/'new_parts.json').read_text());old=json.loads((OLD/'manifest.json').read_text())
replaced={'P05','P06L','P06R','P16A','P16B'};kept=[];records=[]
for r in old['parts']:
 if r['id'] in replaced:continue
 p=PACK/r['file'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(OLD/r['file'],p)
 assert HASH(p)==r['sha256'];kept.append(r['id']);records.append(r)
for r in new:
 assert HASH(PACK/r['file'])==r['sha256'];records.append(r)
 p=CHANGED/r['file'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/r['file'],p)
assert len(kept)==57
manifest={**old,'revision':'R17 lower controls mechanical prototype; fit tests required','source':'roam_lower_controls_r17.blend','master_sha256':qa['master_sha256'],'replaces':['P16A/P16B -> P16C/P16D/P16B matching three-piece skirt','P05, P06L, P06R -> matching curved switched drawer and rear lock guides'],'R17_switch_status':'Mechanical reference only; Ø20.5 x 2.2mm provisional snap seat; actual hardware fit and DC switching rating unverified','R17_additional_hardware':'2 M3x12 screws, 2 standard M3 nuts 5.5 AF x 2.4mm, 2 6mm OD x .5 washers. Reuse S1/S2 and E lock hardware. Two purchased rockers.'}
export_checks={'status':'PASS','master_sha256':qa['master_sha256'],'new_parts':{r['id']:r['reimport_check'] for r in new},'retained_R16_STL_count':len(kept),'retained_R16_STL_ids':kept,'all_new_files_reimported_during_build':True,'fit_status':'Not physically tested. Prototype coupons required.'}
(OUT/'export_checks.json').write_text(json.dumps(export_checks,indent=2))
for target,rows in [(PACK,records),(CHANGED,new)]:
 target.mkdir(parents=True,exist_ok=True)
 (target/'manifest.json').write_text(json.dumps({**manifest,'parts':rows,'unique_stl_files':len(rows),'total_physical_pieces_including_optional':sum(r['quantity'] for r in rows)},indent=2))
 with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
  for r in rows:w.writerow([r['id'],r['label'],r['quantity'],r['file'],*r['reimport_check']['dimensions_mm'],r['support'],r['note']])
 for name in ['ASSEMBLY_R17.md','POWER_NOTES.md','geometry_checks.json','validation.json','export_checks.json','lower_assembled.png','lower_exploded.png','robot_rear_controls.png']:
  shutil.copy2(OUT/name,target/('START_HERE.md' if name=='ASSEMBLY_R17.md' else name))
 with (target/'R17_HARDWARE.csv').open('w',newline='') as f:
  w=csv.writer(f);w.writerow(['Use','Quantity','Hardware','Notes'])
  w.writerow(['New S3/S4 skirt seam',2,'M3x12 screw + M3 nut + washer','New hardware, standard 5.5AF x 2.4 nut, 6OD x .5 washer'])
  w.writerow(['Existing S1/S2 skirt joints',2,'M3x12 screw + M3 nut + washer','Reuse; R17 adds PETG nut retention'])
  w.writerow(['Existing E drawer locks',2,'M3x12 screw + M3 nut + washer','Reuse at revised Y locations with matching new guides'])
  w.writerow(['Power controls',2,'Purchased Hanibos round rocker','Not printed; electrical DC rating and snap fit not verified'])
shutil.copy2(OLD/'HARDWARE.csv',PACK/'HARDWARE_R16_REFERENCE.csv')
shutil.copy2(OLD/'START_HERE.md',PACK/'R16_UPPER_HOOD_REFERENCE.md')
for p in OLD.iterdir():
 if p.is_file() and p.name.endswith('_reference.md'):shutil.copy2(p,PACK/p.name)
shutil.copytree(OLD/'historical_R04_reference',PACK/'historical_R04_reference',dirs_exist_ok=True)
for target in [PACK,CHANGED]:
 with zipfile.ZipFile(OUT/(target.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as z:
  for p in sorted(target.rglob('*')):
   if p.is_file():z.write(p,p.relative_to(OUT))
 with zipfile.ZipFile(OUT/(target.name+'.zip')) as z:
  assert z.testzip() is None
  rows=json.loads((target/'manifest.json').read_text())['parts'];assert len([n for n in z.namelist() if n.endswith('.stl')])==len(rows)
  for r in rows:assert hashlib.sha256(z.read(target.name+'/'+r['file'])).hexdigest()==r['sha256']
print('R17_PACKAGE_COMPLETE',len(records),'STLs;',len(new),'new;',len(kept),'unchanged')
