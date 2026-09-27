"""Isolated K1C/PETG slicing smoke tests; no connection to a printer."""
import ast,hashlib,json,subprocess,tempfile,zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];BASE=ROOT/'mechanical/service_skirt_r19';OUT=BASE/'slicer_check';OUT.mkdir(exist_ok=True)
PACK=BASE/'ROAM_R19_skirt_replacement'
PROFILES=Path('/Applications/Creality Print.app/Contents/Resources/profiles/Creality')
EXE='/Applications/Creality Print.app/Contents/MacOS/CrealityPrint'
for n in ast.parse((ROOT/'tools/cad/slice_drawer_cleanup_r18.py').read_text()).body:
 if isinstance(n,ast.FunctionDef) and n.name=='flatten':exec(compile(ast.Module(body=[n],type_ignores=[]),'<settings>','exec'))
machine=flatten('machine','Creality K1C 0.4 nozzle');process=flatten('process','0.20mm Standard @Creality K1C 0.4 nozzle');filament=flatten('filament','Generic PETG @Creality K1C 0.4 nozzle')
process.update({'name':'R19 diagnostic PETG 0.20mm','enable_support':'1','support_type':'normal(auto)','support_style':'snug','support_threshold_angle':'45','support_on_build_plate_only':'0','brim_type':'outer_only','brim_width':'2','wall_loops':'4','sparse_infill_density':'20%','enable_prime_tower':'0','gcode_comments':'1','skirt_loops':'0'})
for name,data in [('machine',machine),('process',process),('filament',filament)]:
 (OUT/(name+'.json')).write_text(json.dumps(data,indent=2))
def run(row):
 folder=OUT/row['id'];folder.mkdir(exist_ok=True);stl=PACK/row['file']
 datadir=tempfile.mkdtemp(prefix='roam-r19-slicer-')
 command=[EXE,'--datadir',datadir,'--debug','2','--load-settings',str(OUT/'machine.json')+';'+str(OUT/'process.json'),'--load-filaments',str(OUT/'filament.json'),'--orient','0','--arrange','1','--slice','0','--export-3mf',row['id']+'_diagnostic_NOT_PRINT_READY.3mf','--outputdir',str(folder),str(stl)]
 (folder/'command.json').write_text(json.dumps(command,indent=2))
 with (folder/'slice.log').open('w') as log:result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=480)
 archive=folder/(row['id']+'_diagnostic_NOT_PRINT_READY.3mf');gcode_bytes=0
 if result.returncode==0 and archive.exists():
  with zipfile.ZipFile(archive) as z:
   assert z.testzip() is None
   gcode_bytes=len(z.read('Metadata/plate_1.gcode'))
 data={'id':row['id'],'returncode':result.returncode,'stl_sha256':hashlib.sha256(stl.read_bytes()).hexdigest(),'gcode_bytes':gcode_bytes,'status':'CLI_SUCCESS' if result.returncode==0 and gcode_bytes>1000 else 'FAIL'}
 (folder/'result.json').write_text(json.dumps(data,indent=2));print('R19_SLICE',data,flush=True);return data
rows=json.loads((BASE/'new_parts.json').read_text())
with ThreadPoolExecutor(max_workers=2) as executor:results=list(executor.map(run,rows))
report={'status':'PASS' if all(r['status']=='CLI_SUCCESS' for r in results) else 'FAIL','parts':results,'profile':'K1C 0.4mm, Generic PETG, 0.20mm, four walls, 20% infill, automatic snug supports, 2mm brim','limits':'Slicing smoke test only. Inspect supports, nut pockets and printer exclusions. Diagnostic G-code is not an approved print job.'}
(OUT/'result.json').write_text(json.dumps(report,indent=2));print('R19_SLICING_COMPLETE',report['status'],flush=True)
