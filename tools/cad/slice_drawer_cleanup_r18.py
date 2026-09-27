"""Isolated PETG/K1C slicing smoke test, never sends anything to a printer.

The generated project is diagnostic, not a tuned or approved print job.
"""
import json
import hashlib
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'mechanical/drawer_cleanup_r18/slicer_check'
PROFILES = Path('/Applications/Creality Print.app/Contents/Resources/profiles/Creality')
EXE = '/Applications/Creality Print.app/Contents/MacOS/CrealityPrint'

def flatten(kind,name):
    data = json.loads((PROFILES/kind/(name+'.json')).read_text())
    result = flatten(kind,data['inherits']) if data.get('inherits') else {}
    result.update(data)
    result.pop('inherits',None)
    return result

OUT.mkdir(parents=True,exist_ok=True)
machine = flatten('machine','Creality K1C 0.4 nozzle')
process = flatten('process','0.20mm Standard @Creality K1C 0.4 nozzle')
filament = flatten('filament','Generic PETG @Creality K1C 0.4 nozzle')
process.update({'name':'R18 drawer diagnostic PETG 0.20mm',
    'enable_support':'1','support_type':'normal(auto)','support_style':'snug',
    'support_threshold_angle':'45','support_on_build_plate_only':'0',
    'brim_type':'outer_only','brim_width':'4','wall_loops':'3',
    'sparse_infill_density':'15%','enable_prime_tower':'0',
    'gcode_comments':'1','skirt_loops':'0'})
for key,value in [('machine',machine),('process',process),('filament',filament)]:
    (OUT/(key+'.json')).write_text(json.dumps(value,indent=2))
stl = ROOT / 'mechanical/drawer_cleanup_r18/ROAM_R18_drawer_replacement/01_rigid/P05_curved_battery_drawer_R18_qty1.stl'
settings_dir = tempfile.mkdtemp(prefix='roam-r18-slicer-')
command = [EXE,'--datadir',settings_dir,'--debug','2',
    '--load-settings',str(OUT/'machine.json')+';'+str(OUT/'process.json'),
    '--load-filaments',str(OUT/'filament.json'),'--orient','0','--arrange','1',
    '--slice','0','--export-3mf','R18_drawer_diagnostic_NOT_PRINT_READY.3mf',
    '--outputdir',str(OUT),str(stl)]
(OUT/'command.json').write_text(json.dumps(command,indent=2))
with (OUT/'slice.log').open('w') as log:
    result = subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=480)
(OUT/'result.json').write_text(json.dumps({'returncode':result.returncode,
    'stl_sha256':hashlib.sha256(stl.read_bytes()).hexdigest(),
    'status':'CLI_SUCCESS' if result.returncode==0 else 'CLI_FAILED',
    'profile':'K1C 0.4 nozzle; Generic PETG; 0.20mm; normal snug supports; 4mm brim',
    'notice':'Slicing smoke test only, not a calibrated/approved machine job. No printer connection.'},indent=2))
print('R18_SLICER_RETURN_CODE',result.returncode,flush=True)
raise SystemExit(result.returncode)
