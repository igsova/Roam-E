"""Isolated Creality Print comparison; no printer connection or user preset edits.

Uses bundled K1C 0.4 / 0.20 mm / Generic PLA presets with identical overrides
for R06 and R07. Generated projects are comparison artifacts, NOT print-ready
machine instructions. Review material, support and printer settings in GUI.
"""
import argparse
import json
import subprocess
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'mechanical/split_shell_r07/slicer_check'
PROFILES=Path('/Applications/Creality Print.app/Contents/Resources/profiles/Creality')
EXE='/Applications/Creality Print.app/Contents/MacOS/CrealityPrint'

def flatten(kind,name):
    path=PROFILES/kind/(name+'.json')
    data=json.loads(path.read_text())
    result=flatten(kind,data['inherits']) if data.get('inherits') else {}
    result.update(data);result.pop('inherits',None)
    return result

def prepare():
    OUT.mkdir(parents=True,exist_ok=True)
    machine=flatten('machine','Creality K1C 0.4 nozzle')
    process=flatten('process','0.20mm Standard @Creality K1C 0.4 nozzle')
    filament=flatten('filament','Generic PLA @Creality K1C 0.4 nozzle')
    process.update({'name':'R07 controlled comparison 0.20 mm', 'enable_support':'1',
        'support_type':'normal(auto)','support_style':'snug','support_threshold_angle':'45',
        'support_on_build_plate_only':'0','brim_type':'outer_only','brim_width':'4',
        'wall_loops':'3','sparse_infill_density':'15%','enable_prime_tower':'0',
        'gcode_comments':'1','skirt_loops':'0'})
    for key,value in [('machine',machine),('process',process),('filament',filament)]:
        (OUT/(key+'.json')).write_text(json.dumps(value,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('stl',nargs='?')
    parser.add_argument('--name')
    args=parser.parse_args()
    prepare()
    if args.stl:
        name=args.name or Path(args.stl).stem
        dest=OUT/name;dest.mkdir(exist_ok=True)
        command=[EXE,'--datadir','/private/tmp/roam-r07-creality-profiles','--debug','2',
            '--load-settings',f'{OUT}/machine.json;{OUT}/process.json',
            '--load-filaments',str(OUT/'filament.json'),
            '--orient','0','--arrange','1','--slice','0',
            '--export-3mf',str(dest/(name+'.3mf')),'--outputdir',str(dest),str(Path(args.stl).resolve())]
        (dest/'command.json').write_text(json.dumps(command,indent=2))
        with (dest/'slice.log').open('w') as f:
            result=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,timeout=480)
        print(name,'returncode',result.returncode,flush=True)
        raise SystemExit(result.returncode)
