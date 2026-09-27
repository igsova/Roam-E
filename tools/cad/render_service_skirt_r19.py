"""R19 actual-geometry previews and reversible assembly demonstration."""
import ast,json
from pathlib import Path
import bpy
from mathutils import Vector
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'mechanical/service_skirt_r19'
for file,names in [('create_tof_solder_relief_r08.py',{'aim'}),('create_split_shell_r07.py',{'bounds'})]:
 for n in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
  if isinstance(n,ast.FunctionDef) and n.name in names:exec(compile(ast.Module(body=[n],type_ignores=[]),'<view>','exec'))
bpy.ops.wm.open_mainfile(filepath=str(OUT/'roam_service_skirt_r19.blend'));bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene;bpy.context.view_layer.update();report=json.loads((OUT/'geometry_checks.json').read_text())
panels=list(report['renames'].values());cleats=[j['cleat'] for j in report['joints']]
original_visible={o.name:not o.hide_render for o in S.objects}
lower=[o.name for o in S.objects if o.type=='MESH' and original_visible[o.name] and bounds(o)[2][0]<101 and not any(c.name=='06 Studio' for c in o.users_collection)]
groups={p:[p] for p in panels};removed=[]
for o in S.objects:
 o.color=(.29,.34,.37,1)
 if o.name in panels:o.color=(.38,.63,.67,1)
 if o.name in cleats:o.color=(.95,.39,.095,1)
 if 'screw' in o.name or 'washer' in o.name or 'nut' in o.name:o.color=(.58,.60,.62,1)
 if o.name.startswith('R19 ToF '):
  if o.data.materials:o.color=o.data.materials[0].diffuse_color
 tag=o.get('ToF station','')
 if tag:groups[panels[0] if tag=='Right' else panels[1] if tag.startswith('Front') else panels[2]].append(o.name)
 if o.name.startswith('R17 WHEELS rocker'):groups[panels[0]].append(o.name)
 if o.name.startswith(('S1 |','S2 |','R17 S3 |','R17 S4 |')):
  if 'nut' in o.name:groups[panels[0] if o.name.startswith(('S1 |','S2 |')) else panels[2]].append(o.name)
  else:removed.append(o.name)
 if o.name.startswith('R19 M') and 'Retained' not in o.name:removed.append(o.name)
S.render.engine='BLENDER_WORKBENCH';S.render.resolution_x=1550;S.render.resolution_y=1100;S.render.resolution_percentage=100
S.render.image_settings.file_format='PNG';S.render.film_transparent=False;S.display.shading.light='STUDIO';S.display.shading.color_type='OBJECT'
S.display.shading.show_shadows=True;S.display.shading.show_cavity=True;S.display.shading.cavity_type='BOTH';S.display.shading.background_type='WORLD';S.world.color=(.025,.03,.04)
S.view_settings.view_transform='Standard'
def show(names):
 for o in S.objects:
  if o.type not in ['CAMERA','LIGHT']:
   hide=o.name not in names;o.hide_render=hide;o.hide_set(hide)
def render(name,pos,target,span):
 aim(pos,target,span);S.render.filepath=str(OUT/name);bpy.ops.render.render(write_still=True)
show(lower)
render('R19_lower_assembled.png',(330,-420,365),(0,0,56),330)
base={n:S.objects[n].location.copy() for g in groups.values() for n in g}
for p,d in zip(panels,[(65,0,0),(0,-65,0),(0,65,0)]):
 for n in groups[p]:S.objects[n].location=base[n]+Vector(d)/100
show(set(lower)-set(removed))
render('R19_lower_exploded.png',(360,-450,460),(0,0,57),430)
for n,v in base.items():S.objects[n].location=v
show([report['joints'][2]['panel'],report['joints'][2]['cleat']]+[o.name for o in S.objects if o.name.startswith(('R19 M3 |','M3 |','P01 |','P06L |','P03.3 |'))])
render('R19_fixed_cleat_detail.png',(5,-65,156),(-104,39,67),92)
show([panels[1]]+[o.name for o in S.objects if o.get('ToF station')=='Front_L' and original_visible[o.name]])
render('R19_four_female_jumpers.png',(-66,-28,21),(-68,-96,38),56)

# Full travel shown, not just the compact exploded illustration. Removing the
# bolts and unplugging flexible tails is a prerequisite, not an animation.
show(set(lower)-set(removed))
for index,(p,d) in enumerate(zip(panels,[(240,0,0),(0,-240,0),(0,240,0)])):
 start=1+index*40;end=start+40;return_start=241-end;return_end=241-start
 for name in groups[p]:
  o=S.objects[name]
  for f,v in [(1,(0,0,0)),(start,(0,0,0)),(end,d),(return_start,d),(return_end,(0,0,0)),(241,(0,0,0))]:
   o.location=base[name]+Vector(v)/100;o.keyframe_insert(data_path='location',frame=f)
S.frame_start=1;S.frame_end=241;S.render.fps=24
for f,title in [(1,'REMOVE RIGHT — bolts removed, harness unplugged'),(41,'REMOVE FRONT'),(81,'REMOVE REAR'),(121,'ASSEMBLE REAR'),(161,'ASSEMBLE FRONT'),(201,'ASSEMBLE RIGHT')]:S.timeline_markers.new(title,frame=f)
S.frame_set(1);aim((420,-570,510),(0,0,56),690)
S['Inspection only']='Animated shell removal/reassembly. Hide upper assembly for visibility only. Tests include installed upper assembly. Bolts/washers removed and harness unplugged before motion.'
S['Animation motion']='Default Blender interpolation is eased; each segment is a straight translation. The separate validator uses fixed 0.5mm samples, not timeline frames.'
for screen in bpy.data.screens:
 for area in screen.areas:
  if area.type=='VIEW_3D':area.spaces.active.clip_end=100
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_service_skirt_r19_assembly_demo.blend'),compress=True)
print('R19_PREVIEWS_COMPLETE',flush=True)
