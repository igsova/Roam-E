"""Geometry-only R18 inspection views, with flat shading (no cosmetic smoothing)."""
import ast
import json
from pathlib import Path
import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'mechanical/drawer_cleanup_r18'
for n in ast.parse((ROOT / 'tools/cad/create_tof_solder_relief_r08.py').read_text()).body:
    if isinstance(n,ast.FunctionDef) and n.name == 'aim':
        exec(compile(ast.Module(body=[n],type_ignores=[]),'<view>', 'exec'))
bpy.ops.wm.open_mainfile(filepath=str(OUT / 'roam_drawer_cleanup_r18.blend'))
bpy.context.preferences.filepaths.save_version = 0
S = bpy.context.scene
drawer = S.objects[json.loads((OUT / 'geometry_checks.json').read_text())['changed_part']]
for o in S.objects:
    if o.type not in ['LIGHT','CAMERA']:
        o.hide_render = o != drawer
        o.hide_set(o != drawer)
S.render.engine = 'BLENDER_WORKBENCH'
S.render.resolution_x = 1500
S.render.resolution_y = 1050
S.render.resolution_percentage = 100
S.render.image_settings.file_format = 'PNG'
S.render.film_transparent = False
S.display.shading.light = 'STUDIO'
S.display.shading.color_type = 'SINGLE'
S.display.shading.single_color = (.6,.72,.77)
S.display.shading.show_shadows = True
S.display.shading.show_cavity = True
S.display.shading.cavity_type = 'BOTH'
S.display.shading.curvature_ridge_factor = 1.15
S.display.shading.curvature_valley_factor = 1.1
S.display.shading.background_type = 'WORLD'
S.world.color = (.035,.045,.06)
S.view_settings.view_transform = 'Standard'
for name,pos,target,span in [
    ('drawer_front.png',(210,310,180),(0,67,77),225),
    ('drawer_inside.png',(-135,-160,265),(0,65,73),225),
    ('drawer_bottom.png',(140,220,-210),(0,53,75),255),
    ('washer_seat_detail.png',(-160,230,115),(-79,97,78),33),
]:
    aim(pos,target,span)
    S.render.filepath = str(OUT / name)
    bpy.ops.render.render(write_still=True)
aim((-135,-160,265),(0,65,73),225)
S['Inspection view'] = 'P05 only visible; complete assembly remains in roam_drawer_cleanup_r18.blend'
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type == 'VIEW_3D':
            area.spaces.active.clip_end = 100
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'roam_drawer_cleanup_r18_inspection.blend'),compress=True)
print('R18_PREVIEWS_COMPLETE',flush=True)
