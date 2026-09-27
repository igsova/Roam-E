"""Rebuild only P05; preserve R17 and all its mating parts. Blender 100 mm/BU.

Run Blender --background --disable-autoexec --python THIS.py.
An independent validator must pass before the release ZIP is made.
"""
import ast
import hashlib
import json
import math
from functools import lru_cache
from collections import Counter
from pathlib import Path

import bpy
import bmesh
from mathutils import Matrix, Vector
from mathutils.geometry import delaunay_2d_cdt

ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT / 'mechanical/lower_controls_r17'
SOURCE = OLD / 'roam_lower_controls_r17.blend'
BASE = ROOT / 'mechanical/camera_closure_r16/roam_camera_closure_r16.blend'
OUT = ROOT / 'mechanical/drawer_cleanup_r18'
PACK = OUT / 'ROAM_R18_drawer_replacement'
OUT.mkdir(parents=True, exist_ok=True)
HASH = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
OLD_NAME = 'P05 | Curved switched battery drawer R17'
NEW_NAME = 'P05 | Continuous curved battery drawer R18'

for file, names in [
    ('create_split_shell_r07.py', {'clean', 'remove', 'clone', 'box', 'cyl',
        'mesh_object', 'boolean', 'bounds', 'metrics', 'snapshot'}),
    ('create_panel_bolts_r14.py', {'freeze'}),
    ('create_tof_solder_relief_r08.py', {'export'}),
    ('create_lower_controls_r17.py', {'frame', 'local', 'cb', 'cy', 'curve_y'}),
]:
    for node in ast.parse((ROOT / 'tools/cad' / file).read_text()).body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            exec(compile(ast.Module(body=[node], type_ignores=[]), '<R18 helper>', 'exec'))

source_hash, base_hash = HASH(SOURCE), HASH(BASE)
bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version = 0
S = bpy.context.scene
bpy.context.view_layer.update()
original = {o.name: snapshot(o) for o in S.objects}
original_names = set(original)
old = S.objects[OLD_NAME]
materials = list(old.data.materials)
collections = list(old.users_collection)
properties = dict(old.items())

# Recover the original tray/rails, not the defective R17 fascia mesh.
with bpy.data.libraries.load(str(BASE), link=False) as (src, dst):
    dst.objects = ['P05 | Battery drawer with integral fascia']
drawer = dst.objects[0]
S.collection.objects.link(drawer)
freeze(drawer)
remove(old)
drawer.name = NEW_NAME
drawer.data.materials.clear()
for mat in materials:
    drawer.data.materials.append(mat)
for c in list(drawer.users_collection):
    c.objects.unlink(drawer)
for c in collections:
    c.objects.link(drawer)
for key, value in properties.items():
    drawer[key] = value

LOCK_SEAT = curve_y(80, 78)

def fascia_y(x, z):
    """C2 blend into the unchanged 10 mm planar washer lands."""
    y = curve_y(x, z)
    for xx in [-80, 80]:
        t = max(0., min(1., (8. - math.hypot(x-xx, z-78)) / 3.))
        a = t*t*t*(t*(t*6-15)+10)
        y += a*(LOCK_SEAT-y)
    return y

def guide_relief(x,z):
    """Local inside-only clearance for unchanged, coarse-contoured R17 guides.

    Keep the full 3.6mm washer land; only the annular transition is relieved.
    Minimum nominal skin thickness remains 3.05mm, above the 3mm tray floor.
    """
    def smooth(t):
        t = max(0.,min(1.,t))
        return t*t*t*(t*(t*6-15)+10)
    return max(.55*smooth((math.hypot(x-xc,z-78)-5)/1.25)
               *smooth((9-math.hypot(x-xc,z-78))/1.) for xc in [-80,80])

def stations(lo, hi, fine_intervals):
    # 0.25 mm around screw blends; <=1 mm on the gently curved remainder.
    result = {lo, hi}
    for i in range(math.ceil(hi-lo)+1):
        result.add(min(hi, lo+i))
    for a, b in fine_intervals:
        for i in range(round((b-a)*4)+1):
            value = a+i/4
            if lo <= value <= hi:
                result.add(value)
    return sorted(result)

@lru_cache(maxsize=1)
def fascia_topology():
    """A single conforming surface, with circular topology around each land.

    No coplanar patch unions and no stair-stepped Cartesian land boundary.
    Constrained rings retain the exact flat-land and blend-limit boundaries.
    """
    pts,lookup,edges = [],{},set()
    def vertex(x,z):
        key = (round(max(-87,min(87,x)),7),round(z,7))
        if key not in lookup:
            lookup[key] = len(pts)
            pts.append(Vector(key))
        return lookup[key]
    for x in stations(-87,87,[]):
        for z in sorted(set(stations(54.5,99.5,[]))|{61.,98.}):
            if all(math.hypot(x-xc,z-78)>9 for xc in [-80,80]):
                vertex(x,z)
    for xc in [-80,80]:
        vertex(xc,78)
        radii = [1.7,5]+[5+i*.125 for i in range(1,25)]+[8.25,8.5,9]
        for r in radii:
            ring = [vertex(xc+r*math.cos(i*math.tau/256),
                           78+r*math.sin(i*math.tau/256)) for i in range(256)]
            for a,b in zip(ring,ring[1:]+ring[:1]):
                if a != b:
                    edges.add(tuple(sorted((a,b))))
    coords,_,triangles,_,_,_ = delaunay_2d_cdt(pts,list(edges),[],0,1e-6,False)
    assert all(len(f)==3 for f in triangles)
    counts = Counter(tuple(sorted((a,b))) for f in triangles
                     for a,b in zip(f,f[1:]+f[:1]))
    boundary = [edge for edge,count in counts.items() if count==1]
    assert all(count<=2 for count in counts.values())
    return coords,triangles,boundary

def curved_volume(name, filled=False):
    coords,triangles,boundary = fascia_topology()
    vertices = []
    for layer in [0,1]:
        for x,z in coords:
            y = fascia_y(x,z)
            vertices.append((x,(-50 if filled else y-3.6+guide_relief(x,z)) if layer==0 else y,z))
    n = len(coords)
    faces = [tuple(reversed(f)) for f in triangles]
    faces.extend(tuple(i+n for i in f) for f in triangles)
    faces.extend((a,b,b+n,a+n) for a,b in boundary)
    return mesh_object(name,vertices,faces)

def rounded_outline(cx, cz, w, h, r, steps=12):
    pts = []
    for ux, uz, start in [(1,1,0), (-1,1,90), (-1,-1,180), (1,-1,270)]:
        for i in range(steps+1):
            a = math.radians(start+i*90/steps)
            pts.append((cx+ux*(w/2-r)+r*math.cos(a),
                        cz+uz*(h/2-r)+r*math.sin(a)))
    return pts

def rounded_pocket_volume(name, rings):
    vertices = []
    for y, w, h, r in rings:
        vertices += [(x,y,z) for x,z in rounded_outline(-22,76,w,h,r)]
    n = len(vertices)//len(rings)
    faces = [tuple(reversed(range(n))), tuple(range(len(vertices)-n,len(vertices)))]
    for k in range(len(rings)-1):
        for i in range(n):
            j = (i+1)%n
            faces.append((k*n+i,k*n+j,(k+1)*n+j,(k+1)*n+i))
    return mesh_object(name, vertices, faces)

print('R18_BUILD: original tray, continuous floor, refined fascia', flush=True)
boolean(drawer, box('_old flat fascia', (0,147,78), (220,100,80)))
# Remove BOTH the inherited pull backing/flat strip and the old recess.
# The tray top, side walls, rails and bolt interfaces are outside this cut.
boolean(drawer, box('_legacy central handle remnants', (0,102.5,90.75), (138,35,58.5)))
envelope = curved_volume('_R18 outside envelope', filled=True)
boolean(drawer, clone(envelope, '_trim inherited tray'), 'INTERSECT')
floor = box('_continuous front floor', (0,101,60), (147,11,3))
boolean(floor, clone(envelope, '_floor contour'), 'INTERSECT')
boolean(drawer, floor, 'UNION')
boolean(drawer, curved_volume('_refined curved fascia'), 'UNION')
remove(envelope)

# A shallow rounded cup: 2.4 mm closed backing instead of the bulky R17 block.
# The one-mm rear chamfer and rounded outline remove sharp projecting corners.
boolean(drawer, rounded_pocket_volume('_integrated pull cup', [
    (94.6,38,13,3), (95.6,40,15,4), (104,40,15,4)]), 'UNION')
boolean(drawer, rounded_pocket_volume('_rounded finger recess', [
    (97,34,9,2), (120,34,9,2)]))

for x in [-12,12]:
    boolean(drawer, box('_caster travel relief', (x,38,55.2), (7,78,9.4)))
for x in [-80,80]:
    mat = frame((x,LOCK_SEAT,78), (1,0,0), (0,0,1), (0,-1,0))
    boolean(drawer, cy(mat,(0,0,1),1.7,40))

# Identical dimensions and datum to the existing R17 BATTERY switch interface.
switch_mat = frame((40,curve_y(40,78)+.2,78), (1,0,0), (0,0,1), (0,-1,0))
boolean(drawer, cy(switch_mat,(0,0,1),15,5), 'UNION')
boolean(drawer, cy(switch_mat,(0,0,-5),16,10))
boolean(drawer, cy(switch_mat,(0,0,13.6),12.2,22.8))
boolean(drawer, cy(switch_mat,(0,0,0),10.25,55))
clean(drawer)
for p in drawer.data.polygons:
    p.material_index = 0
    p.use_smooth = False  # Preview and STL reveal actual geometry, not shade smoothing.
drawer['R18 changes'] = 'Continuous 3mm front floor; concentric C2 washer-pad blends with inside guide relief; rounded closed finger cup with 2.4mm backing.'
drawer['R18 compatibility'] = 'Use unchanged R17 P06L/P06R guides, lower panels, M3x12 locks and switch. Not pre-R17 guides.'
drawer['Manufacture'] = 'PRINT; PETG; inspect local supports and physical fit.'

solid = metrics(drawer)
assert solid['components'] == 1 and solid['nonmanifold_edges'] == 0 and solid['loose_vertices'] == 0 and solid['volume_mm3'] > 0, solid
assert set(S.objects.keys()) == (original_names-{OLD_NAME})|{NEW_NAME}
unexpected = [n for n,h in original.items() if n != OLD_NAME and snapshot(S.objects[n]) != h]
assert not unexpected, unexpected
S['Mechanical revision'] = 'R18 drawer cleanup; all other R17 parts unchanged'
S['Revision'] = S['Mechanical revision']
bpy.ops.object.select_all(action='DESELECT')
drawer.hide_set(False)
drawer.select_set(True)
bpy.context.view_layer.objects.active = drawer
master = OUT / 'roam_drawer_cleanup_r18.blend'
bpy.ops.wm.save_as_mainfile(filepath=str(master), compress=True)

stl = PACK / '01_rigid/P05_curved_battery_drawer_R18_qty1.stl'
check = export(drawer, stl, Matrix.Identity(4))
report = {
    'revision':'R18', 'source_R17_sha256':source_hash, 'base_R16_sha256':base_hash,
    'master_sha256':HASH(master), 'changed_part':NEW_NAME,
    'unchanged_object_count':len(original)-1, 'unexpected_original_changes':unexpected,
    'solid':solid, 'lock_seat_y_mm':LOCK_SEAT,
    'switch_matrix':[list(r) for r in switch_mat],
    'parameters':{'floor_thickness_mm':3, 'floor_underside_z_mm':58.5,
                  'floor_top_z_mm':61.5, 'pull_backing_mm':2.4,
                  'pull_opening_mm':[34,9], 'fascia_thickness_mm':3.6,
                  'guide_relief_max_mm':.55, 'guide_relief_min_skin_mm':3.05,
                  'washer_land_diameter_mm':10, 'blend_width_mm':3,
                  'blend_radial_spacing_mm':.125, 'blend_angular_segments':256,
                  'switch_bore_mm':20.5,
                  'switch_seat_mm':2.2},
    'parts':[{'id':'P05', 'quantity':1, 'file':str(stl.relative_to(PACK)),
              'sha256':HASH(stl), 'reimport_check':check}],
    'status':'BUILT; independent validation pending',
}
(OUT / 'geometry_checks.json').write_text(json.dumps(report, indent=2))
assert HASH(SOURCE) == source_hash and HASH(BASE) == base_hash
print('R18_BUILD_COMPLETE', json.dumps(solid), flush=True)
