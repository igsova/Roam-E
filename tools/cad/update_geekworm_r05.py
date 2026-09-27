"""R04 -> R05 reversible Geekworm converter update, run with Blender 5.2.

All new modeling parameters are millimetres; original scene is 100 mm/BU.
Original .blend and R04 print package are opened/read, never overwritten.
Usage: Blender --background --disable-autoexec --python THIS.py --
  --source /path/roam_tof.blend --print-pack /path/ROAM_STL_R04_K1C
  --out /path/geekworm_r05 [--render]
"""
import argparse
import csv
import hashlib
import json
import math
import shutil
import sys
import zipfile
from pathlib import Path

import bpy
import bmesh
from mathutils import Matrix, Vector

P = argparse.ArgumentParser()
P.add_argument('--source', type=Path, required=True)
P.add_argument('--print-pack', type=Path, required=True)
P.add_argument('--out', type=Path, required=True)
P.add_argument('--render', action='store_true')
A = P.parse_args(sys.argv[sys.argv.index('--') + 1:])
A.out.mkdir(parents=True, exist_ok=True)
CFG = json.loads((Path(__file__).resolve().parents[2] / 'mechanical/geekworm_r05/dimensions.json').read_text())
SCALE = 100.0
AD = CFG['adapter_mm']
EST = CFG['estimated_mm_pending_delivery']
CX, CY = AD['centre_xy']
Z0 = AD['bottom_z']
FLOOR = AD['floor_thickness']
MODULE_BOTTOM = Z0 + FLOOR
SOURCE_HASH = hashlib.sha256(A.source.read_bytes()).hexdigest()
bpy.ops.wm.open_mainfile(filepath=str(A.source))
bpy.context.preferences.filepaths.save_version = 0
S = bpy.context.scene
assert abs(S.unit_settings.scale_length - 0.1) < 1e-6
assert 'ToF fit' in S, 'Use R04 assembled ToF source, not the older R03/downloaded exploded source.'
assert not S.get('Exploded view'), 'Use assembled source.'
S.render.engine = 'CYCLES'
S.cycles.device = 'CPU'
S.cycles.samples = 24
S.render.resolution_x = 1400
S.render.resolution_y = 1100
S.render.resolution_percentage = 100
S.render.image_settings.file_format = 'PNG'
S.render.film_transparent = False
S.render.use_file_extension = True
S.world.color = (0.15, 0.15, 0.15)

def collection(name):
    c = bpy.data.collections.get(name) or bpy.data.collections.new(name)
    if c.name not in S.collection.children:
        S.collection.children.link(c)
    return c

ELEC = collection('10 Geekworm - purchased reference, NOT FOR PRINT')
MOUNT = collection('11 R05 Geekworm - printable adapter and retainers')
HARD = collection('12 R05 Geekworm - purchased fasteners and pads')
GUIDES = collection('13 R05 Geekworm - clearance guides, hidden')

def material(name, color, metal=0, rough=0.45):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = (*color, 1)
    m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = (*color, 1)
    p.inputs['Metallic'].default_value = metal
    p.inputs['Roughness'].default_value = rough
    return m

MAT = {
    'pcb': material('R05 | blue solder mask', (0.018, 0.095, 0.22), .25),
    'base': material('R05 | OEM black plastic', (.017, .022, .026)),
    'fan': material('R05 | satin black fan', (.028, .032, .038), .12, .29),
    'metal': material('R05 | connector stainless steel', (.48, .52, .56), .82, .25),
    'dark': material('R05 | socket interior', (.006, .008, .012)),
    'green': material('R05 | 3.81 terminal green', (.03, .47, .20)),
    'white': material('R05 | silkscreen', (.80, .85, .86)),
    'gold': material('R05 | gold copper pads', (.80, .47, .10), .7),
    'yellow': material('R05 | trim jumper yellow', (.95, .62, .02)),
    'red': material('R05 | fan wire red', (.55, .012, .016)),
    'mount': material('R05 | printed mount teal', (.018, .30, .32), .05),
    'pad': material('R05 | soft pad', (.06, .08, .08)),
}

def rel(x=0, y=0, z=0):
    return (CX + x, CY + y, Z0 + z)

def finish(o, name, mat='base', coll=ELEC):
    o.name = name
    for c in list(o.users_collection):
        c.objects.unlink(o)
    coll.objects.link(o)
    o.data.materials.clear()
    o.data.materials.append(MAT[mat])
    return o

def box(name, pos, size, mat='base', coll=ELEC, bevel=0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=Vector(pos) / SCALE)
    o = finish(bpy.context.object, name, mat, coll)
    o.dimensions = Vector(size) / SCALE
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        mod = o.modifiers.new('Soft molded edges', 'BEVEL')
        mod.width = bevel / SCALE
        mod.segments = 3
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.modifier_apply(modifier=mod.name)
    return o

def cylinder(name, pos, radius, height, mat='metal', coll=ELEC, vertices=64):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius / SCALE,
                                      depth=height / SCALE, location=Vector(pos) / SCALE)
    return finish(bpy.context.object, name, mat, coll)

def clean(o, mm=False):
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=0.00001 if mm else 1e-7)
    bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=0.000001 if mm else 1e-8)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(o.data)
    bm.free()
    o.data.update()

def boolean(a, b, op='DIFFERENCE', delete=True):
    bpy.context.view_layer.update()
    mod = a.modifiers.new('R05 ' + op, 'BOOLEAN')
    mod.operation = op
    mod.solver = 'EXACT'
    mod.object = b
    bpy.context.view_layer.objects.active = a
    bpy.ops.object.modifier_apply(modifier=mod.name)
    if delete:
        bpy.data.objects.remove(b, do_unlink=True)
    clean(a)
    return a

def bore(o, x, y, low, high, diameter):
    return boolean(o, cylinder('_tool bore', (x, y, (low + high) / 2), diameter / 2, high - low))

def rounded_xy(name, pos, dims, radius, mat='base', coll=ELEC):
    # Rounded XY perimeter without beveling the mating top and bottom planes.
    w, d, h = dims
    pts = []
    for x, y, a0 in [(w/2-radius, d/2-radius, 0), (-w/2+radius, d/2-radius, 90),
                     (-w/2+radius, -d/2+radius, 180), (w/2-radius, -d/2+radius, 270)]:
        for i in range(13):
            a = math.radians(a0 + i * 90 / 12)
            pts.append((x + radius * math.cos(a), y + radius * math.sin(a)))
    n = len(pts)
    verts = [(x/SCALE, y/SCALE, z/SCALE) for z in (-h/2, h/2) for x, y in pts]
    faces = [tuple(reversed(range(n))), tuple(range(n, 2*n))]
    faces += [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    o = bpy.data.objects.new(name, me)
    ELEC.objects.link(o)
    o.location = Vector(pos)/SCALE
    finish(o, name, mat, coll)
    clean(o)
    return o

def wire(name, points, radius, mat, coll=ELEC):
    cu = bpy.data.curves.new(name, 'CURVE')
    cu.dimensions = '3D'
    cu.resolution_u = 16
    cu.bevel_depth = radius / SCALE
    cu.bevel_resolution = 3
    sp = cu.splines.new('BEZIER')
    sp.bezier_points.add(len(points)-1)
    for p, co in zip(sp.bezier_points, points):
        p.co = Vector(co) / SCALE
        p.handle_left_type = p.handle_right_type = 'AUTO'
    o = bpy.data.objects.new(name, cu)
    coll.objects.link(o)
    cu.materials.append(MAT[mat])
    return o

def text(name, words, pos, size, mat='white', coll=ELEC, rotation=0):
    cu = bpy.data.curves.new(name, 'FONT')
    cu.body = words
    cu.size = size / SCALE
    cu.align_x = 'CENTER'
    cu.extrude = .006/SCALE
    o = bpy.data.objects.new(name, cu)
    coll.objects.link(o)
    o.location = Vector(pos)/SCALE
    o.rotation_euler.z = math.radians(rotation)
    cu.materials.append(MAT[mat])
    return o

def bb(o):
    bpy.context.view_layer.update()
    return [(min((o.matrix_world @ Vector(p))[i]*SCALE for p in o.bound_box),
             max((o.matrix_world @ Vector(p))[i]*SCALE for p in o.bound_box)) for i in range(3)]

def snapshot(o):
    # Detect inadvertent geometry/transform edits to unrelated R04 objects.
    h = hashlib.sha256()
    h.update(str(tuple(tuple(r) for r in o.matrix_world)).encode())
    if o.type == 'MESH':
        for v in o.data.vertices:
            h.update(str(tuple(v.co)).encode())
        for p in o.data.polygons:
            h.update(str(tuple(p.vertices)).encode())
    return h.hexdigest()

ORIGINAL = {o.name: snapshot(o) for o in S.objects}
old_names = [o.name for o in S.objects if o.name == 'Component | Buck_Converter'
             or o.name == 'Buck_Converter | Removable carrier'
             or o.name.startswith(('Buck_Converter | PCB spacer', 'N_Buck_Converter_', 'Label | BUCK CONVERTER'))]
for name in old_names:
    bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)

# Retain the two original S01 lower spacers and the unmodified P04 deck.
adapter = rounded_xy('C02 | Geekworm base cradle - reuses R04 deck holes',
                     rel(z=FLOOR/2), (*AD['floor_xy'], FLOOR), 2.2, 'mount', MOUNT)
for x in (-18, 18):
    for y in (-26.5, 26.5):
        boolean(adapter, cylinder('_clamp mounting ear', rel(x, y, FLOOR/2), 4.7, FLOOR, 'mount', MOUNT), 'UNION')
# Clearance notch avoids the neighbouring IMU carrier, without moving the IMU.
boolean(adapter, box('_IMU corner clearance', (-6, -35, Z0+1.5), (22, 33, 8)))
for x, y in AD['existing_deck_holes_xy']:
    low = AD['support_bottom_z']
    boolean(adapter, cylinder('_integrated support leg', (x,y,(low+Z0+.1)/2),4,(Z0+.1)-low,'mount',MOUNT),'UNION')
for x, y in AD['existing_deck_holes_xy']:
    bore(adapter, x, y, AD['support_bottom_z']-1, Z0+4, AD['mount_clearance_diameter'])
    bpy.ops.mesh.primitive_cone_add(vertices=64, radius1=1.45/SCALE, radius2=2.85/SCALE,
                                    depth=1.4/SCALE, location=(x/SCALE, y/SCALE, (Z0+2.3)/SCALE))
    boolean(adapter, bpy.context.object)
for x in (-18, 18):
    for y in (-26.5, 26.5):
        bore(adapter, CX+x, CY+y, Z0-1, Z0+4, 2.9)
        # Underside hex pockets: put nuts in before fastening cradle to deck.
        nuttool = cylinder('_M2.5 captive nut pocket', rel(x, y, 1.1), 5.4/math.sqrt(3), 2.21, vertices=6)
        nuttool.rotation_euler.z = math.radians(30)
        boolean(adapter, nuttool)
adapter['Print ID'] = 'C02'
adapter['Orientation'] = 'Print broad top face down, with the integral legs pointing up; STL already oriented.'
adapter['Mount'] = 'Original two M2.5 deck axes and 2.5 mm spacers; 4 mm integral feet; flush M2.5x16 countersunk screws. OEM PCB holes are NOT used.'
adapter['Pending fit'] = '59x44 OEM base; verify supplied base height 5 mm, corner shape, retainer ledge and tolerances.'

clamps, pads = [], []
for sign in (-1, 1):
    # Feet land on cradle ears. Lips grip only the outer plastic-base rim.
    clamp = box('C08 | Geekworm base edge retainer ' + ('front' if sign < 0 else 'rear'),
                rel(0, sign*25.6, 6.6), (46, 6.4, 7.2), 'mount', MOUNT)
    boolean(clamp, box('_retainer lip', rel(0, sign*22.2, 9.4), (46, 2.8, 1.6), 'mount', MOUNT), 'UNION')
    for x in (-18, 18):
        bore(clamp, CX+x, CY+sign*26.5, Z0+2, Z0+11, 2.9)
    clamp['Print ID'] = 'C08'
    clamp['Orientation'] = 'Print top face down; flip 180 degrees around X.'
    clamps.append(clamp)
    pad = box('F05 | Geekworm compliant edge strip ' + str(sign), rel(0, sign*21.4, 8.3),
              (46, 1.2, .6), 'pad', HARD)
    pad['Print ID'] = 'F05'
    pad['Material'] = 'Prefer cut 0.6 mm compliant tape; not rigid plastic.'
    pads.append(pad)

# Purchased OEM converter reference: provided dimensions + explicitly estimated details.
base = rounded_xy('Geekworm | Supplied 59x44 base - estimated 5 mm high',
                  rel(z=5.5), (59, 44, 5), 4, 'base')
pcb = rounded_xy('Geekworm | 55x40 PCB - component layout approximate',
                 rel(z=8.2), (55, 40, 1.6), 3, 'pcb')
for x in (-24, 24):
    for y in (-16.5, 16.5):
        bore(base, CX+x, CY+y, Z0+2, Z0+10, 3)
        bore(pcb, CX+x, CY+y, Z0+6, Z0+11, 3)
        ring = cylinder('Geekworm | Mount pad - unmeasured pattern', rel(x, y, 9.04), 2.1, .07, 'gold')
        bore(ring, CX+x, CY+y, Z0+8, Z0+10, 3)

# Fan's upper face is the supplied 22 mm assembly-height datum.
fan = rounded_xy('Geekworm | 30x30x7 fan frame', rel(z=21.5), (30, 30, 7), 2, 'fan')
bore(fan, CX, CY, Z0+17, Z0+26, 27.4)
for x in (-12, 12):
    for y in (-12, 12):
        bore(fan, CX+x, CY+y, Z0+17, Z0+26, 2.2)
        cylinder('Geekworm | Fan support post', rel(x, y, 13.5), 1.8, 9, 'dark')
        screw = cylinder('Geekworm | Fan screw head', rel(x, y, 24.45), 1.7, .6, 'metal')
        boolean(screw, box('_fan screw slot', rel(x, y, 24.8), (2.5, .55, .6)))
cylinder('Geekworm | Fan rotor', rel(z=22), 6.0, 4.4, 'fan')
for i in range(7):
    a = math.radians(i*360/7)
    verts = []
    for z in (20.0, 21.1):
        for r, da in ((5.7, -9), (12.9, -29), (13.1, -9), (7.0, 16)):
            t = a + math.radians(da)
            verts.append((r*math.cos(t)/SCALE, r*math.sin(t)/SCALE, z/SCALE))
    me = bpy.data.meshes.new('fan swept blade')
    me.from_pydata(verts, [], [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
    o = bpy.data.objects.new('Geekworm | Fan swept blade', me)
    ELEC.objects.link(o)
    o.location = Vector(rel())/SCALE
    finish(o, o.name, 'fan')

def usb_c(name, x, y, outward):
    shell = box(name + ' shell', rel(x, y, 11.1), (7.4, 9.0, 3.4), 'metal', bevel=.6)
    cutx = x + outward*1.0
    boolean(shell, box('_usb-c mouth', rel(cutx, y, 11.1), (7.6, 7.5, 2.2), bevel=.45))
    box(name + ' tongue', rel(x, y, 11.1), (5.2, 5.8, .6), 'dark', bevel=.15)
    return shell

usb_c('Geekworm | USB-C PD INPUT', -25, 9, -1)
usb_c('Geekworm | USB-C PD OUTPUT', 25, -11, 1)
usb_a = box('Geekworm | USB-A OUTPUT shell', rel(23.9, 7, 13.2), (9.0, 13.8, 8.4), 'metal', bevel=.4)
boolean(usb_a, box('_USB-A mouth', rel(25.6, 7, 13.2), (10, 11.8, 6.4), bevel=.2))
box('Geekworm | USB-A tongue', rel(23.5, 7, 11.3), (7, 10.6, 1.1), 'dark')
for y in (3.8, 5.8, 7.8, 9.8):
    box('Geekworm | USB-A contact', rel(25, y, 12), (4, .65, .15), 'gold')

# Terminal protrusion is an estimate; dimensioned photo depicts the barrel variant.
term = box('Geekworm | 3.81 mm 2-pin removable terminal - estimated envelope',
           rel(-29.5, -6, 14), (18, 11, 11), 'green', bevel=.7)
boolean(term, box('_terminal seam', rel(-28.5, -6, 14.3), (.3, 12, 11)))
# Keep two sub-blocks as separate references (seam makes terminal removable).
for y in (-6-1.905, -6+1.905):
    cylinder('Geekworm | Terminal screw', rel(-32, y, 19.4), 1.25, .25, 'metal')
    box('Geekworm | Terminal wire entry', rel(-38.52, y, 12.8), (.12, 2.55, 3.4), 'dark', bevel=.1)
box('Geekworm | Fan JST connector', rel(-17.5, -12, 12.5), (4, 6, 6.5), 'white', bevel=.3)
for y in (-13.1, -10.9):
    wire('Geekworm | Fan lead', [rel(-17.5,y,15), rel(-21,y,20), rel(-19,0,23), rel(-14,8,22)], .42,
         'red' if y < -12 else 'dark')
box('Geekworm | Voltage trim jumper - reference only', rel(-10, -18, 11.5), (5, 2.2, 5), 'yellow', bevel=.25)
box('Geekworm | Power switch housing', rel(11, 16.3, 11.2), (10, 4.8, 4), 'metal', bevel=.3)
box('Geekworm | Power switch slider', rel(13, 16.3, 13.1), (3, 3, 1.2), 'dark')
for x, y, w, h in [(-19,2,5,4),(18,-2,4,5),(-5,17,4,3),(2,-17,5,3),(18,-15,3,2)]:
    box('Geekworm | Reference power component', rel(x,y,10), (w,h,1.8), 'dark', bevel=.2)
for x in (-21,-17,-13,-9,-5):
    box('Geekworm | SMD capacitor', rel(x,17,9.5), (1.9,1.0,.8), 'gold', bevel=.1)
text('Geekworm | Board model legend', 'PI5-5V5A-PD-24', rel(1,-18.5,9.07), 1.3)
text('Geekworm | Input legend', '9-24V DC', rel(-22,-15.5,9.07), 1.0)
text('Geekworm | Output legend', '5V 5A PD', rel(21,-17.5,9.07), 1.1)
base['Reference status'] = '59x44 base and 22 mm nominal assembly height from supplied image; terminal overhang, base height and holes unmeasured.'
base['Not for fabrication'] = True

# Purchased screws/nuts: schematic smooth shafts, not modeled threads.
for i, (x, y) in enumerate(AD['existing_deck_holes_xy'], 1):
    cylinder(f'N_Buck_R05_{i} | M2.5x16 countersunk shaft', (x,y,Z0+3-8), 1.25, 16, coll=HARD)
    bpy.ops.mesh.primitive_cone_add(vertices=48, radius1=1.25/SCALE, radius2=2.5/SCALE,
                                   depth=1.25/SCALE, location=(x/SCALE,y/SCALE,(Z0+2.375)/SCALE))
    finish(bpy.context.object, f'N_Buck_R05_{i} | flush countersunk head', 'metal', HARD)
    n = cylinder(f'N_Buck_R05_{i} | reused underside M2.5 nut', (x,y,108), 5/math.sqrt(3), 2, coll=HARD, vertices=6)
    bore(n,x,y,106,110,2.5)
    w = cylinder(f'N_Buck_R05_{i} | reused underside washer', (x,y,109.25),2.5,.5,coll=HARD)
    bore(w,x,y,108,110,2.5)
for i,(x,y) in enumerate([(x,y) for x in (-18,18) for y in (-26.5,26.5)],1):
    cylinder(f'U{i} | M2.5x10 clamp screw shaft', rel(x,y,5.2),1.25,10,coll=HARD)
    cylinder(f'U{i} | clamp screw head', rel(x,y,11.25),2.25,2.1,coll=HARD)
    n=cylinder(f'U{i} | captive M2.5 nut',rel(x,y,1.1),5/math.sqrt(3),2,coll=HARD,vertices=6)
    n.rotation_euler.z=math.radians(30)
    bore(n,CX+x,CY+y,Z0-1,Z0+3,2.5)

# Keep-out guides are intentionally not printed or rendered in normal views.
air = box('CHECK | Fan intake 30x30, first 12 mm clear', rel(z=31), (30,30,12), 'mount', GUIDES)
usb = box('CHECK | USB-C output plug, 24 mm body estimate', rel(40.7,-11,11.1), (24,12,9), 'yellow', GUIDES)
dc = box('CHECK | DC wire approach estimate', rel(-43,-6,14), (9,11,10), 'red', GUIDES)
envelope = box('REFERENCE ONLY | Geekworm conservative envelope',rel(z=14),(59,44,22),'base',GUIDES)
boolean(envelope,box('_terminal envelope',rel(-29.5,-6,14),(18,11,11)), 'UNION')
envelope['Purpose'] = 'Conservative packaging mock-up, NOT a functional or printable electronics part.'

# Turn only the purchased module and its port guides; cradle/fasteners stay on deck axes.
pivot = Vector((CX/SCALE, CY/SCALE, 0))
turn = Matrix.Translation(pivot) @ Matrix.Rotation(math.radians(AD['module_rotation_degrees']),4,'Z') @ Matrix.Translation(-pivot)
for obj in list(ELEC.objects) + list(GUIDES.objects):
    obj.matrix_world = turn @ obj.matrix_world
bpy.context.view_layer.update()
BOUNDS_CACHE = {o.name: bb(o) for o in S.objects if o.type == 'MESH'}

def closed_mesh(o):
    bm=bmesh.new();bm.from_object(o,bpy.context.evaluated_depsgraph_get())
    closed=bool(bm.faces) and all(e.is_manifold for e in bm.edges)
    bm.free()
    return closed

CLOSED_CACHE = {o.name: closed_mesh(o) for o in S.objects if o.type=='MESH'}

def intersection_volume(a, b):
    ba, bbx = BOUNDS_CACHE[a.name], BOUNDS_CACHE[b.name]
    if not all(min(ba[i][1],bbx[i][1])-max(ba[i][0],bbx[i][0]) > .015 for i in range(3)):
        return 0.0
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(a.evaluated_get(dg))
    tmp = bpy.data.objects.new('_QA intersection',me)
    S.collection.objects.link(tmp)
    tmp.matrix_world = a.matrix_world.copy()
    mod = tmp.modifiers.new('Intersection','BOOLEAN')
    mod.operation='INTERSECT'
    # Manifold solver avoids an Exact-solver false containment result verified
    # with nearest-surface/ray tests on the closed R04 sensor frame.
    mod.solver='MANIFOLD' if CLOSED_CACHE[a.name] and CLOSED_CACHE[b.name] else 'EXACT'
    mod.object=b
    bpy.context.view_layer.objects.active=tmp
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bm=bmesh.new();bm.from_mesh(tmp.data)
    bm.transform(tmp.matrix_world)
    vol=abs(bm.calc_volume(signed=True))*SCALE**3
    bm.free();bpy.data.objects.remove(tmp,do_unlink=True)
    return vol

# Check unchanged assembled environment, not exploded/display geometry.
exclude = {'P04 | Removable electronics deck', 'Buck_Converter | Lower spacer 1', 'Buck_Converter | Lower spacer 2'}
obstacles = [o for o in S.objects if o.name in ORIGINAL and o.name not in old_names and o.name not in exclude
             and o.type=='MESH' and not any(c.name.startswith(('06','08')) for c in o.users_collection)]
clashes=[]
checked_new = [adapter,*clamps] + [o for o in ELEC.objects if o.type=='MESH'] + [usb,dc,air]
for obj in checked_new:
    for other in obstacles:
        v=intersection_volume(obj,other)
        if v>.03:
            clashes.append({'new':obj.name,'existing':other.name,'intersection_mm3':round(v,4)})
internal_clashes=[]
for obj in [adapter,*clamps,*pads]:
    for other in [o for o in ELEC.objects if o.type=='MESH']:
        v=intersection_volume(obj,other)
        if v>.03:
            internal_clashes.append({'mount':obj.name,'module':other.name,'intersection_mm3':round(v,4)})
for i,obj in enumerate([adapter,*clamps,*pads]):
    for other in [adapter,*clamps,*pads][i+1:]:
        v=intersection_volume(obj,other)
        if v>.03:
            internal_clashes.append({'mount':obj.name,'mount_other':other.name,'intersection_mm3':round(v,4)})

def duplicate_error(first, second):
    # Normalize translation and rotate the opposite edge 180 degrees in XY.
    from mathutils.kdtree import KDTree
    clouds=[]
    for obj,rotate in ((first,False),(second,True)):
        points=[obj.matrix_world @ v.co * SCALE for v in obj.data.vertices]
        lo=Vector(tuple(min(p[i] for p in points) for i in range(3)))
        hi=Vector(tuple(max(p[i] for p in points) for i in range(3)))
        points=[p-(lo+hi)/2 for p in points]
        if rotate:points=[Vector((-p.x,-p.y,p.z)) for p in points]
        clouds.append(points)
    err=0
    for src,dst in (clouds,clouds[::-1]):
        tree=KDTree(len(dst))
        for i,p in enumerate(dst):tree.insert(p,i)
        tree.balance()
        err=max(err,max(tree.find(p)[2] for p in src))
    return err

duplicate_checks={'C08_max_vertex_difference_mm':duplicate_error(*clamps),
                  'F05_max_vertex_difference_mm':duplicate_error(*pads)}
assert max(duplicate_checks.values())<.01,duplicate_checks
preserved=[n for n in ORIGINAL if n not in old_names]
changes=[n for n in preserved if n not in bpy.data.objects or snapshot(bpy.data.objects[n]) != ORIGINAL[n]]
assert not changes, ('Unrelated R04 geometry changed',changes)
REPORT = {'revision':CFG['revision'],'source':str(A.source),'source_sha256':SOURCE_HASH,
          'preserved_R04_objects':len(preserved),'unexpected_changes':changes,
          'removed_old_buck_objects':old_names,'clashes_against_existing_geometry':clashes,
          'mount_to_module_and_mount_internal_clashes':internal_clashes,'duplicate_part_checks':duplicate_checks,
          'fan_clearance_checked_mm':12,'new_meshes_and_guides_checked':len(checked_new),'dimensions':CFG,
          'limitations':['Photographic estimates are not physical fit measurements.',
                        'Intersections test all modeled new converter meshes, cradle, retainers and three guides against existing environment meshes; cables/strength/thermal performance remain untested.']}
(A.out/'geometry_checks.json').write_text(json.dumps(REPORT,indent=2))
print('CLASHES',json.dumps(clashes),flush=True)
if clashes or internal_clashes:
    bpy.ops.wm.save_as_mainfile(filepath=str(A.out/'geekworm_r05_fit_review.blend'))
    raise RuntimeError('Resolve clashes before exporting final R05 files.')

# Export printable replacements only; preserve all unrelated R04 STLs bit-for-bit.
PACK = A.out/'ROAM_STL_R05_K1C'
PACK.mkdir(exist_ok=True)
old_manifest=json.loads((A.print_pack/'manifest.json').read_text())
records=[]
for r in old_manifest['parts']:
    if r['id'] in ('C02','S05'):
        continue
    path=PACK/r['file'];path.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(A.print_pack/r['file'],path)
    assert hashlib.sha256(path.read_bytes()).hexdigest()==r['sha256']
    records.append(r)

def metrics(o):
    bm=bmesh.new();bm.from_mesh(o.data)
    pending=set(bm.verts);parts=0
    while pending:
        parts+=1;stack=[pending.pop()]
        while stack:
            v=stack.pop()
            for e in v.link_edges:
                q=e.other_vert(v)
                if q in pending:pending.remove(q);stack.append(q)
    result={'components':parts,'nonmanifold_edges':sum(not e.is_manifold for e in bm.edges),
            'loose_vertices':sum(not v.link_faces for v in bm.verts),'volume_mm3':bm.calc_volume(signed=True)}
    bm.free()
    bounds=[(min(v.co[i] for v in o.data.vertices),max(v.co[i] for v in o.data.vertices)) for i in range(3)]
    result['dimensions_mm']=[b-a for a,b in bounds];result['min_z_mm']=bounds[2][0]
    return result

def export(obj, path, flip=False):
    dg=bpy.context.evaluated_depsgraph_get()
    me=bpy.data.meshes.new_from_object(obj.evaluated_get(dg))
    me.transform(Matrix.Scale(SCALE,4)@obj.matrix_world)
    if flip:me.transform(Matrix.Rotation(math.pi,4,'X'))
    lim=[(min(v.co[i] for v in me.vertices),max(v.co[i] for v in me.vertices)) for i in range(3)]
    me.transform(Matrix.Translation(Vector((-(lim[0][0]+lim[0][1])/2,-(lim[1][0]+lim[1][1])/2,-lim[2][0]))))
    tmp=bpy.data.objects.new('_EXPORT',me);S.collection.objects.link(tmp);clean(tmp,True)
    expected=metrics(tmp)
    assert expected['components']==1 and expected['nonmanifold_edges']==0 and expected['volume_mm3']>0,expected
    path.parent.mkdir(parents=True,exist_ok=True)
    bpy.ops.object.select_all(action='DESELECT');tmp.select_set(True);bpy.context.view_layer.objects.active=tmp
    bpy.ops.wm.stl_export(filepath=str(path),export_selected_objects=True,use_scene_unit=False,global_scale=1,forward_axis='Y',up_axis='Z')
    bpy.ops.object.select_all(action='DESELECT');bpy.ops.wm.stl_import(filepath=str(path),use_facet_normal=False)
    imported=bpy.context.object;actual=metrics(imported)
    assert actual['components']==1 and actual['nonmanifold_edges']==0 and actual['loose_vertices']==0 and actual['volume_mm3']>0,actual
    assert abs(actual['min_z_mm'])<.001
    assert all(abs(a-b)<.002 for a,b in zip(expected['dimensions_mm'],actual['dimensions_mm']))
    assert all(a<b for a,b in zip(actual['dimensions_mm'],[220,220,250]))
    bpy.data.objects.remove(imported,do_unlink=True);bpy.data.objects.remove(tmp,do_unlink=True)
    return actual

for id,label,obj,qty,folder,flip,note in [
    ('C02','geekworm_base_cradle',adapter,1,'01_rigid',True,'R05 replacement: broad top face on bed, integral legs up. Reuses original two S01 lower spacers. Install captive nuts and countersunk mounting screws BEFORE placing converter.'),
    ('C08','geekworm_edge_retainer',clamps[0],2,'01_rigid',True,'Two identical edge retainers. Print flat upper face down. Grip OEM plastic base only; verify base rim before tightening.'),
    ('F05','geekworm_edge_pad',pads[0],2,'03_flexible_optional',False,'Prefer cut 0.6 mm compliant tape. Prototype STL template; do not use rigid material.')]:
    filename=f'{folder}/{id}_{label}_qty{qty}.stl'
    check=export(obj,PACK/filename,flip)
    r={'id':id,'label':label,'sources':[obj.name],'folder':folder,'quantity':qty,'file':filename,
       'support':'No expected; inspect hex-pocket bridges and countersinks in slicer', 'note':note,
       'reimport_check':check,'sha256':hashlib.sha256((PACK/filename).read_bytes()).hexdigest(),
       'brim_4mm_fits_K1C':all(d+8<=220 for d in check['dimensions_mm'][:2])}
    records.append(r)
    print('EXPORTED',id,json.dumps(check),flush=True)
export(envelope,A.out/'reference_only/geekworm_clearance_envelope_NOT_A_PRINT_PART.stl')
manifest={**old_manifest,'revision':'R05 Geekworm update - physical fit pending delivery',
          'source':'roam_geekworm_r05.blend','unique_stl_files':len(records),
          'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),'parts':records,
          'unchanged_parts_source':str(A.print_pack),'replaces':['R04 C02','R04 S05 (no longer needed)']}
(PACK/'manifest.json').write_text(json.dumps(manifest,indent=2))
with (PACK/'PRINT_PARTS.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
    for r in records:w.writerow([r['id'],r['label'],r['quantity'],r['file'],*[round(v,3) for v in r['reimport_check']['dimensions_mm']],r['support'],r['note']])
# Keep inherited instructions under a clearly historical folder, not as stale R05 instructions.
for filename in ('START_HERE.md','ASSEMBLY_GUIDE.md','HARDWARE.csv','ROAM_ToF_update.md','mechanical_joint_locations.json','tof_geometry_checks.json',
                 'K1C_cover_orientations.png','roam_mechanical_structure.png','roam_sensor_mounts.png',
                 'roam_tof_inside.png','roam_tof_front.png','roam_tof_rear.png'):
    src=A.print_pack/filename
    if src.exists():
        dest=PACK/'R04_reference'/filename;dest.parent.mkdir(exist_ok=True);shutil.copy2(src,dest)
CHANGED=A.out/'ROAM_R05_changed_parts'
for r in records:
    if r['id'] in ('C02','C08','F05'):
        dst=CHANGED/r['file'];dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/r['file'],dst)
with (A.print_pack/'HARDWARE.csv').open(newline='') as f:
    hardware=list(csv.DictReader(f))
for row in hardware:
    if row['Modeled screw']=='M2.5x20':
        row['Quantity']=str(int(row['Quantity'])-2)
        row['Notes']+='; R05 removes the two old buck mounting screws'
hardware += [
    {'Modeled screw':'M2.5x16 countersunk','Quantity':'2','Notes':'R05 cradle-to-deck; reuse two original underside M2.5 nuts and washers. Heads must sit flush/below cradle; check actual head geometry and engagement.'},
    {'Modeled screw':'M2.5x10','Quantity':'4','Notes':'R05 two edge retainers; add four M2.5 nuts in underside cradle pockets. Do not force against PCB.'},
]
with (PACK/'HARDWARE.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=['Modeled screw','Quantity','Notes']);w.writeheader();w.writerows(hardware)
joint_data={'units':'mm, R04 assembled coordinates','cradle_countersunk_axes_xy':AD['existing_deck_holes_xy'],
            'retainer_axes_xy':[[CX+x,CY+y] for x in (-18,18) for y in (-26.5,26.5)],
            'deck_top_z':112.5,'reused_S01_spacer_top_z':115,'cradle_floor_bottom_z':Z0,
            'cradle_top_and_OEM_base_bottom_z':MODULE_BOTTOM,'OEM_base_top_z_estimated':MODULE_BOTTOM+5,
            'fan_upper_face_z':MODULE_BOTTOM+22,'OEM_holes_used_for_mounting':False}
(PACK/'R05_buck_joints.json').write_text(json.dumps(joint_data,indent=2))
for target in (PACK,CHANGED):
    shutil.copy2(A.out/'ASSEMBLY_R05.md',target/'START_HERE.md')
    shutil.copy2(A.out/'geometry_checks.json',target/'geometry_checks.json')
    shutil.copy2(A.out/'dimensions.json',target/'dimensions.json')
    shutil.copy2(PACK/'R05_buck_joints.json',target/'R05_buck_joints.json') if target!=PACK else None
(CHANGED/'manifest.json').write_text(json.dumps({**manifest,'parts':[r for r in records if r['id'] in ('C02','C08','F05')],
    'unique_stl_files':3,'total_physical_pieces_including_optional':5},indent=2))
for folder in (PACK,CHANGED):
    for part in json.loads((folder/'manifest.json').read_text())['parts']:
        assert hashlib.sha256((folder/part['file']).read_bytes()).hexdigest()==part['sha256']
    with zipfile.ZipFile(A.out/(folder.name+'.zip'),'w',zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(folder.rglob('*')):
            if path.is_file():archive.write(path,path.relative_to(A.out))

S['Mechanical revision']=CFG['revision']
S['Geekworm fit status']='Prototype: 59x44x22 mm supplied-base module; connector dimensions and base-retainer fit require physical validation.'
S['Geekworm mounting']='C02 replaces old carrier, keeps deck + S01 spacers; remove old S05. Two C08 edge clamps with two F05 pads.'
S['Geometry checks']='geometry_checks.json; unrelated R04 geometry preserved; nominated clash checks passed.'
S['Units']='1 Blender unit = 100 mm; STL exports use millimetres, no slicer rescaling.'

def visibility(c, state):
    for o in c.objects:
        o.hide_render=not state;o.hide_set(not state)

def aim(pos,target,span):
    cam=S.camera
    cam.location=Vector(pos)/SCALE
    cam.rotation_euler=(Vector(target)/SCALE-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.data.type='ORTHO';cam.data.ortho_scale=span/SCALE
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=='VIEW_3D':
                sp=area.spaces.active
                sp.region_3d.view_rotation=cam.rotation_euler.to_quaternion()
                sp.region_3d.view_location=Vector(target)/SCALE
                sp.region_3d.view_distance=span/SCALE
                sp.clip_start=.001
                sp.overlay.show_extras=False

def render(filename):
    if A.render:
        S.render.filepath=str(A.out/filename)
        bpy.ops.render.render(write_still=True)

visibility(GUIDES,False)
for c in (ELEC,MOUNT,HARD):visibility(c,True)
bpy.ops.object.select_all(action='DESELECT');adapter.select_set(True);bpy.context.view_layer.objects.active=adapter
aim((390,-600,360),(0,0,110),345)
bpy.ops.wm.save_as_mainfile(filepath=str(A.out/'roam_geekworm_r05.blend'),compress=True)
render('robot_assembled.png')
# Deck detail: hide obstruction by sensor frame/hood, leaving their saved original geometry untouched.
for key in ('04 Sensor hood','07 Bolted sensor frame'):visibility(bpy.data.collections[key],False)
aim((-210,-265,365),(-32,-30,118),192)
bpy.ops.wm.save_as_mainfile(filepath=str(A.out/'roam_geekworm_r05_inspection.blend'),compress=True)
render('electronics_installation.png')
for c in list(S.collection.children):
    if c not in (ELEC,MOUNT,HARD,GUIDES) and c.name!='06 Studio':visibility(c,False)
# Isolated detail files should not carry the whole hidden robot as baggage.
for o in list(S.objects):
    if o.type not in ('CAMERA','LIGHT') and not any(c in (ELEC,MOUNT,HARD,GUIDES) or c.name=='06 Studio' for c in o.users_collection):
        bpy.data.objects.remove(o,do_unlink=True)
bpy.data.orphans_purge(do_recursive=True)
aim((-145,-177,238),(CX,CY,128),105)
bpy.ops.wm.save_as_mainfile(filepath=str(A.out/'geekworm_mount_detail.blend'),compress=True)
render('geekworm_mount_detail.png')
visibility(MOUNT,False);visibility(HARD,False)
for c in (MOUNT,HARD,GUIDES):
    for o in list(c.objects):bpy.data.objects.remove(o,do_unlink=True)
bpy.data.orphans_purge(do_recursive=True)
bpy.ops.object.select_all(action='DESELECT');base.select_set(True);bpy.context.view_layer.objects.active=base
aim((-135,-173,233),(CX,CY,128),87)
bpy.ops.wm.save_as_mainfile(filepath=str(A.out/'geekworm_converter_reference.blend'),compress=True)
render('geekworm_converter_reference.png')
# Restore assembled view and make the same module-level exploded presentation as R04.
bpy.ops.wm.open_mainfile(filepath=str(A.out/'roam_geekworm_r05.blend'))
S=bpy.context.scene
for c in ('03 Electronics cassette','10 Geekworm - purchased reference, NOT FOR PRINT',
          '11 R05 Geekworm - printable adapter and retainers','12 R05 Geekworm - purchased fasteners and pads',
          '13 R05 Geekworm - clearance guides, hidden'):
    for o in bpy.data.collections[c].objects:o.location.z+=.45
for o in bpy.data.collections['07 Bolted sensor frame'].objects:o.location.z+=.9
for o in bpy.data.collections['02 Battery drawer'].objects:o.location.y+=1.55
for o in bpy.data.collections['04 Sensor hood'].objects:o.location.z+=2.5
for c in bpy.data.collections:
    if c.name=='05 Removable wheel skirt' or c.name.startswith('09'):
        for o in c.objects:o.location.x+=2.85
S['Exploded view']='R05 identification view, not a disassembly animation. New converter follows electronics cassette.'
aim((490,700,460),(90,40,220),790)
bpy.ops.wm.save_as_mainfile(filepath=str(A.out/'roam_geekworm_r05_exploded.blend'),compress=True)
render('robot_exploded.png')
assert hashlib.sha256(A.source.read_bytes()).hexdigest()==SOURCE_HASH
print('R05_COMPLETE',str(A.out),flush=True)
