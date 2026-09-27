"""Independent R18 source identity, functional sections, interference and motion QA."""
import ast
import hashlib
import json
import math
from pathlib import Path

import bpy
import bmesh
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'mechanical/drawer_cleanup_r18'
OLD = ROOT / 'mechanical/lower_controls_r17'
MASTER = OUT / 'roam_drawer_cleanup_r18.blend'
HASH = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
for file, names in [
    ('create_split_shell_r07.py', {'clean','remove','clone','box','cyl','boolean',
        'bounds','metrics','snapshot','intersection'}),
    ('validate_panel_bolts_r14.py', {'mesh_arrays'}),
    ('validate_lower_controls_r17.py', {'contact_volume'}),
    ('create_lower_controls_r17.py', {'local','cb','cy'}),
]:
    for node in ast.parse((ROOT / 'tools/cad' / file).read_text()).body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            exec(compile(ast.Module(body=[node],type_ignores=[]), '<R18 QA>', 'exec'))

build = json.loads((OUT / 'geometry_checks.json').read_text())
prior = json.loads((OLD / 'validation.json').read_text())
assert prior['status'] == 'PASS'
assert HASH(OLD / 'roam_lower_controls_r17.blend') == build['source_R17_sha256'] == prior['master_sha256']
assert HASH(MASTER) == build['master_sha256']
DN = build['changed_part']
bpy.ops.wm.open_mainfile(filepath=str(OLD / 'roam_lower_controls_r17.blend'))
bpy.context.view_layer.update()
old_snapshots = {o.name:snapshot(o) for o in bpy.context.scene.objects}
old_vs, old_fs = mesh_arrays(bpy.context.scene.objects['P05 | Curved switched battery drawer R17'])
bpy.ops.wm.open_mainfile(filepath=str(MASTER))
S = bpy.context.scene
bpy.context.view_layer.update()
drawer = S.objects[DN]
qa = {'revision':'R18', 'status':'RUNNING', 'master_sha256':HASH(MASTER),
      'source_R17_sha256':build['source_R17_sha256'],
      'limits':['Digital geometry checks, not a physical fit/strength or print-quality guarantee.',
                'Motion sampled every 0.5mm, not a continuous collision proof.',
                'Actual wires, insulation, strain relief and switch tolerances still need physical checking.',
                'No electrical design or switch DC-current approval is provided.']}
def save():
    (OUT / 'validation.json').write_text(json.dumps(qa,indent=2))

def curve(x,z):
    if z <= 61:
        w,d = 116.,104.
    elif z <= 98:
        f = (z-61)/37
        w,d = 116-2*f,104-f
    else:
        f = (z-98)/2
        w,d = 114-f,103-f
    return d*(1-(abs(x)/w)**4)**.25

def target_surface(x,z):
    y = curve(x,z)
    for centre in [-80,80]:
        d = math.hypot(x-centre,z-78)
        t = max(0,min(1,(8-d)/3))
        a = 6*t**5-15*t**4+10*t**3
        y = (1-a)*y+a*curve(80,78)
    return y

def ray(obj,origin,direction):
    hit,p,_,_ = obj.ray_cast(Vector(origin)/100,Vector(direction))
    assert hit,(obj.name,origin,direction)
    return p*100

def functional(obj):
    result = {'solid':metrics(obj)}
    m = result['solid']
    assert m['components'] == 1 and m['nonmanifold_edges'] == 0 and m['loose_vertices'] == 0 and m['volume_mm3'] > 0,m
    # Regression for the R17 wedge gaps. A ray starting IN the tray floor
    # must not exit it before it reaches the OUTSIDE of the curved fascia.
    section_errors = []
    for x in range(-70,71):
        for z in [58.7,59.2,60.,60.8,61.3]:
            y = ray(obj,(x,96,z),(0,1,0)).y
            section_errors.append(abs(y-curve(x,z)))
    assert max(section_errors)<.012,max(section_errors)
    result['continuous_floor_sections'] = {'sample_count':len(section_errors),
        'max_outside_surface_error_mm':max(section_errors),'early_exits_or_gaps':0}
    thicknesses = []
    for x in range(-60,61,2):
        end = curve(x,60)-3.6-.08
        for f in [.05,.25,.5,.75,.95]:
            y = 97+(end-97)*f
            bottom = ray(obj,(x,y,57),(0,0,1)).z
            top = ray(obj,(x,y,62),(0,0,-1)).z
            assert abs(bottom-58.5)<.006 and abs(top-61.5)<.006,(x,y,bottom,top)
            thicknesses.append(top-bottom)
    result['new_floor_thickness_mm'] = {'samples':len(thicknesses),
        'min':min(thicknesses),'max':max(thicknesses)}
    channel = []
    for x in [-12,12]:
        bottom = ray(obj,(x,35,55),(0,0,1)).z
        top = ray(obj,(x,35,63),(0,0,-1)).z
        assert abs(bottom-59.9)<.006 and abs(top-61.5)<.006,(bottom,top)
        channel.append(top-bottom)
    result['retained_caster_channel_floor_mm'] = channel
    # Four original battery-strap slots must remain open; no blind hole fill.
    for x in [-45,45]:
        for y in [4.5,53.5]:
            hit,p,_,_ = obj.ray_cast(Vector((x,y,58))/100,Vector((0,0,1)))
            assert not hit or p.z*100 > 62,(x,y,p)
    result['original_strap_slots_open'] = 4
    walls = []
    for x in [-34,-28,-22,-16,-10]:
        for z in [74,76,78]:
            inner = ray(obj,(x,90,z),(0,1,0)).y
            recess = ray(obj,(x,120,z),(0,-1,0)).y
            assert abs(inner-94.6)<.008 and abs(recess-97)<.008,(x,z,inner,recess)
            walls.append(recess-inner)
    result['pull_closed_backing_mm'] = {'min':min(walls),'max':max(walls),'samples':len(walls)}
    seats, blend_errors, blend_thickness = [],[],[]
    for centre in [-80,80]:
        for r in [2,3,4,4.7]:
            for i in range(32):
                a = math.tau*i/32
                x,z = centre+r*math.cos(a),78+r*math.sin(a)
                front = ray(obj,(x,120,z),(0,-1,0)).y
                rear = ray(obj,(x,90,z),(0,1,0)).y
                assert abs(front-curve(80,78))<.012,(x,z,front)
                assert abs(front-rear-3.6)<.015,(x,z,front,rear)
                seats.append(front-rear)
        for r in [5.125,5.625,6.125,6.625,7.125,7.625,8.125]:
            for i in range(64):
                a = math.tau*(i+.31)/64
                x,z = centre+r*math.cos(a),78+r*math.sin(a)
                if abs(x) >= 86.9:
                    continue
                p = ray(obj,(x,120,z),(0,-1,0))
                blend_errors.append(abs(p.y-target_surface(x,z)))
                rear = ray(obj,(x,p.y-.05,z),(0,-1,0))
                blend_thickness.append(p.y-rear.y)
        for zoff in [-1,0,1]:
            hit,p,_,_ = obj.ray_cast(Vector((centre,120,78+zoff))/100,Vector((0,-1,0)))
            assert not hit,(centre,zoff,p)
    assert max(blend_errors)<.035,max(blend_errors)
    result['washer_seats_mm'] = {'samples':len(seats),'min':min(seats),'max':max(seats)}
    result['washer_blend_surface_fidelity'] = {'samples':len(blend_errors),
        'max_error_mm':max(blend_errors),'limit_mm':.035}
    assert min(blend_thickness)>3.025,min(blend_thickness)
    result['inside_guide_relief_skin_mm'] = {'samples':len(blend_thickness),
        'min':min(blend_thickness),'max':max(blend_thickness)}
    mat = Matrix(build['switch_matrix'])
    surfaces = []
    for i in range(32):
        a = i*math.tau/32
        u,v = 10.8*math.cos(a),10.8*math.sin(a)
        front = ray(obj,mat@Vector((u/100,v/100,-.02))*100,(0,-1,0))
        back = ray(obj,mat@Vector((u/100,v/100,.05))*100,(0,1,0))
        f,b = (mat.inverted()@(front/100)).z*100,(mat.inverted()@(back/100)).z*100
        assert abs(f)<.015 and abs(b-2.2)<.015,(f,b)
        surfaces.append(b-f)
    result['switch_snap_seat_mm'] = {'min':min(surfaces),'max':max(surfaces),'samples':len(surfaces)}
    return result

try:
    expected = (set(old_snapshots)-{'P05 | Curved switched battery drawer R17'})|{DN}
    assert set(S.objects.keys()) == expected
    changes = [n for n,h in old_snapshots.items() if n != 'P05 | Curved switched battery drawer R17' and snapshot(S.objects[n]) != h]
    assert not changes,changes
    qa['unchanged_R17_objects'] = len(old_snapshots)-1
    qa['unexpected_object_changes'] = changes
    qa['master_functional_checks'] = functional(drawer)
    save()
    print('R18_QA: master functional sections passed',flush=True)

    row = build['parts'][0]
    stl = OUT / 'ROAM_R18_drawer_replacement' / row['file']
    assert HASH(stl) == row['sha256']
    bpy.ops.wm.stl_import(filepath=str(stl),use_facet_normal=False)
    imp = bpy.context.object
    bb = bounds(drawer)
    shift = Vector(((bb[0][0]+bb[0][1])/2,(bb[1][0]+bb[1][1])/2,bb[2][0]))/100
    imp.data.transform(Matrix.Translation(shift)@Matrix.Scale(.01,4))
    qa['exported_STL_functional_checks'] = functional(imp)
    remove(imp)
    save()
    print('R18_QA: exported STL functional sections passed',flush=True)

    me = bpy.data.meshes.new('_baseline R17 drawer')
    me.from_pydata(old_vs,[],old_fs)
    me.update()
    baseline = bpy.data.objects.new('_baseline R17 drawer',me)
    S.collection.objects.link(baseline)
    # All geometry ahead of the handle/front repair remains the same,
    # including battery bed, strap slots, rails and caster relief channels.
    # Coplanar solid subtraction is ill-conditioned after retriangulation
    # (both Boolean solvers produced nonmanifold residual surfaces). Instead
    # compare boundary samples in BOTH directions, including triangle centres
    # and edge midpoints, against full-size, unshrunk opposite surfaces.
    arrays = [mesh_arrays(baseline),mesh_arrays(drawer)]
    comparisons = []
    for (vs,fs),(other_v,other_f) in [arrays,arrays[::-1]]:
        # Work in millimetres; the BU-scale nearest-point routine has a
        # degeneracy threshold that misclassifies skinny Boolean triangles.
        other_tree = BVHTree.FromPolygons([v*100 for v in other_v],other_f,all_triangles=True)
        distances = []
        samples = []
        for f in fs:
            a,b,c = [vs[i]*100 for i in f]
            normal = (b-a).cross(c-a)
            if normal.length < 1e-10:
                continue
            normal.normalize()
            samples.extend((p,normal) for p in [a,b,c,(a+b+c)/3,(a+b)/2,(b+c)/2,(c+a)/2])
        worst = None
        ray_fallbacks = 0
        for p,source_normal in samples:
            if p.y > 84.9:
                continue
            hit,normal,index,d = other_tree.find_nearest(p)
            assert hit is not None
            if d >= .005:
                # Extremely skinny inherited floor triangles also stress
                # nearest-point arithmetic in mm. A perpendicular ray gives
                # an independent, actual opposite-surface witness point.
                for sign in [-1,1]:
                    q,_,_,_ = other_tree.ray_cast(p+source_normal*(sign*.25),
                                                  source_normal*(-sign),.5)
                    if q is not None:
                        d = min(d,(q-p).length)
                ray_fallbacks += 1
            if not distances or d > max_distance:
                max_distance = d
                worst = list(p)
            distances.append(d)
        comparisons.append({'samples':len(distances),'max_distance_mm':max(distances),
                            'worst_sample_mm':worst,'normal_ray_fallback_samples':ray_fallbacks})
    qa['unchanged_tray_and_rail_surface_comparison'] = comparisons
    print('R18_CORE_SURFACE_COMPARISON',comparisons,flush=True)
    assert all(c['max_distance_mm']<.005 for c in comparisons),comparisons
    env = [o for o in S.objects if o.type == 'MESH' and o not in [drawer,baseline]
           and not o.hide_render and not any(c.name=='06 Studio' for c in o.users_collection)
           and bounds(o)[2][0] < 105]
    contacts,collisions = [],[]
    for other in env:
        new_v = contact_volume(drawer,other)
        if new_v < .03:
            continue
        old_v = contact_volume(baseline,other)
        record = {'object':other.name,'R18_mm3':new_v,'R17_mm3':old_v}
        (collisions if new_v > old_v+.04 else contacts).append(record)
    qa['new_static_collisions'] = collisions
    qa['retained_contact_volumes'] = contacts
    remove(baseline)
    save()
    assert not collisions,collisions
    print('R18_QA: tray/rail identity and static clearances passed',flush=True)

    # Check the complete switch body/terminal service envelope against P05.
    mat = Matrix(build['switch_matrix'])
    reserve = cy(mat,(0,0,10.5),11.8,16)
    boolean(reserve,cb(mat,(0,0,27),(16,16,26)),'UNION')
    v = intersection(drawer,reserve)
    remove(reserve)
    assert v<.03,v
    qa['switch_service_envelope_interference_mm3'] = v

    # Only P05 changed: re-test its movement against all stationary R17 parts.
    # Payload/switch references and released E lock screws travel with it or
    # are removed, matching the already-validated R17 assembly procedure.
    stationary = [o for o in env if not o.name.startswith(('Component | Battery','Battery |','R17 BATTERY rocker'))
        and not (o.name.startswith(('E-1 |','E1 |')) and 'drawer lock nut' not in o.name)]
    sv,sf = [],[]
    for o in stationary:
        vs,fs = mesh_arrays(o)
        off = len(sv)
        sv.extend(vs)
        sf.extend(tuple(i+off for i in f) for f in fs)
    fixed = BVHTree.FromPolygons(sv,sf,all_triangles=True,epsilon=0)
    mv,mf = mesh_arrays(drawer)
    collisions = []
    for i in range(301):
        d = Vector((0,i*.005,0))
        moving = BVHTree.FromPolygons([v+d for v in mv],mf,all_triangles=True,epsilon=0)
        if moving.overlap(fixed):
            collisions.append(i*.5)
        if i%100 == 0:
            print('R18_QA: withdrawal',i*.5,'mm',flush=True)
    qa['drawer_withdrawal'] = {'range_mm':[0,150],'step_mm':.5,
        'sample_count':301,'surface_intersection_offsets_mm':collisions,
        'stationary_object_count':len(stationary)}
    assert not collisions,collisions
    assert HASH(MASTER) == build['master_sha256']
    assert HASH(OLD / 'roam_lower_controls_r17.blend') == build['source_R17_sha256']
    qa['status'] = 'PASS'
    save()
    print('R18_VALIDATION_COMPLETE PASS',flush=True)
except Exception as error:
    qa['status'] = 'FAIL'
    qa['error'] = repr(error)
    save()
    raise
