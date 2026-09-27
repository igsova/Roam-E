"""R07: non-scaled, serviceable split hood and ToF skirt, derived ONLY from R06.

Blender 5.2 --background --disable-autoexec --python THIS.py -- [--render]
Assembly coordinates: 100 mm/BU. STL coordinates: millimetres.
No original revision is overwritten. Purchased hardware is reference geometry.
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

parser = argparse.ArgumentParser()
parser.add_argument('--render', action='store_true')
args = parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT/'mechanical/geekworm_mux_r06'
OUT = ROOT/'mechanical/split_shell_r07'
PACK = OUT/'ROAM_STL_R07_Split_Shell_K1C'
CHANGED = OUT/'ROAM_R07_changed_parts'
SOURCE = OLD/'roam_geekworm_mux_r06.blend'
OLDPACK = OLD/'ROAM_STL_R06_Geekworm_MUX_K1C'
OUT.mkdir(parents=True, exist_ok=True)
HASH = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
source_hash = HASH(SOURCE)
bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version = 0
S = bpy.context.scene
assert S.unit_settings.scale_length == .1 or abs(S.unit_settings.scale_length-.1)<1e-7
assert 'R06' in S['Mechanical revision'] and not S.get('Exploded view')
SEAM, GAP, FIT = 95., .30, .30
HOOD = bpy.data.objects['Shell | Removable sensor hood']
SKIRT = bpy.data.objects['Shell | Independent lower skirt']
C = bpy.data.collections.new('16 R07 split shell joints - purchased hardware')
S.collection.children.link(C)
metal = bpy.data.materials['Machined metal']
capmat = bpy.data.materials.new('R07 | side panel graphite teal')
capmat.diffuse_color = (.065, .20, .22, 1)
capmat.use_nodes = True
capmat.node_tree.nodes.get('Principled BSDF').inputs['Base Color'].default_value = capmat.diffuse_color
capmat.node_tree.nodes.get('Principled BSDF').inputs['Roughness'].default_value = .38

def clean(o):
    bm=bmesh.new(); bm.from_mesh(o.data)
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=1e-6)
    bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=1e-7)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(o.data); bm.free(); o.data.update()

def remove(o): bpy.data.objects.remove(o, do_unlink=True)

def move_collection(o, c):
    for old in list(o.users_collection): old.objects.unlink(o)
    c.objects.link(o)

def clone(o, name):
    bpy.context.view_layer.update()
    mesh=bpy.data.meshes.new_from_object(o.evaluated_get(bpy.context.evaluated_depsgraph_get()))
    result=bpy.data.objects.new(name, mesh); S.collection.objects.link(result)
    mesh.transform(o.matrix_world); result.matrix_world=Matrix.Identity(4); clean(result)
    return result

def box(name, pos, dims):
    bpy.ops.mesh.primitive_cube_add(size=1, location=Vector(pos)/100)
    o=bpy.context.object; o.name=name; o.dimensions=Vector(dims)/100
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return o

def cyl(name, pos, radius, height, vertices=64):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius/100, depth=height/100, location=Vector(pos)/100)
    o=bpy.context.object; o.name=name
    return o

def mesh_object(name, verts, faces):
    me=bpy.data.meshes.new(name); me.from_pydata([Vector(v)/100 for v in verts], [], faces); me.update()
    o=bpy.data.objects.new(name, me); S.collection.objects.link(o); clean(o)
    return o

def prism(name, polygon, w0, w1):
    n=len(polygon)
    return mesh_object(name, [(u,v,w) for w in (w0,w1) for u,v in polygon],
        [tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)])

def boolean(a,b,op='DIFFERENCE',solver='MANIFOLD'):
    bpy.context.view_layer.update()
    m=a.modifiers.new('R07 '+op,'BOOLEAN'); m.operation=op; m.solver=solver; m.object=b
    bpy.context.view_layer.objects.active=a; bpy.ops.object.modifier_apply(modifier=m.name)
    remove(b); clean(a)
    assert not a.modifiers, ('Boolean did not apply', a.name, op)
    return a

def bounds(o):
    pts=[o.matrix_world@Vector(p)*100 for p in o.bound_box]
    return [(min(p[i] for p in pts),max(p[i] for p in pts)) for i in range(3)]

def metrics(o, factor=100):
    bm=bmesh.new(); bm.from_mesh(o.data); bm.transform(Matrix.Scale(factor,4)@o.matrix_world)
    pending=set(bm.verts); components=0
    while pending:
        components+=1; stack=[pending.pop()]
        while stack:
            for e in stack.pop().link_edges:
                for v in e.verts:
                    if v in pending: pending.remove(v); stack.append(v)
    pts=[v.co for v in bm.verts]
    result={'components':components, 'nonmanifold_edges':sum(not e.is_manifold for e in bm.edges),
        'loose_vertices':sum(not v.link_faces for v in bm.verts), 'volume_mm3':bm.calc_volume(signed=True),
        'dimensions_mm':[max(p[i] for p in pts)-min(p[i] for p in pts) for i in range(3)] if pts else [0]*3,
        'min_z_mm':min(p.z for p in pts) if pts else 0}
    bm.free(); return result

def snapshot(o):
    h=hashlib.sha256(); h.update(str(tuple(tuple(r) for r in o.matrix_world)).encode())
    if o.type=='MESH':
        for v in o.data.vertices: h.update(str(tuple(v.co)).encode())
        for p in o.data.polygons: h.update(str(tuple(p.vertices)).encode())
    return h.hexdigest()

def intersection(a,b,inset=False):
    if not all(min(x[1],y[1])-max(x[0],y[0])>.01 for x,y in zip(bounds(a),bounds(b))): return 0.
    tmp=clone(a,'_QA intersection')
    if inset:
        ps=[v.co for v in tmp.data.vertices]
        centre=Vector([(min(p[i] for p in ps)+max(p[i] for p in ps))/2 for i in range(3)])
        tmp.data.transform(Matrix.Translation(centre)@Matrix.Scale(.998,4)@Matrix.Translation(-centre))
    solid=True
    for obj in (tmp,b):
        bm=bmesh.new();bm.from_mesh(obj.data)
        solid=solid and all(e.is_manifold for e in bm.edges);bm.free()
    boolean(tmp,clone(b,'_QA other'),'INTERSECT',solver='MANIFOLD' if solid else 'EXACT')
    v=abs(metrics(tmp)['volume_mm3']); remove(tmp); return v

def envelope(rings):
    # Same vertices and ring interpolation as the original 128-sided skin.
    # Closed outer envelope clips additions INSIDE the original cover outline.
    n=128; verts=[]
    mid_z=(rings[0][2]+rings[-1][2])/2
    for w,d,z,cy,p in rings:
        # Stay just inside the source shell, avoiding coincident union faces.
        w-=.15; d-=.15; z=mid_z+(z-mid_z)*.997
        sp=lambda a: math.copysign(abs(a)**p,a)
        verts.extend((w*sp(math.cos(i*math.tau/n)),cy+d*sp(math.sin(i*math.tau/n)),z) for i in range(n))
    faces=[tuple(reversed(range(n))),tuple(range((len(rings)-1)*n,len(rings)*n))]
    faces += [(k*n+j,k*n+(j+1)%n,(k+1)*n+(j+1)%n,(k+1)*n+j) for k in range(len(rings)-1) for j in range(n)]
    return mesh_object('_original outer envelope',verts,faces)

def half_tool(main):
    edge=SEAM+(-GAP/2 if main else GAP/2)
    return box('_split plane',(edge+(-250 if main else 250),0,100),(500,600,500))

def ray_surface(o, origin, direction):
    inv=o.matrix_world.inverted(); d=(inv.to_3x3()@Vector(direction)).normalized()
    hit,p,_,_=o.ray_cast(inv@(Vector(origin)/100),d)
    assert hit, (o.name,origin,direction)
    return o.matrix_world@p*100

bpy.context.view_layer.update()
ORIGINAL={o.name:snapshot(o) for o in S.objects}
original_shell_names=[HOOD.name,SKIRT.name]
main_hood=clone(HOOD,'P15A | Main smiling hood R07')
cap_hood=clone(HOOD,'P15B | Removable hood side panel R07')
main_skirt=clone(SKIRT,'P16A | Main ToF skirt R07')
cap_skirt=clone(SKIRT,'P16B | Removable ToF side panel R07')
parts=[main_hood,cap_hood,main_skirt,cap_skirt]
for main,cap,source in [(main_hood,cap_hood,HOOD),(main_skirt,cap_skirt,SKIRT)]:
    boolean(main,half_tool(True),'INTERSECT'); boolean(cap,half_tool(False),'INTERSECT')
    for obj in (main,cap): move_collection(obj,source.users_collection[0])
    cap.data.materials.clear(); cap.data.materials.append(capmat)
    for poly in cap.data.polygons: poly.material_index=0

# A straight cut would leave the existing M4 skirt foot floating on the main
# part. Transfer that complete original foot to the side panel, unchanged.
ear=clone(SKIRT,'_original M4 foot')
boolean(ear,box('_whole M4 foot',(99,42,57.7),(34,14,6.1)),'INTERSECT')
boolean(main_skirt,box('_M4 foot allocation',(99,42,57.7),(34,14,6.1)))
boolean(cap_skirt,ear,'UNION')

hood_env=envelope([(113,102,102,0,.5),(113,102,107,0,.5),(111,101,157,0,.5),
    (109,99,176,0,.5),(105,94,185,0,.5),(97,85,190,0,.5),(52,52,191,5,1)])
skirt_env=envelope([(113,101,13,0,.5),(116,104,18,0,.5),(116,104,61,0,.5),(114,103,98,0,.5),(113,102,100,0,.5)])
joints=[]; hardware=[]; added=[]

def place(o, matrix):
    bpy.context.view_layer.update(); o.matrix_world=matrix@o.matrix_world
    return o

def joint(main, cap, source, env, name, sign=1, z=118, roof=False, roof_y=0, skirt=False, small=False, short=False):
    """Integral cap tongue, recessed main receiver, external screw/captive nut.

    Local u = assembly X. w points into the cover. v is along the wall.
    Hood gussets grow from the roof down; skirt gussets grow from the bed up.
    A 0.30 mm clearance pocket prevents modeled tongue interference.
    """
    if roof:
        p=ray_surface(source,(SEAM,roof_y,250),(0,0,-1)); v=Vector((0,-1,0)); w=Vector((0,0,-1))
    else:
        p=ray_surface(source,(SEAM,sign*200,z),(0,-sign,0))
        v=Vector((0,0,-1 if skirt else 1)); w=Vector((0,-sign,0))
    u=Vector((1,0,0))
    # Reflection is permitted; recalc normals after the local-to-world bake.
    mat=Matrix(((u.x,v.x,w.x,p.x/100),(u.y,v.y,w.y,p.y/100),(u.z,v.z,w.z,p.z/100),(0,0,0,1)))
    def local(o):
        place(o,mat); o.data.transform(o.matrix_world); o.matrix_world=Matrix.Identity(4); clean(o); return o
    def cb(pos,dims): return local(box('_joint tool',pos,dims))
    def cy(pos,r,h,n=64): return local(cyl('_joint tool',pos,r,h,n))
    half=4.5 if small else (5.5 if short else 7.)
    top=half if roof or small or short else 20.
    front=4.5 if roof else 2.4
    back=front+5.
    pad_end=front-FIT
    poly=[(-12,-half),(10,-half),(10,top),(1,top),(-12,half)] if top>half else [(-12,-half),(10,-half),(10,half),(-12,half)]
    tongue=local(prism('_alignment tongue',poly,front,back))
    boolean(tongue,clone(env,'_tongue envelope'),'INTERSECT')
    # Main pad has a 45 degree inward-growth gusset in the chosen print direction.
    pad=cb((-6.075,(top-half)/2,(-12+pad_end)/2),(11.85,top+half,12+pad_end))
    if top>half:
        # Cut the unsupported upper-inner wedge (v,w basis) using a prism.
        wedge=prism('_pad gusset',[(half,1.1),(top+2,1.1),(top+2,-(top-half)-.9)],-30,30)
        # Original prism axes are (v,w,u); permute into local (u,v,w).
        perm=Matrix(((0,0,1,0),(1,0,0,0),(0,1,0,0),(0,0,0,1)))
        wedge.data.transform(perm); local(wedge); boolean(pad,wedge)
    boolean(pad,clone(env,'_pad envelope'),'INTERSECT')
    boolean(pad,half_tool(True),'INTERSECT')
    boolean(main,pad,'UNION')
    # The open-sided receiver also clears the original wall behind the tongue.
    receiver=cb((-1,(top-half)/2,(front+back)/2),(22+2*FIT,top+half+2*FIT,5+2*FIT))
    boolean(main,receiver)
    root=cb((5.075,0,(1.4+back)/2),(9.85,2*half,back-1.4))
    boolean(root,clone(env,'_tongue root envelope'),'INTERSECT')
    boolean(cap,root,'UNION')
    boolean(cap,tongue,'UNION')
    # Screw at u=-6 leaves >2.6 mm around the M3 captive-nut corners.
    axis_point=p+u*(-6)
    surface_depths=[]
    for k in range(24):
        angle=k*math.tau/24
        sample=axis_point+3*(u*math.cos(angle)+v*math.sin(angle))
        surface=ray_surface(source,sample-w*100,w)
        surface_depths.append((surface-axis_point).dot(w))
    # Full 6 mm washer bears on a flat spotface, including the curved skirt.
    seat=max(surface_depths)+.25
    for obj in (main,cap): boolean(obj,cy((-6,0,0),1.7,70))
    boolean(main,cy((-6,0,seat-25),3.2,50))
    boolean(cap,cy((-6,0,back-.4),5.9/math.sqrt(3),4.8,6))
    L=next(l for l in (10,12,16,20,25) if seat-.5+l>=back+.8)
    assert pad_end-seat>=2.5,('Insufficient main screw shoulder',name,pad_end-seat)
    shaft=cy((-6,0,seat-.5+L/2),1.45,L)
    head=cy((-6,0,seat-2),2.75,3)
    boolean(head,cy((-6,0,seat-3.1),2.5/math.sqrt(3),1.4,6))
    boolean(shaft,head,'UNION'); shaft.name=name+f' | M3x{L} screw'
    nut=cy((-6,0,back-1.5),5.5/math.sqrt(3),2.4,6)
    boolean(nut,cy((-6,0,back-1.5),1.4,5)); nut.name=name+' | M3 captive nut'
    washer=cy((-6,0,seat-.25),3,.5)
    boolean(washer,cy((-6,0,seat-.25),1.7,2)); washer.name=name+' | M3 washer'
    for obj in (shaft,nut,washer):
        obj.data.materials.clear(); obj.data.materials.append(metal); move_collection(obj,C)
        obj['Reference only']='Purchased metal hardware, not a printable part'; obj['R07 cover']='skirt' if skirt else 'hood'
        hardware.append(obj)
    main['Joint clearance mm']=FIT; cap['Joint clearance mm']=FIT
    main['Manufacture']='PRINT'; cap['Manufacture']='PRINT'
    qp=[mat@(Vector((uu,vv,ww))/100)*100 for uu in (-12.3,10.3) for vv in (-half-.3,top+.3) for ww in (-13,back+2)]
    qb=[(min(p[i] for p in qp),max(p[i] for p in qp)) for i in range(3)]
    joints.append({'id':name,'main':main.name,'cap':cap.name,'origin_mm':list(p),
        'screw_axis_inward':list(w),'screw_seat_mm':list(axis_point+w*seat),
        'screw':f'M3x{L}','quantity':1,'captive_nut':'M3 standard 5.5 AF x 2.4 thick',
        'nut_recess_AF_mm':5.9,'nut_recess_depth_mm':2.8,'bore_mm':3.4,
        'receiver_clearance_mm':FIT,'tongue_thickness_mm':5.,'seam_gap_mm':GAP,'qa_bounds_mm':qb,
        'main_screw_shoulder_at_axis_mm':pad_end-seat,'nut_shoulder_mm':2.2})
    print('JOINT',json.dumps(joints[-1]),flush=True)

joint(main_hood,cap_hood,HOOD,hood_env,'H1',roof=True,roof_y=-35)
joint(main_hood,cap_hood,HOOD,hood_env,'H2',roof=True,roof_y=0)
joint(main_hood,cap_hood,HOOD,hood_env,'H3',roof=True,roof_y=35)
joint(main_skirt,cap_skirt,SKIRT,skirt_env,'S1',sign=-1,z=19,skirt=True,small=True)
joint(main_skirt,cap_skirt,SKIRT,skirt_env,'S2',sign=1,z=19,skirt=True,small=True)
# Passive lower hood locators: shallow enough to stay outside the uprights.
for sign in (-1,1):
    pt=ray_surface(HOOD,(SEAM,sign*200,135),(0,-sign,0))
    tongue=box('_hood lower locating lip',(95,pt.y-sign*1.9,135),(10,1.,8))
    boolean(tongue,clone(hood_env,'_locator envelope'),'INTERSECT')
    boolean(main_hood,box('_locator clearance',(95,pt.y-sign*1.9,135),(10.6,1.6,8.6)))
    boolean(cap_hood,tongue,'UNION')
# Tiny disconnected tetrahedra can remain where the lower lap pocket meets the
# original rounded bottom chamfer. Remove ONLY sub-0.1 mm3 slivers, not features.
def discard_tiny_islands(obj):
    bm=bmesh.new();bm.from_mesh(obj.data);todo=set(bm.verts);removed=[]
    while todo:
        verts={todo.pop()};stack=list(verts)
        while stack:
            for e in stack.pop().link_edges:
                for v in e.verts:
                    if v in todo:todo.remove(v);verts.add(v);stack.append(v)
        if len(verts)<=8:
            ps=[v.co for v in verts]
            bbvol=math.prod((max(p[i] for p in ps)-min(p[i] for p in ps))*100 for i in range(3))
            assert bbvol<.2,('Unexpected detached feature',bbvol)
            removed.append({'vertices':len(verts),'bounding_box_volume_mm3':bbvol})
            bmesh.ops.delete(bm,geom=list(verts),context='VERTS')
    bm.to_mesh(obj.data);bm.free();clean(obj);return removed
slivers=discard_tiny_islands(main_skirt)
remove(hood_env); remove(skirt_env)
bpy.context.view_layer.update()

# The smile stays separate/material-colored in Blender, and is fused into P15A STL.
print_hood=clone(main_hood,'_P15A print smile union')
for o in list(S.objects):
    if o.name.startswith('Smile |'): boolean(print_hood,clone(o,'_smile print union'),'UNION')
for obj in parts+[print_hood]:
    checked=metrics(obj)
    assert checked['nonmanifold_edges']==0 and checked['components']==1,(obj.name,checked)

# Check every new print and fastener against the unchanged visible assembly.
ignore=set(original_shell_names)
environment=[o for o in S.objects if o.name in ORIGINAL and o.name not in ignore and o.type=='MESH' and not o.hide_render
    and not o.name.startswith('Smile |') and not any(c.name=='06 Studio' for c in o.users_collection)]
clashes=[]
probes=[]
for obj,source in [(main_hood,HOOD),(cap_hood,HOOD),(main_skirt,SKIRT),(cap_skirt,SKIRT)]:
    for j in [j for j in joints if obj.name in (j['main'],j['cap'])]:
        region=clone(obj,'_check '+obj.name+' '+j['id'])
        bb=j['qa_bounds_mm']
        boolean(region,box('_joint QA region',[(a+b)/2 for a,b in bb],[b-a for a,b in bb]),'INTERSECT')
        if region.data.vertices:probes.append((region,obj.name))
        else:remove(region)
for obj in (cap_hood,):
    for sign in (-1,1):
        pt=ray_surface(HOOD,(SEAM,sign*200,135),(0,-sign,0))
        region=clone(obj,'_passive locator QA')
        boolean(region,box('_passive QA region',(95,pt.y-sign*1.9,135),(11,2,9)),'INTERSECT')
        if region.data.vertices:probes.append((region,obj.name+' lower locator'))
        else:remove(region)
for o,label in probes+[(o,o.name) for o in hardware]:
    for other in environment:
        vol=intersection(o,other)
        if vol>.03: clashes.append([label,other.name,round(vol,4)])
for o,_ in probes:remove(o)
mating=[]
for main,cap in [(main_hood,cap_hood),(main_skirt,cap_skirt)]:
    vol=intersection(main,cap)
    if vol>.03: mating.append([main.name,cap.name,vol])
for obj in hardware:
    for other in parts:
        vol=intersection(obj,other,inset=True)
        if vol>.03: mating.append([obj.name,other.name,vol])
solids={o.name:metrics(o) for o in parts+[print_hood]}
unchanged=[n for n,h in ORIGINAL.items() if snapshot(bpy.data.objects[n])!=h]
report={'revision':'R07 split hood and skirt','source':str(SOURCE),'source_sha256':source_hash,
    'seam_x_mm':SEAM,'seam_gap_mm':GAP,'joint_fit_clearance_mm':FIT,'joints':joints,
    'solid_checks':solids,'assembly_clashes':clashes,'mating_clashes':mating,
    'removed_chamfer_slivers':slivers,'contact_test':'Fasteners inset 0.2 percent for mesh contact robustness; unscaled hardware retained in assembly.',
    'unexpected_original_changes':unchanged,'unchanged_original_objects':len(ORIGINAL)-2,
    'M4_original_skirt_foot':'Transferred intact to P16B; same hole and same robot fastener position',
    'limits':['Prototype mechanical fit; print joint coupons before covers.',
        'No scale reduction. New local screw recesses, seam gaps and internal receivers are intentional.',
        'Support quantity is slicer/profile dependent; no support-free claim.',
        'R05 Geekworm and R06 MUX physical-fit caveats remain.']}
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
bpy.ops.wm.save_as_mainfile(filepath='/private/tmp/r07-debug.blend',compress=True)
print('R07_CHECKS',json.dumps(report),flush=True)
assert not clashes and not mating and not unchanged, 'Inspect geometry_checks.json'
assert all(m['components']==1 and m['nonmanifold_edges']==0 and m['loose_vertices']==0 and m['volume_mm3']>0 for m in solids.values()),solids

# Coupons are actual joint regions, with flange/core and nut pocket preserved.
# H1 samples the actual curved-roof lap; independent prints, no robot role.
coupons=[]
for obj,pid in [(main_hood,'Q07A'),(cap_hood,'Q07B')]:
    q=clone(obj,pid+' | Actual H1 joint fit coupon')
    boolean(q,box('_coupon boundary',(94,-35,187),(26,22,14)),'INTERSECT')
    coupons.append(q)

records=[]
old_manifest=json.loads((OLDPACK/'manifest.json').read_text())
for row in old_manifest['parts']:
    if row['id'] in ('P15','P16'): continue
    target=PACK/row['file']; target.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(OLDPACK/row['file'],target); assert HASH(target)==row['sha256']
    records.append(row)

def export(obj,path,rot):
    tmp=clone(obj,'_export')
    tmp.data.transform(rot@Matrix.Scale(100,4)@tmp.matrix_world); tmp.matrix_world=Matrix.Identity(4)
    pts=[v.co for v in tmp.data.vertices]
    lo=[min(p[i] for p in pts) for i in range(3)]; hi=[max(p[i] for p in pts) for i in range(3)]
    tmp.data.transform(Matrix.Translation(Vector((-(lo[0]+hi[0])/2,-(lo[1]+hi[1])/2,-lo[2])))); clean(tmp)
    before=metrics(tmp,1)
    path.parent.mkdir(parents=True,exist_ok=True)
    bpy.ops.object.select_all(action='DESELECT'); tmp.select_set(True); bpy.context.view_layer.objects.active=tmp
    bpy.ops.wm.stl_export(filepath=str(path),export_selected_objects=True,use_scene_unit=False,forward_axis='Y',up_axis='Z')
    bpy.ops.object.select_all(action='DESELECT'); bpy.ops.wm.stl_import(filepath=str(path),use_facet_normal=False)
    imp=bpy.context.object; result=metrics(imp,1)
    assert result['components']==1 and result['nonmanifold_edges']==0 and result['volume_mm3']>0,result
    assert abs(result['min_z_mm'])<.001 and all(abs(a-b)<.002 for a,b in zip(before['dimensions_mm'],result['dimensions_mm']))
    assert all(v+8<220 for v in result['dimensions_mm'][:2]) and result['dimensions_mm'][2]<250,result
    remove(imp); remove(tmp); return result

ROOF_DOWN=Matrix.Rotation(math.pi,4,'X')
SIDE_DOWN=Matrix.Rotation(math.pi/2,4,'Y') # exterior +X faces bed
for pid,label,obj,rot,folder,note,support in [
    ('P15A','main_smiling_hood',print_hood,ROOF_DOWN,'01_rigid','Roof down, cavity up; do NOT auto-orient or scale. Camera opening, smile, LiDAR aperture and four service ports unchanged.','Local supports under curved roof, aperture bridges and joint details; inspect preview'),
    ('P15B','hood_side_panel',cap_hood,SIDE_DOWN,'01_rigid','Outer side toward bed, integral locating tongues upward. H1-H3 captive M3 nuts inserted from cavity before assembly.','Shallow supports under curved exterior and overhanging joint details'),
    ('P16A','main_tof_skirt',main_skirt,Matrix.Identity(4),'01_rigid','Original bottom down; all front/rear and left-side ToF pockets remain.','Local supports under original mounting flanges, wheel relief and sensor pocket details'),
    ('P16B','tof_skirt_side_panel',cap_skirt,SIDE_DOWN,'01_rigid','Outer side toward bed. Right ToF station and COMPLETE original M4 foot retained.','Shallow supports under curved exterior and original inward-facing pockets/flange'),
    ('Q07A','hood_joint_receiver_test',coupons[0],ROOF_DOWN,'04_fit_test','Optional actual H1 receiver sample; not installed on robot. Test with Q07B and M3 hardware before full covers.','Local supports; keep bores/nut seats clear'),
    ('Q07B','hood_joint_tongue_test',coupons[1],SIDE_DOWN,'04_fit_test','Optional actual H1 tongue sample; test 0.30 mm receiver clearance and captive-nut insertion.','Local supports; keep bores/nut seats clear')]:
    filename=f'{folder}/{pid}_{label}_qty1.stl'; result=export(obj,PACK/filename,rot)
    sources=[obj.name]
    if pid=='P15A':sources=[main_hood.name]+[o.name for o in S.objects if o.name.startswith('Smile |')]
    if pid in ('Q07A','Q07B'):sources=[main_hood.name if pid=='Q07A' else cap_hood.name]
    row={'id':pid,'label':label,'sources':sources,'quantity':1,'folder':folder,'file':filename,
        'note':note,'support':support,'reimport_check':result,'sha256':HASH(PACK/filename),'brim_4mm_fits_K1C':True,
        'export_rotation_matrix':[[round(x,8) for x in r] for r in rot]}
    records.append(row); print('R07_EXPORTED',json.dumps(row),flush=True)
for o in coupons: remove(o)
remove(print_hood)
for o in (HOOD,SKIRT): remove(o)

manifest={**old_manifest,'revision':'R07 non-scaled split hood and skirt','source':'roam_split_shell_r07.blend',
    'unique_stl_files':len(records),'total_physical_pieces_including_optional':sum(r['quantity'] for r in records),
    'parts':records,'unchanged_parts_source':str(OLDPACK),
    'replaces':['P15 -> P15A + P15B','P16 -> P16A + P16B'],
    'new_optional_coupons':['Q07A','Q07B'],'joint_hardware':joints}
(PACK/'manifest.json').write_text(json.dumps(manifest,indent=2))
with (PACK/'PRINT_PARTS.csv').open('w',newline='') as f:
    writer=csv.writer(f); writer.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
    for r in records: writer.writerow([r['id'],r['label'],r['quantity'],r['file'],*[round(v,3) for v in r['reimport_check']['dimensions_mm']],r['support'],r['note']])
with (OLDPACK/'HARDWARE.csv').open(newline='') as f: hw=list(csv.DictReader(f))
for screw in sorted(set(j['screw'] for j in joints)):
    hw.append({'Modeled screw':screw,'Quantity':str(sum(j['screw']==screw for j in joints)),
        'Notes':'ADDITIONAL R07 shell joint hardware; one M3 standard nut and one M3 washer per screw. See R07 joint schedule.'})
with (PACK/'HARDWARE.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=['Modeled screw','Quantity','Notes']);writer.writeheader();writer.writerows(hw)
changed=[r for r in records if r['id'] in ('P15A','P15B','P16A','P16B','Q07A','Q07B')]
for r in changed:
    p=CHANGED/r['file'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/r['file'],p)
(CHANGED/'manifest.json').write_text(json.dumps({**manifest,'parts':changed,'unique_stl_files':len(changed),'total_physical_pieces_including_optional':len(changed)},indent=2))

S['Mechanical revision']='R07 split hood + skirt, R06 electronics retained'
S['Revision']=S['Mechanical revision']
S['R07 split']='X=95 mm; 0.30 mm seam gap; integral cap-side lap tongues and 0.30 mm receivers; five M3 bolted joints plus two passive hood locators.'
S['R07 skirt']='P16B retains original M4 foot and right ToF station. No sensor or mounting axis moved.'
S['Geometry checks']='split_shell_r07/geometry_checks.json'
S.render.engine='CYCLES'; S.cycles.samples=24; S.cycles.device='CPU'
S.render.resolution_x=1500; S.render.resolution_y=1150; S.render.resolution_percentage=100

def aim(pos,target,span):
    cam=S.camera; cam.location=Vector(pos)/100; cam.rotation_euler=(Vector(target)/100-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.data.type='ORTHO'; cam.data.ortho_scale=span/100
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type=='VIEW_3D':
                view=area.spaces.active; view.region_3d.view_rotation=cam.rotation_euler.to_quaternion()
                view.region_3d.view_location=Vector(target)/100; view.region_3d.view_distance=span/100; view.overlay.show_extras=False
def save(name): bpy.ops.wm.save_as_mainfile(filepath=str(OUT/name),compress=True)
def render(name):
    if args.render: S.render.filepath=str(OUT/name);bpy.ops.render.render(write_still=True)
bpy.ops.object.select_all(action='DESELECT'); main_hood.select_set(True);bpy.context.view_layer.objects.active=main_hood
aim((410,-560,370),(0,0,110),365); save('roam_split_shell_r07.blend'); render('robot_assembled.png')
# Cover-only exploded view: colors distinguish new removable right panels.
for o in S.objects:
    if o not in parts+hardware and not o.name.startswith('Smile |') and o.type not in ('CAMERA','LIGHT') and not any(c.name=='06 Studio' for c in o.users_collection):
        o.hide_render=True;o.hide_set(True)
for obj in (main_hood,cap_hood): obj.location.z+=1.4
for o in S.objects:
    if o.name.startswith('Smile |'):o.location.z+=1.4
cap_hood.location.x+=.75;cap_skirt.location.x+=.75
for obj in hardware:
    if obj['R07 cover']=='hood':obj.location.z+=1.4
    if 'captive nut' in obj.name:obj.location.x+=.75
S['Exploded view']='R07 cover identification only. Side panels shown +75 mm X, hood +140 mm Z. Not a disassembly sweep.'
aim((420,-540,425),(30,0,170),600);save('roam_split_shell_r07_exploded.blend');render('split_covers_exploded.png')
assert HASH(SOURCE)==source_hash
print('R07_COMPLETE',OUT,flush=True)
