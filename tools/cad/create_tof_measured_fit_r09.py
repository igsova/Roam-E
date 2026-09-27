"""R09: measured 10.66 mm PCB, 4.80 mm optic-edge to header-centre spacing.

Rebuild only the two skirts from the untouched R07 topology so the misplaced
R08 channel is not left behind. Everything else starts from the current R08
assembly. Run in Blender background with -- --render for native CAD previews.
"""
import ast
import csv
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path
import bpy
import bmesh
from mathutils import Matrix,Vector
from mathutils.bvhtree import BVHTree

ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'mechanical/tof_solder_relief_r08'
BASE=ROOT/'mechanical/split_shell_r07'
SOURCE=OLD/'roam_tof_solder_relief_r08.blend'
BASEFILE=BASE/'roam_split_shell_r07.blend'
OLDPACK=OLD/'ROAM_STL_R08_ToF_Solder_Relief_K1C'
OUT=ROOT/'mechanical/tof_measured_fit_r09'
PACK=OUT/'ROAM_STL_R09_Measured_ToF_K1C'
CHANGED=OUT/'ROAM_R09_changed_parts'
OUT.mkdir(parents=True,exist_ok=True)
HASH=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source_hash=HASH(SOURCE);base_hash=HASH(BASEFILE)
RENDER='--render' in sys.argv
for file,names in [
    ('create_split_shell_r07.py',{'clean','remove','move_collection','clone','box','mesh_object','boolean','bounds','metrics','snapshot','intersection'}),
    ('create_tof_solder_relief_r08.py',{'local_box','tree','ray','export','aim','render','material','label'})]:
    for node in ast.parse((ROOT/'tools/cad'/file).read_text()).body:
        if isinstance(node,ast.FunctionDef) and node.name in names:
            exec(compile(ast.Module(body=[node],type_ignores=[]),'<reused CAD primitive>','exec'))

bpy.ops.wm.open_mainfile(filepath=str(SOURCE))
bpy.context.preferences.filepaths.save_version=0
S=bpy.context.scene
assert 'R08' in S['Mechanical revision'] and not S.get('Exploded view')
bpy.context.view_layer.update()
ORIGINAL={o.name:snapshot(o) for o in S.objects}
main=S.objects['P16A | Main ToF skirt R07'];cap=S.objects['P16B | Removable ToF side panel R07']
coupon=S.objects['P19 | ToF fit-test pocket']
old_coupon_mesh=coupon.data.copy()
original_references={}
changed_names={main.name,cap.name,coupon.name}
# Append base meshes to this scene, then use their topology and existing R08
# world transforms. R07 -> R08 had no skirt-object transform changes.
with bpy.data.libraries.load(str(BASEFILE),link=False) as (data_from,data_to):
    data_to.objects=[main.name,cap.name]
base_copies=[]
for current,base in zip([main,cap],data_to.objects):
    assert max(abs(v) for row in (current.matrix_world-base.matrix_world) for v in row)<1e-6
    current.data=base.data.copy()
    remove(base)
    base_copies.append(clone(current,'_baseline '+current.name))

PCB_FACE=-.85;PCB_WIDTH=10.66;OPTICAL_HALF=1.2;EDGE_TO_ROW=4.8
ROW_V=-(OPTICAL_HALF+EDGE_TO_ROW) # -6.0 mm; old -7.9 mm
ROW_SHIFT=1.9
PCB_TOP=3.1 # optical-side PCB edge retained, not a newly measured datum
PCB_BOTTOM=PCB_TOP-PCB_WIDTH # -7.56
POCKET_BOTTOM=PCB_BOTTOM-.3 # 0.30 mm tolerance at pin-side edge
CLEARANCE=2.;FLOOR=PCB_FACE+CLEARANCE;SKIN=1.2;OUTSIDE=FLOOR+SKIN
WIDTH=16.2;HEIGHT=4.2
tags=['Front_L','Front_C','Front_R','Rear_L','Rear_C','Rear_R','Left','Right']
matrices={};additions=[]
report={'revision':'R09 measured ToF fit','source_R08_sha256':source_hash,'base_R07_sha256':base_hash,
    'measured_inputs':{'PCB_short_dimension_mm':PCB_WIDTH,'sensor_edge_to_pin_row_centre_mm':EDGE_TO_ROW,
        'pin_projection_estimate_mm':1.},
    'derived_design':{'optical_package_short_dimension_mm':2.4,'sensor_centre_to_row_centre_mm':6.,
        'row_moved_toward_optics_mm':ROW_SHIFT,'PCB_pin_side_edge_mm':PCB_BOTTOM,
        'pocket_pin_side_edge_mm':POCKET_BOTTOM,'pocket_short_dimension_mm':3.4-POCKET_BOTTOM,
        'channel_width_mm':WIDTH,'channel_height_mm':HEIGHT,'clearance_from_PCB_mm':CLEARANCE,
        'remaining_outside_skin_mm':SKIN,'aperture_to_channel_edge_gap_mm':(-ROW_V-HEIGHT/2)-1.8},
    'remaining_assumptions':{'PCB_length_mm':25.,'PCB_thickness_mm':1.6,
        'optical_side_PCB_edge_from_sensor_centre_mm':PCB_TOP,
        'ear_centres_local_mm':[[-10,-3.2],[10,-3.2]],
        'note':'These old datums are retained, not inferred to be measured by the two new dimensions. Print P19 first.'},
    'stations':[],'addition_clashes':[],'reference_clashes':[],'optical_ray_obstructions':[]}

def bump(mat):
    # Shift the complete closed bump with the row. Lower transition remains
    # tapered; optic-side edge still does not enter the optical aperture.
    rings=[(-.4,10.2,-15.5+ROW_SHIFT,-4.1+ROW_SHIFT),
           (OUTSIDE,9.4,-11.3+ROW_SHIFT,-4.5+ROW_SHIFT)]
    vs=[]
    for w,half,low,high in rings:
        vs += [(-half,low,w),(half,low,w),(half,high,w),(-half,high,w)]
    o=mesh_object('_new solder bulge',vs,[(3,2,1,0),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
    o.matrix_world=mat;return o

def contact_volume(ob,part):
    # P17 has long pressure fingers: shrinking its whole bounding box would
    # MOVE its frame's -8 mm contact plane forward and create a false clash.
    # Instead separate that intentional contact by 0.02 mm in a QA copy only.
    if ob.name.startswith('P17 |'):
        tmp=clone(ob,'_P17 separated contact QA')
        mat=matrices[ob['ToF station']]
        tmp.matrix_world.translation-=mat.to_3x3()@Vector((0,0,.02/100))
        vol=intersection(tmp,part);remove(tmp);return vol
    return intersection(ob,part,inset=True)

for tag in tags:
    pcb=S.objects['ToF '+tag+' | Reference PCB 25x12.6'];mat=pcb.matrix_world.copy();matrices[tag]=mat
    part=cap if tag=='Right' else main
    original_references[tag]=pcb.name
    # Restore the lower pocket wall to the new board edge; do not shift either
    # screw or mounting-ear seat. Includes the obsolete shallow plating relief.
    low=-10.05
    filler=local_box('_narrow pin-side pocket',(0,(low+POCKET_BOTTOM)/2,-4),
                     (26.2,POCKET_BOTTOM-low,7.9),mat)
    additions.append((tag,clone(filler,'_filler QA '+tag)))
    boolean(part,filler,'UNION')
    bulge=bump(mat);additions.append((tag,clone(bulge,'_bulge QA '+tag)))
    boolean(part,bulge,'UNION')
    boolean(part,local_box('_measured solder channel',(0,ROW_V,(FLOOR-1.4)/2),
                          (WIDTH,HEIGHT,FLOOR+1.4),mat))
    # Shorten only the pin-side PCB edge. Optical package and the circular ears
    # keep their existing datums; do not scale circles into ellipses.
    changed_names.add(pcb.name)
    boolean(pcb,local_box('_measured PCB width',(0,(PCB_BOTTOM+50)/2,-1.65),
                         (50,50-PCB_BOTTOM,20),mat),'INTERSECT')
    pcb.name='ToF '+tag+' | Reference PCB 25x10.66 R09'
    pcb['R09 measurement']='10.66 mm short dimension; 25 mm length/thickness/ear datums retained assumptions.'
    for prefix in ['Header pad ','Inward header pin ']:
        for i in range(6):
            ob=S.objects['ToF '+tag+' | '+prefix+str(i)]
            changed_names.add(ob.name)
            ob.data=ob.data.copy();ob.data.transform(Matrix.Translation(Vector((0,ROW_SHIFT/100,0))))
            ob['R09 datum']='4.80 mm from nearest optical-package edge to header-row centre.'
    # With the optical reference unchanged, measure width and row centres from
    # actual evaluated mesh vertices, not only generator parameters.
    pcb_local=[mat.inverted()@(pcb.matrix_world@v.co)*100 for v in pcb.data.vertices]
    width=max(p.y for p in pcb_local)-min(p.y for p in pcb_local)
    assert abs(width-PCB_WIDTH)<.005,(tag,width)
    pins=[]
    for i in range(6):
        ob=S.objects['ToF '+tag+' | Header pad '+str(i)]
        vs=[mat.inverted()@(ob.matrix_world@v.co)*100 for v in ob.data.vertices]
        centre=(min(v.y for v in vs)+max(v.y for v in vs))/2
        assert abs(centre-ROW_V)<.005,(tag,centre)
        pins.append(centre)
    depths=[];skins=[]
    for u in [-7.8,-6.35,-3.81,-1.27,0,1.27,3.81,6.35,7.8]:
        for v in [ROW_V-1.9,ROW_V,ROW_V+1.9]:
            d=ray(part,mat,u,v,PCB_FACE,1);outer=ray(part,mat,u,v,OUTSIDE+1,-1)
            assert d is not None and abs(d-CLEARANCE)<.005,(tag,u,v,d)
            assert outer is not None and abs(outer-1)<.005,(tag,u,v,outer)
            depths.append(d);skins.append(OUTSIDE+1-outer-PCB_FACE-d)
    assert min(skins)>1.19
    # Restored wall locations and fixed ear/optical-side seating patches.
    support=[ray(part,mat,u,v,PCB_FACE-.1,1) for u,v in [(0,2),(10.9,-3.2),(-10.9,-3.2)]]
    assert all(d is not None and abs(d-.1)<.005 for d in support),(tag,support)
    start=mat@(Vector((0,PCB_BOTTOM,-4))/100)
    direction=mat.to_3x3()@Vector((0,-1,0));inv=part.matrix_world.inverted()
    hit,at,_,_=part.ray_cast(inv@start,(inv.to_3x3()@direction).normalized())
    board_margin=(part.matrix_world@at-start).length*100 if hit else None
    assert board_margin is not None and abs(board_margin-.3)<.005,(tag,board_margin)
    pin_probe=local_box('_solder envelope',(0,ROW_V,PCB_FACE+.75),(15.4,3.6,1.5),mat)
    pin_volume=intersection(part,pin_probe);remove(pin_probe);assert pin_volume<.02
    for ob in [o for o in S.objects if o.get('ToF station')==tag and o.type=='MESH']:
        volume=contact_volume(ob,part)
        if volume>.04:report['reference_clashes'].append([tag,ob.name,volume])
    report['stations'].append({'station':tag,'PCB_width_mm':width,'header_row_v_mm':pins,
        'min_solder_clearance_mm':min(depths),'max_solder_clearance_mm':max(depths),
        'min_closed_skin_mm':min(skins),'PCB_pin_edge_clearance_mm':board_margin,
        'unchanged_support_probe_distances_mm':support,'solder_envelope_intersection_mm3':pin_volume,
        'matrix':[[float(x) for x in r] for r in mat]})
    print('R09_STATION',tag,flush=True)

assert not report['reference_clashes'],report['reference_clashes']
environment=[o for o in S.objects if o.name in ORIGINAL and o not in [main,cap,coupon]
    and o.type=='MESH' and not o.hide_render and not any(c.name=='06 Studio' for c in o.users_collection)]
for tag,region in additions:
    # Only solid material remaining after the channel cut matters for contacts.
    boolean(region,clone(cap if tag=='Right' else main,'_actual local material'),'INTERSECT')
    for other in environment:
        vol=contact_volume(other,region)
        if vol>.04:report['addition_clashes'].append([tag,other.name,vol])
    remove(region)
assert not report['addition_clashes'],report['addition_clashes']

report['preserved_R07_surfaces']=[]
for part,original in zip([main,cap],base_copies):
    t=tree(part);distances=[];invs=[m.inverted() for m in matrices.values()]
    for vert in original.data.vertices:
        p=original.matrix_world@vert.co
        locals_=[inv@p*100 for inv in invs]
        # Precise union of intentional edit boxes: moved bump/channel and
        # restored lower wall. All other original features must be unchanged.
        def edit(q):
            return (abs(q.x)<10.5 and -13.9<q.y<-1.9 and -1.7<q.z<2.7) or (abs(q.x)<13.3 and -10.2<q.y<-7.7 and -8.2<q.z<.2)
        if any(edit(q) for q in locals_):continue
        distances.append(t.find_nearest(p)[3]*100)
    assert max(distances)<.01,(part.name,max(distances))
    report['preserved_R07_surfaces'].append({'part':part.name,'vertices_checked':len(distances),'max_error_mm':max(distances)})
    remove(original)

# Optical rays from both apertures, up to a conservative 40-degree full cone.
for tag,mat in matrices.items():
    for ox in [-1.7,-1,1,1.7]:
        origin=mat@(Vector((ox,0,.19))/100)
        for tilt in [0,10,20]:
            for az in range(0,360,30):
                r=math.radians(tilt);a=math.radians(az)
                direction=mat.to_3x3()@Vector((math.sin(r)*math.cos(a),math.sin(r)*math.sin(a),math.cos(r)))
                for part in [main,cap]:
                    inv=part.matrix_world.inverted()
                    hit,p,_,_=part.ray_cast(inv@origin,(inv.to_3x3()@direction).normalized())
                    if hit and .001<(part.matrix_world@p-origin).length*100<300:
                        report['optical_ray_obstructions'].append([tag,tilt,az,part.name])
assert not report['optical_ray_obstructions'],report['optical_ray_obstructions']
report['panel_separation']=[];start=cap.matrix_world.copy()
for offset in [0,1,3,7,12,20,40]:
    cap.matrix_world=start.copy();cap.matrix_world.translation.x+=offset/100
    vol=intersection(main,cap);assert vol<.03,(offset,vol)
    report['panel_separation'].append({'offset_x_mm':offset,'intersection_mm3':vol})
cap.matrix_world=start;bpy.context.view_layer.update()

trial=clone(main,'_R09 coupon');trial.data.transform(matrices['Front_C'].inverted())
boolean(trial,box('_coupon boundary',(0,-3.2,-2.5),(43,27,15)),'INTERSECT')
coupon.data=trial.data.copy();remove(trial)
coupon['R09 fit']='10.66 mm board / 4.80 mm optic-edge to pin centre / 2 mm solder depth. Print this coupon first.'
for ob in [main,cap]:
    ob['R09 fit']='Pin-row relief moved 1.9 mm toward optics; pin-side board cavity narrowed by 1.94 mm.'
unexpected=[n for n,h in ORIGINAL.items() if n not in changed_names and (n not in S.objects or snapshot(S.objects[n])!=h)]
assert not unexpected,unexpected
report['allowed_original_changes']=sorted(changed_names)
report['renamed_reference_PCBS']={old:'ToF '+tag+' | Reference PCB 25x10.66 R09' for tag,old in original_references.items()}
report['unexpected_original_changes']=unexpected
report['unchanged_original_objects']=len(ORIGINAL)-len(changed_names)
report['contact_check_tolerance']='P17 QA copies separated 0.02 mm inward along sensor normal; other component QA copies contracted 0.2%, to distinguish intended contact from interference. Delivered models unscaled and unmoved.'
report['solid_checks']={o.name:metrics(o) for o in [main,cap,coupon]}
assert all(m['components']==1 and m['nonmanifold_edges']==0 and m['volume_mm3']>0 for m in report['solid_checks'].values())

manifest=json.loads((OLDPACK/'manifest.json').read_text());records=[];same=[]
for old in manifest['parts']:
    row=dict(old);pid=row['id'];path=PACK/row['file'];path.parent.mkdir(parents=True,exist_ok=True)
    if pid not in ['P16A','P16B','P19']:
        shutil.copy2(OLDPACK/row['file'],path);assert HASH(path)==row['sha256'];same.append(pid)
    else:
        obj={'P16A':main,'P16B':cap,'P19':coupon}[pid]
        rot=Matrix.Identity(4) if pid=='P16A' else Matrix.Rotation(math.pi/2 if pid=='P16B' else math.pi,4,'Y' if pid=='P16B' else 'X')
        row['reimport_check']=export(obj,path,rot);row['sha256']=HASH(path);row['sources']=[obj.name]
        row['revision']='R09';row['sensor_edge_to_pin_row_centre_mm']=4.8;row['PCB_short_dimension_mm']=10.66
        row['export_rotation_matrix']=[[float(v) for v in r] for r in rot]
        row['brim_4mm_fits_K1C']=all(d+8<220 for d in row['reimport_check']['dimensions_mm'][:2])
        row['brim_2mm_fits_nominal_K1C']=True
        row['note']={'P16A':'R09: original bottom down, 2 mm brim starting point. Seven measured ToF pockets.',
            'P16B':'R09: outside +X face down. One measured ToF pocket. Original M4-station foot unchanged.',
            'P19':'R09 actual front-centre pocket. Test the measured soldered board with existing P17/P18 and M2.5 hardware before either skirt half.'}[pid]
    records.append(row)
assert len(same)==42
report['unchanged_STL_ids']=same
manifest.update({'revision':'R09 measured 10.66 mm ToF board and 4.8 mm sensor-edge/pin-row spacing',
    'source':'roam_tof_measured_fit_r09.blend','parts':records,'unchanged_parts_source':str(OLDPACK),
    'changed_from_R08':['P16A','P16B','P19'],'measured_ToF_fit':report['derived_design'],
    'remaining_ToF_assumptions':report['remaining_assumptions']})
manifest.pop('R08_assumptions',None)
changed=[r for r in records if r['id'] in ['P16A','P16B','P19']]
for target,rows in [(PACK,records),(CHANGED,changed)]:
    target.mkdir(parents=True,exist_ok=True)
    for row in rows:
        if target!=PACK:
            p=target/row['file'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(PACK/row['file'],p)
    (target/'manifest.json').write_text(json.dumps({**manifest,'parts':rows,'unique_stl_files':len(rows),
        'total_physical_pieces_including_optional':sum(r['quantity'] for r in rows)},indent=2))
    with (target/'PRINT_PARTS.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['ID','Part','Quantity','STL file','X mm','Y mm','Z mm','Supports','Notes'])
        for r in rows:w.writerow([r['id'],r['label'],r['quantity'],r['file'],*[round(d,3) for d in r['reimport_check']['dimensions_mm']],r['support'],r['note']])
for name in ['HARDWARE.csv','R07_JOINTS.csv']:shutil.copy2(OLDPACK/name,PACK/name)
(OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
S['Mechanical revision']='R09 measured ToF fit; R07 split shells and R06 electronics retained'
S['Revision']=S['Mechanical revision'];S['ToF fit']='R09: board short dimension 10.66 mm; optical edge to pin-row centre 4.80 mm; remaining board length, thickness, ear datums provisional.'
S['R09 measured fit']='All eight solder rows 6.0 mm from optic centre, 1.9 mm closer than R08. 2 mm solder depth. Pocket short dimension 11.26 mm; pin-side edge tolerance 0.3 mm.'
S.pop('R08 solder relief',None);S['Geometry checks']='tof_measured_fit_r09/geometry_checks.json'
bpy.ops.object.select_all(action='DESELECT');main.select_set(True);bpy.context.view_layer.objects.active=main
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_tof_measured_fit_r09.blend'),compress=True)
S.render.engine='CYCLES';S.cycles.samples=24;S.cycles.device='CPU'
S.render.resolution_x=1500;S.render.resolution_y=1050;S.render.resolution_percentage=100
render('robot_assembled.png')
for o in S.objects:
    if o not in [main,cap] and not o.name.startswith(('ToF ','P17 |','P18 |','T1.','T2.','T3.','T4.','T5.','T6.','T7.','T8.')) and o.type not in ['CAMERA','LIGHT'] and not any(c.name=='06 Studio' for c in o.users_collection):
        o.hide_render=True;o.hide_set(True)
aim((180,-300,270),(0,0,38),330)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_tof_measured_fit_r09_skirt_inspection.blend'),compress=True)
render('skirt_inspection.png')

for o in list(S.objects):
    if o.type not in ['CAMERA','LIGHT']:remove(o)
grey=material('R08 reference grey',(.30,.36,.39));teal=material('R09 teal',(.025,.32,.35))
white=material('Labels',(.8,.9,.92));floor=material('Floor',(.025,.039,.052))
old=bpy.data.objects.new('R08 reference only - DO NOT PRINT',old_coupon_mesh);S.collection.objects.link(old)
old.data.transform(Matrix.Rotation(math.pi,4,'X'));bb=bounds(old)
old.location=Vector((-sum(bb[0])/2,-sum(bb[1])/2,-bb[2][0]))/100
bpy.ops.wm.stl_import(filepath=str(PACK/next(r['file'] for r in changed if r['id']=='P19')),use_facet_normal=False)
new=bpy.context.object;new.name='R09 P19 actual STL';new.data.transform(Matrix.Scale(.01,4))
for o,x,ma in [(old,-28,grey),(new,28,teal)]:
    o.location.x+=x/100;o.data.materials.clear();o.data.materials.append(ma)
    for p in o.data.polygons:p.material_index=0
label('R08  |  row 7.9 mm from optic centre',(-28,-23,.2),2.1)
label('R09  |  row 6.0 mm from optic centre',(28,-23,.2),2.1)
f=box('Display floor NOT PRINTABLE',(0,0,-1),(500,500,2));f.data.materials.append(floor)
aim((38,-70,140),(0,1,2),125)
S['Mechanical revision']='R09 pocket comparison; use supplied STLs, display floor/text not printable'
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'roam_tof_measured_fit_r09_fit_comparison.blend'),compress=True)
render('measured_fit_comparison.png')
assert HASH(SOURCE)==source_hash and HASH(BASEFILE)==base_hash
print('R09_COMPLETE',json.dumps({'unchanged_STLs':len(same),'changed_STLs':[(r['id'],r['reimport_check']['dimensions_mm']) for r in changed]}),flush=True)
