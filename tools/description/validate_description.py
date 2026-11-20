#!/usr/bin/env python3
"""Check R19 meshes, TF tree, both camera modes and calibration overrides.

No running ROS nodes or connected hardware are required. Requires xacro/PyYAML.
"""
from __future__ import annotations
import hashlib
import argparse
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET
import xacro
import yaml

OPTICS={'camera_rgb_optical_frame','camera_depth_optical_frame','camera_ir_optical_frame'}


def binary_stl_stats(path):
    with path.open('rb') as f:
        f.read(80);n=struct.unpack('<I',f.read(4))[0]
        assert path.stat().st_size==84+50*n,(path,n)
        lo=[math.inf]*3;hi=[-math.inf]*3
        for _ in range(n):
            values=struct.unpack('<12fH',f.read(50))
            assert all(math.isfinite(v) for v in values[:12]),path
            for start in [3,6,9]:
                for axis in range(3):
                    lo[axis]=min(lo[axis],values[start+axis]);hi[axis]=max(hi[axis],values[start+axis])
        assert n>0 and max(abs(v) for v in lo+hi)<1,(path,lo,hi)
        return {'triangles':n,'bounds_m':list(map(list,zip(lo,hi)))}


def check_tree(root):
    links={n.get('name') for n in root.findall('link')}
    joints=root.findall('joint');names=[n.get('name') for n in joints]
    assert len(names)==len(set(names))
    assert len(links)==len(root.findall('link'))
    parents={};edges={}
    for j in joints:
        parent=j.find('parent').get('link');child=j.find('child').get('link')
        assert parent in links and child in links and child not in parents
        parents[child]=parent;edges[child]=j
        for attr in ['xyz','rpy']:
            vals=[float(v) for v in j.find('origin').get(attr,'0 0 0').split()]
            assert len(vals)==3 and all(math.isfinite(v) for v in vals)
    roots=links-set(parents)
    assert roots=={'base_footprint'} and len(joints)==len(links)-1
    for child in parents:
        current=child;seen=set()
        while current in parents:
            assert current not in seen,'TF cycle';seen.add(current);current=parents[current]
        assert current=='base_footprint'
    return links,edges


def xyz(j):return [float(v) for v in j.find('origin').get('xyz').split()]


def rotation(rpy):
    r,p,y=rpy;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
            [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]


def validate(workspace, source_workspace=None, expected_imu_rpy=(math.pi, 0, 0)):
    package=workspace/'src/roam_e_description'
    manifest=json.loads((package/'config/mesh_export_r19.json').read_text())
    assert hashlib.sha256(((source_workspace or workspace)/manifest['source']).read_bytes()).hexdigest()==manifest['source_sha256']
    ET.parse(package/'package.xml')
    config=yaml.safe_load((package/'config/calibration.yaml').read_text())
    assert config['forward']==manifest['forward'],'Calibration/mesh convention mismatch'
    main=package/'urdf/roam_e.urdf.xacro'
    def expanded(file=None,optics=True):return ET.fromstring(xacro.process_file(str(main),mappings={
        'calibration_file':str(file or package/'config/calibration.yaml'),
        'include_camera_optical_frames':str(optics).lower()}).toxml())
    preview=expanded();live=expanded(optics=False)
    links,edges=check_tree(preview);live_links,_=check_tree(live)
    assert OPTICS<=links and not OPTICS&live_links
    expected={'base_footprint','base_link','imu_link',*manifest['visuals'],*OPTICS}
    assert links==expected,(links,expected)
    meshes={}
    refs=set()
    for element in preview.findall('.//geometry/mesh'):
        uri=element.get('filename');prefix='package://roam_e_description/'
        assert uri.startswith(prefix)
        path=(package/uri[len(prefix):]).resolve()
        assert path.is_relative_to(package.resolve()) and path.is_file()
        scale=[float(v) for v in element.get('scale').split()]
        wheel_mesh=path.name.startswith(('left_wheel_link_','right_wheel_link_'))
        expected_scale=([config['physical_wheels']['radius']/.033875,
                         config['physical_wheels']['width']/.029,
                         config['physical_wheels']['radius']/.033875] if wheel_mesh else [1,1,1])
        assert max(abs(a-b) for a,b in zip(scale,expected_scale))<1e-12,'Unexpected mesh scale'
        refs.add(path.name)
    assert refs==set(manifest['mesh_stats'])
    for name,s in manifest['mesh_stats'].items():
        path=package/'meshes/r19'/name
        assert hashlib.sha256(path.read_bytes()).hexdigest()==s['sha256']
        actual=binary_stl_stats(path);meshes[name]=actual
        assert actual['triangles']==s['triangles']
        assert max(abs(a-b) for aa,bb in zip(actual['bounds_m'],s['bounds_m']) for a,b in zip(aa,bb))<1e-7
    assert len([n for n in links if n.startswith('tof_')])==8
    for child,f in config['frames'].items():
        if child not in edges:continue
        assert max(abs(a-b) for a,b in zip(xyz(edges[child]),f['xyz']))<1e-12
    # Verify range beams point outward from the body, with +Z up.
    for name in sorted(n for n in links if n.startswith('tof_')):
        f=config['frames'][name];rot=rotation(f['rpy'])
        dx=f['xyz'][0]-(.037 if manifest['forward']=='caster' else -.037);dy=f['xyz'][1]
        assert rot[0][0]*dx+rot[1][0]*dy>0
        assert abs(rot[2][2]-1)<1e-6
    # Optical +Z looks along camera-body +X, +X is right and +Y is down.
    for name in OPTICS:
        r=rotation(config['frames'][name]['rpy'])
        assert abs(r[0][2]-1)<1e-6 and abs(r[1][0]+1)<1e-6 and abs(r[2][1]+1)<1e-6
    # The camera faces the forward axis; measured mount corrections are a few degrees.
    camyaw=config['frames']['camera_link']['rpy'][2]
    assert math.cos(camyaw)*(-1 if manifest['forward']=='caster' else 1)>math.cos(math.radians(10))
    actual_imu=rotation(config['frames']['imu_link']['rpy'])
    expected_imu=rotation(expected_imu_rpy)
    assert max(abs(a-b) for aa,bb in zip(actual_imu,expected_imu)
               for a,b in zip(aa,bb))<1e-8,'Unexpected candidate IMU rotation'
    # Only tyre/hub components move; the source objects' motor bodies stay base rigid.
    assert len(manifest['wheel_separation_checks'])==2
    assert all(w['rotating_vertices']==35630 for w in manifest['wheel_separation_checks'])
    assert max(r['bounds_error_m'] for r in manifest['source_objects'])<=.0005
    exported={r['object'] for r in manifest['source_objects']}
    assert not exported & set(manifest['excluded_objects'])
    assert 'Component | LIDAR' not in exported # superseded pre-R15 reference
    assert all('reserve' not in n.lower() for n in exported)
    for side,sign in [('left',1),('right',-1)]:
        j=edges[side+'_wheel_link']
        assert j.get('type')=='continuous' and j.find('axis').get('xyz')=='0 1 0'
        assert abs(xyz(j)[1]-sign*config['physical_wheels']['separation']/2)<1e-12
        bb=[meshes[Path(v['mesh']).name]['bounds_m'] for v in manifest['visuals'][side+'_wheel_link']]
        radial=config['physical_wheels']['radius']/.033875
        low=min(b[2][0] for b in bb)*radial;high=max(b[2][1] for b in bb)*radial
        assert abs(low+config['physical_wheels']['radius'])<.00002
        assert abs(high-config['physical_wheels']['radius'])<.00002
    # An edited file must actually change TF and physical wheel geometry.
    edited=yaml.safe_load((package/'config/calibration.yaml').read_text())
    edited['frames']['laser_frame']['xyz'][0]+=.01
    edited['frames']['tof_front_c_link']['rpy'][2]+=.05
    edited['physical_wheels']['separation']+=.002
    edited['physical_wheels']['radius']+=.001
    with tempfile.TemporaryDirectory(prefix='roam-r19-calibration-') as tmp:
        path=Path(tmp)/'override.yaml';path.write_text(yaml.safe_dump(edited))
        override=expanded(path);_,over_edges=check_tree(override)
        assert abs(xyz(over_edges['laser_frame'])[0]-xyz(edges['laser_frame'])[0]-.01)<1e-10
        assert abs(xyz(over_edges['left_wheel_link'])[1]-xyz(edges['left_wheel_link'])[1]-.001)<1e-10
        assert abs(xyz(over_edges['base_link'])[2]-xyz(edges['base_link'])[2]-.001)<1e-10
        scales=[m.get('scale') for m in override.findall("link[@name='left_wheel_link']/visual/geometry/mesh")]
        assert all(float(s.split()[0])>1 for s in scales)
    default=ET.parse(package/'urdf/roam_e.urdf').getroot()
    # Ignore pretty printing and generated comments when comparing XML.
    def normalize(n):
        n.text=None;n.tail=None
        for c in n:normalize(c)
    normalize(default);normalize(preview)
    assert ET.tostring(default)==ET.tostring(preview),'Expanded URDF is stale'
    rot=manifest['cad_to_ros_rotation']
    det=(rot[0][0]*(rot[1][1]*rot[2][2]-rot[1][2]*rot[2][1])
        -rot[0][1]*(rot[1][0]*rot[2][2]-rot[1][2]*rot[2][0])
        +rot[0][2]*(rot[1][0]*rot[2][1]-rot[1][1]*rot[2][0]))
    assert abs(det-1)<1e-9,'CAD to ROS must not mirror geometry'
    return {'status':'PASS','revision':'R19','forward':manifest['forward'],
        'preview_links':len(links),'live_links':len(live_links),'mesh_files':len(meshes),
        'triangles':sum(m['triangles'] for m in meshes.values()),'ToF_stations':8,
        'source_blend_unchanged':True,'calibration_override_test':'PASS','camera_tf_modes':'PASS',
        'bounds_m':manifest['assembled_bounds_m'],
        'not_verified':['ROS colcon build and RViz runtime (ROS not installed on this Mac)',
            'Physical sensor extrinsics, LiDAR scan plane/yaw and jumper dimensions',
            'Simulation dynamics/inertias and Nav2 runtime footprint']}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace',type=Path,nargs='?',default=Path('.'))
    parser.add_argument('--source-workspace',type=Path,
                        help='Read the unchanged source Blender file from this checkout')
    parser.add_argument('--expected-imu-rpy',type=float,nargs=3,default=(0,0,math.pi/2),
                        help='Expected imu_link rotation relative to imu_mount_link (measured mount in calibration.yaml)')
    args=parser.parse_args()
    workspace=args.workspace.resolve()
    result=validate(workspace,args.source_workspace,args.expected_imu_rpy)
    out=workspace/'build/description';out.mkdir(parents=True,exist_ok=True)
    (out/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
