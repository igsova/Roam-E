"""Offline regression checks; run with python3 -m pytest (xacro/PyYAML needed)."""
import importlib.util
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import xacro
import yaml
import pytest

PKG=Path(__file__).resolve().parents[1]
WORKSPACE=PKG.parents[1]


def expand(optics=False,config=None):
    return ET.fromstring(xacro.process_file(str(PKG/'urdf/roam_e.urdf.xacro'),mappings={
        'include_camera_optical_frames':str(optics).lower(),
        'calibration_file':str(config or PKG/'config/calibration.yaml')}).toxml())


def test_camera_tf_is_opt_in():
    default=ET.fromstring(xacro.process_file(str(PKG/'urdf/roam_e.urdf.xacro')).toxml())
    assert default.find("link[@name='camera_link']") is not None
    assert not default.findall("link[@name='camera_rgb_optical_frame']")
    assert len(expand(True).findall('link'))-len(expand(False).findall('link'))==3
    launch=(PKG/'launch/display.launch.py').read_text()
    assert "'include_camera_optical_frames', default_value='false'" in launch


def test_physical_and_effective_wheels_are_separate():
    c=yaml.safe_load((PKG/'config/calibration_nominal_r19.yaml').read_text())
    assert c['physical_wheels']['radius']==.033875
    assert c['effective_odometry']['radius']==.033975
    assert c['physical_wheels']['separation']==.173630
    assert c['effective_odometry']['separation']==.175150


def test_sensor_offsets_are_not_hardcoded():
    c=yaml.safe_load((PKG/'config/calibration.yaml').read_text())
    c['frames']['laser_frame']['xyz']=[.1,.2,.3]
    with tempfile.TemporaryDirectory() as folder:
        p=Path(folder)/'test.yaml';p.write_text(yaml.safe_dump(c))
        root=expand(config=p)
        j=root.find("joint[@name='laser_frame_joint']/origin")
        assert [float(v) for v in j.get('xyz').split()]==[.1,.2,.3]


def test_all_tof_frames_exist():
    root=expand()
    expected={'tof_front_l_link','tof_front_c_link','tof_front_r_link',
              'tof_rear_l_link','tof_rear_c_link','tof_rear_r_link','tof_left_link','tof_right_link'}
    actual={n.get('name') for n in root.findall('link') if n.get('name').startswith('tof_')}
    assert actual==expected


@pytest.mark.skipif(not (WORKSPACE/'tools/description/build_description.py').exists(),
    reason='Regeneration test requires the full source checkout, not just the standalone package ZIP')
def test_regeneration_does_not_erase_calibration():
    spec=importlib.util.spec_from_file_location('r19_builder',WORKSPACE/'tools/description/build_description.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    import shutil
    with tempfile.TemporaryDirectory() as folder:
        p=Path(folder)/'roam_e_description'
        # Generating XML/configs does not read STL contents; copying only sources suffices.
        for directory in ['config','urdf']:shutil.copytree(PKG/directory,p/directory)
        config=yaml.safe_load((p/'config/calibration.yaml').read_text())
        config['frames']['laser_frame']['xyz'][2]+=.001
        path=p/'config/calibration.yaml';path.write_text(yaml.safe_dump(config))
        before=path.read_bytes();module.generate(p)
        assert path.read_bytes()==before
