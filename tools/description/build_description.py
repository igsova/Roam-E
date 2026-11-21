#!/usr/bin/env python3
"""Generate the URDF files of roam_e_description from the Blender export.

Reads config/mesh_manifest.json, written by export_blender_description.py, and writes:

- urdf/visuals.xacro: the visual geometry of every link and the rigid_links macro;
- config/calibration_cad_nominal.yaml: the CAD-nominal frames and wheel geometry;
- config/geometry.yaml: wheel geometry and the footprint hull;
- urdf/roam_e.urdf: the expanded URDF, with the camera optical frames.

config/calibration.yaml holds the measured frames. It is created from the CAD-nominal
values when it is missing and is otherwise left alone, unless --reset-calibration is
given. Requires the xacro and PyYAML Python packages.
"""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml

ROOT = Path(__file__).resolve().parents[2]
XACRO_NS = 'http://www.ros.org/wiki/xacro'
PACKAGE_URI = 'package://roam_e_description/'
WHEEL_LINKS = ('left_wheel_link', 'right_wheel_link')
FOOTPRINT_PADDING_M = 0.01
LONG_TEXT = 80  # characters; longer YAML strings are written folded

CALIBRATION_HEADER = """\
# Frames and wheel geometry used by urdf/roam_e.urdf.xacro. The robot launch files in
# src/roam_e/launch load this file through launch/display.launch.py.
#
# Values start from the CAD model (config/calibration_cad_nominal.yaml) and are replaced
# by measurements; each frame's note says where its values come from. build_description.py
# never overwrites this file unless it is run with --reset-calibration.
#
# Each frame gives xyz (metres) and rpy (radians, roll-pitch-yaw about the fixed X, Y, Z
# axes) relative to its parent. After an edit, run tools/description/validate_description.py.
# To try other values without editing this file, pass calibration_file:=/path/to/copy.yaml
# to display.launch.py.
#
# physical_wheels sets the wheel joints, the height of base_link and the wheel mesh scale.
# effective_odometry records the floor-calibrated values that the encoder odometry node
# uses; the URDF does not read them.
"""

NOMINAL_HEADER = """\
# CAD-nominal frames and wheel geometry, written by tools/description/build_description.py.
# Do not edit: the URDF reads config/calibration.yaml. Copy an entry from here to reset
# a frame to its CAD value. Distances in metres, angles in radians.
"""

GEOMETRY_HEADER = """\
# Written by tools/description/build_description.py from config/mesh_manifest.json.
# Do not edit. Distances in metres.
"""

URDF_COMMENT = (
    ' Written by tools/description/build_description.py from urdf/roam_e.urdf.xacro and '
    'config/calibration.yaml, with the camera optical frames. Do not edit. '
)

ET.register_namespace('xacro', XACRO_NS)


def xacro_tag(name: str) -> str:
    return f'{{{XACRO_NS}}}{name}'


class _Dumper(yaml.SafeDumper):
    """Writes lists of numbers, such as xyz and rpy, on one line and long text folded."""


def _represent_list(dumper, data):
    flow = all(isinstance(item, (int, float)) for item in data)
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=flow)


def _represent_str(dumper, data):
    style = '>' if len(data) > LONG_TEXT else None
    return dumper.represent_scalar('tag:yaml.org,2002:str', data, style=style)


_Dumper.add_representer(list, _represent_list)
_Dumper.add_representer(str, _represent_str)


def dump_yaml(data: dict) -> str:
    return yaml.dump(data, Dumper=_Dumper, sort_keys=False, width=90)


def visuals_xacro(manifest: dict) -> str:
    """Return the visuals.xacro document for the meshes in the manifest."""
    radius = manifest['physical_wheel_radius_m']
    width = manifest['physical_wheel_width_m']
    # The wheel meshes are exported at the physical radius and width; scale them to the
    # values in the calibration file.
    wheel_scale = (
        f'${{wheel_radius / {radius}}} ${{wheel_width / {width}}} ${{wheel_radius / {radius}}}'
    )

    root = ET.Element('robot')
    macro = ET.SubElement(root, xacro_tag('macro'), name='link_visuals', params='link')
    for link, visuals in sorted(manifest['visuals'].items()):
        condition = ET.SubElement(macro, xacro_tag('if'), value=f"${{link == '{link}'}}")
        for index, visual in enumerate(visuals):
            element = ET.SubElement(condition, 'visual', name=f'{link}_visual_{index}')
            geometry = ET.SubElement(element, 'geometry')
            ET.SubElement(
                geometry,
                'mesh',
                filename=PACKAGE_URI + visual['mesh'],
                scale=wheel_scale if link in WHEEL_LINKS else '1 1 1',
            )
            material = ET.SubElement(element, 'material', name=f'{link}_material_{index}')
            ET.SubElement(material, 'color', rgba=' '.join(str(c) for c in visual['rgba']))

    # Every link except the wheels is fixed to its parent frame in the calibration file.
    rigid = ET.SubElement(root, xacro_tag('macro'), name='rigid_links', params='')
    for link in sorted(manifest['visuals']):
        if link in WHEEL_LINKS:
            continue
        element = ET.SubElement(rigid, 'link', name=link)
        ET.SubElement(element, xacro_tag('link_visuals'), link=link)
        if link != 'base_link':
            ET.SubElement(rigid, xacro_tag('fixed_frame'), child=link)

    ET.indent(root)
    return (
        '<?xml version="1.0"?>\n'
        '<!-- Written by tools/description/build_description.py; do not edit. Units: metres. -->\n'
        + ET.tostring(root, encoding='unicode')
        + '\n'
    )


def nominal_calibration(manifest: dict) -> dict:
    """CAD-nominal calibration: the manifest frames, without the wheel joints."""
    frames = {name: f for name, f in manifest['frames'].items() if name not in WHEEL_LINKS}
    return {
        'forward': manifest['forward'],
        'physical_wheels': {
            'radius': manifest['physical_wheel_radius_m'],
            'separation': manifest['physical_wheel_separation_m'],
            'width': manifest['physical_wheel_width_m'],
        },
        'effective_odometry': {
            'radius': manifest['effective_odometry_radius_m'],
            'separation': manifest['effective_odometry_separation_m'],
            'note': 'Floor-calibrated values used by the encoder odometry; not read by the URDF',
        },
        'frames': frames,
    }


def geometry_summary(manifest: dict) -> dict:
    return {
        'mesh_units': 'metres',
        'forward': manifest['forward'],
        'wheel_radius': manifest['physical_wheel_radius_m'],
        'wheel_separation': manifest['physical_wheel_separation_m'],
        'wheel_width': manifest['physical_wheel_width_m'],
        'footprint': {
            'points': manifest['footprint'],
            'padding': FOOTPRINT_PADDING_M,
            'note': 'Convex hull of all visual meshes projected onto the base_link XY plane',
        },
        'calibration_file': 'calibration.yaml',
        'limits': manifest['limits'],
    }


def generate(package: Path, reset_calibration: bool = False) -> dict:
    """Write the generated files of the package and return the manifest."""
    manifest = json.loads((package / 'config/mesh_manifest.json').read_text())
    (package / 'urdf/visuals.xacro').write_text(visuals_xacro(manifest))

    nominal = dump_yaml(nominal_calibration(manifest))
    (package / 'config/calibration_cad_nominal.yaml').write_text(NOMINAL_HEADER + nominal)

    calibration = package / 'config/calibration.yaml'
    if reset_calibration or not calibration.exists():
        calibration.write_text(CALIBRATION_HEADER + nominal)
    else:
        forward = yaml.safe_load(calibration.read_text())['forward']
        if forward != manifest['forward']:
            raise ValueError(
                f'{calibration} is for forward={forward} but the export uses '
                f'forward={manifest["forward"]}; fix the file or pass --reset-calibration'
            )

    geometry = dump_yaml(geometry_summary(manifest))
    (package / 'config/geometry.yaml').write_text(GEOMETRY_HEADER + geometry)

    urdf = xacro.process_file(
        str(package / 'urdf/roam_e.urdf.xacro'),
        mappings={
            'calibration_file': str(calibration),
            'include_camera_optical_frames': 'true',
        },
    )
    # Replace the xacro banner, which contains the absolute path of the input file.
    for node in list(urdf.childNodes):
        if node.nodeType == node.COMMENT_NODE:
            urdf.removeChild(node)
    urdf.insertBefore(urdf.createComment(URDF_COMMENT), urdf.documentElement)
    (package / 'urdf/roam_e.urdf').write_text(urdf.toprettyxml(indent='  '))
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    parser.add_argument('--package', type=Path, default=ROOT / 'src/roam_e_description')
    parser.add_argument(
        '--reset-calibration',
        action='store_true',
        help='Overwrite config/calibration.yaml with the CAD-nominal values',
    )
    args = parser.parse_args()
    generate(args.package.resolve(), args.reset_calibration)
    state = 'reset to CAD nominal' if args.reset_calibration else 'left unchanged'
    print(
        'Wrote urdf/visuals.xacro, urdf/roam_e.urdf, config/calibration_cad_nominal.yaml '
        f'and config/geometry.yaml; config/calibration.yaml {state}.'
    )


if __name__ == '__main__':
    main()
