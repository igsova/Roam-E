# Tools

Scripts that run outside the ROS launch files.

## `rtabmap/`: visual relocalization and map export

| Script | What it does |
|---|---|
| `visual_keyframes.py` | Builds a keyframe cache of learned features (XFeat or SuperPoint) with 3D map points from an RTAB-Map export |
| `visual_relocalizer.py` | ROS node. Matches the live colour image against the keyframe cache (LighterGlue + PnP) and sends the pose to RTAB-Map as an initial pose when it is lost or on request. Never commands motion |
| `visual_features.py` | Feature extraction and matching shared by the two scripts above |
| `publish_ply_cloud.py` | Publishes an RTAB-Map dense point-cloud export once as a latched `PointCloud2`, for Foxglove |

Commands are in [`docs/quick_commands.md`](../docs/quick_commands.md).

## `calibration/`: camera checks

Both scripts only subscribe; run them on the Pi with the camera running.

| Script | What it does |
|---|---|
| `measure_camera_wall.py` | Compares the depth camera with the lidar on a flat wall and reports the camera's yaw, pitch and depth-scale error |
| `measure_depth_color_offset.py` | Measures the pixel offset between the Astra's registered depth and colour images |

## `description/`: robot model from CAD

| Script | What it does |
|---|---|
| `export_blender_description.py` | Runs inside Blender. Exports the visual meshes, in link frames, and the CAD-nominal sensor frames from [`mechanical/roam_e.blend`](../mechanical/roam_e.blend) without saving it |
| `build_description.py` | Generates `visuals.xacro`, the CAD-nominal calibration, `geometry.yaml` and the expanded `roam_e.urdf` from that export. Never overwrites `calibration.yaml` |
| `validate_description.py` | Checks the meshes, the TF tree, both camera TF modes, the sensor frame conventions and the calibration overrides, without ROS running |

Run them in order from the repository root. The last two need the `xacro` and
`PyYAML` Python packages.

```bash
blender --background --python tools/description/export_blender_description.py
python3 tools/description/build_description.py
python3 tools/description/validate_description.py
```

On macOS, `blender` is `/Applications/Blender.app/Contents/MacOS/Blender`. The
package README, [`src/roam_e_description`](../src/roam_e_description/README.md),
describes the frames and the calibration file.

The firmware build and host tests live in [`firmware/`](../firmware/).
