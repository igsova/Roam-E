# Tools

Scripts that run outside the ROS launch files.

## `rtabmap/`: visual relocalization and map export

| Script | What it does |
|---|---|
| `visual_keyframes.py` | Builds a keyframe cache of learned features (XFeat or SuperPoint) with 3D map points from an RTAB-Map export |
| `visual_relocalizer.py` | ROS node. Matches the live colour image against the keyframe cache (LighterGlue + PnP) and sends the pose to RTAB-Map as an initial pose when it is lost or on request. Never commands motion |
| `visual_features.py` | Feature extraction and matching shared by the two scripts above |
| `publish_ply_cloud.py` | Publishes an RTAB-Map dense point-cloud export once as a latched `PointCloud2`, for Foxglove |

Commands are in [`docs/ROBOT_COMMANDS.md`](../docs/ROBOT_COMMANDS.md).

## `calibration/`: camera checks

Both scripts only subscribe; run them on the Pi with the camera running.

| Script | What it does |
|---|---|
| `measure_camera_wall.py` | Compares the depth camera with the lidar on a flat wall and reports the camera's yaw, pitch and depth-scale error |
| `measure_depth_color_offset.py` | Measures the pixel offset between the Astra's registered depth and colour images |

## `description/`: robot model from CAD

| Script | What it does |
|---|---|
| `export_blender_description.py` | Exports link-relative visual meshes and sensor frames from [`mechanical/roam_e.blend`](../mechanical/roam_e.blend). Run it inside Blender: `blender --background --python tools/description/export_blender_description.py` |
| `build_description.py` | Generates the visual xacro and config files of `roam_e_description` from that export |
| `validate_description.py` | Checks the meshes, the TF tree, both camera TF modes and the calibration overrides, without ROS running |

The firmware build and host tests live in [`firmware/`](../firmware/).
