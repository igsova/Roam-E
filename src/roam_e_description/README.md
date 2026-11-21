# roam_e_description

The URDF model of Roam-E: visual meshes exported from the CAD model in
[`mechanical/roam_e.blend`](../../mechanical/roam_e.blend), the TF frames of the
sensors, and a launch file that publishes them with `robot_state_publisher` and
shows the robot in RViz. `sensors_viz.launch.py` and `roam_e_imu.launch.py` in
`src/roam_e` include that launch file, so the drive, mapping, localization and
navigation launches all publish this model.

| Path | Contents |
|---|---|
| `urdf/roam_e.urdf.xacro` | The model: links, wheel joints and the fixed frames read from the calibration file |
| `urdf/visuals.xacro` | Visual geometry of each link (generated) |
| `urdf/roam_e.urdf` | The expanded model with the default calibration and the optical frames, for tools without xacro (generated) |
| `meshes/visual/` | Binary STL meshes, one per link and colour, in metres in the link frame (generated) |
| `config/calibration.yaml` | The frames and wheel geometry the model uses. Edited by hand |
| `config/calibration_cad_nominal.yaml` | The same frames at their CAD values (generated) |
| `config/mesh_manifest.json` | Export record: source hash, CAD-nominal frames, mesh hashes, triangle counts and bounds (generated) |
| `config/geometry.yaml` | Wheel geometry and the footprint hull (generated) |
| `launch/display.launch.py` | `robot_state_publisher`, a stationary wheel-joint publisher and RViz |
| `test/test_description.py` | Offline checks, run by `colcon test` |

## Frames

`base_link` follows [REP-103](https://www.ros.org/reps/rep-0103.html): +X points
towards the camera, +Y to the robot's left, +Z up. Its origin is the middle of the
wheel axle; the caster is at the rear.

| Frame | Parent | What it is |
|---|---|---|
| `base_footprint` | | Root of the tree, on the floor below `base_link` |
| `base_link` | `base_footprint` | Middle of the wheel axle, one wheel radius above the floor |
| `left_wheel_link`, `right_wheel_link` | `base_link` | Wheel centres; continuous joints about +Y |
| `laser_frame` | `base_link` | RPLIDAR A1 rotor centre |
| `imu_mount_link` | `base_link` | BNO055 breakout board |
| `imu_link` | `imu_mount_link` | BNO055 chip axes, the frame of the IMU messages |
| `camera_link` | `base_link` | Astra Pro at the depth lens; +X looks out of the lens |
| `camera_{rgb,depth,ir}_optical_frame` | `camera_link` | Optical frames (+Z forward, +X right, +Y down). Published only with `include_camera_optical_frames:=true` |
| `tof_front_{l,c,r}_link` | `base_link` | VL53L0X time-of-flight sensors at the front; +X along the beam |
| `tof_rear_{l,c,r}_link` | `base_link` | Time-of-flight sensors at the rear |
| `tof_left_link`, `tof_right_link` | `base_link` | Time-of-flight sensors on the sides |
| `hood_{side,front,rear}_link`, `skirt_{side,front,rear}_link`, `battery_drawer_link` | `base_link` | Body panels at the `base_link` origin, so RViz can hide them separately |

`l` and `left` are always on the +Y side. The CAD model labels the ToF stations as
seen from in front of the camera, so its left is the robot's right; the export maps
the labels to these names.

## Calibration

`urdf/roam_e.urdf.xacro` takes every fixed frame and the wheel geometry from the
file passed as `calibration_file`, by default `config/calibration.yaml`:

- `frames.<name>`: `parent`, `xyz` in metres, `rpy` in radians (roll, pitch, yaw
  about the fixed X, Y, Z axes) and a `note` saying where the values come from.
  Each link's meshes are in its own frame, so moving a frame moves its meshes.
- `physical_wheels`: `radius` sets the height of `base_link`, the wheel collision
  cylinder and the radial scale of the wheel meshes; `separation` places the wheel
  joints at ±separation/2; `width` sets the cylinder length and mesh width.
- `effective_odometry`: the floor-calibrated radius and separation used by the
  encoder odometry. The model does not read them.

The file starts from the CAD values in `config/calibration_cad_nominal.yaml`;
measured values replace them as they are made. At present the `camera_link`
orientation comes from a LiDAR wall check
([`tools/calibration/measure_camera_wall.py`](../../tools/calibration/measure_camera_wall.py))
and the `imu_link` orientation from axis checks on the robot. Regenerating the model
never overwrites this file.

To try other values without editing the installed file, copy it and pass the copy:

```bash
ros2 launch roam_e_description display.launch.py calibration_file:=/path/to/calibration.yaml
```

After editing the file in the source tree, run `build_description.py` to refresh
`urdf/roam_e.urdf`, then `validate_description.py` (see below).

## Launch

```bash
ros2 launch roam_e_description display.launch.py
```

| Argument | Default | Effect |
|---|---|---|
| `calibration_file` | `config/calibration.yaml` | Frames and wheel geometry |
| `include_camera_optical_frames` | `false` | Publish the CAD optical frames. Leave `false` when the camera driver publishes them |
| `publish_joints` | `true` | Publish zero wheel angles, for a static view without encoder feedback |
| `rviz` | `true` | Start RViz |
| `rviz_config` | `rviz/roam_e.rviz` | RViz configuration file |
| `use_sim_time` | `false` | Use the `/clock` topic |

## Regenerating from CAD

The meshes, the CAD-nominal frames and the generated files come from three scripts
in [`tools/description`](../../tools/description). Run them from the repository
root; the last two need the `xacro` and `PyYAML` Python packages (`pip install
xacro pyyaml` outside a ROS environment).

```bash
blender --background --python tools/description/export_blender_description.py
python3 tools/description/build_description.py
python3 tools/description/validate_description.py
```

1. The export runs inside Blender (on macOS the binary is
   `/Applications/Blender.app/Contents/MacOS/Blender`). It opens the `.blend`
   without saving it and writes `meshes/visual/` and `config/mesh_manifest.json`.
2. The build writes `urdf/visuals.xacro`, `config/calibration_cad_nominal.yaml`,
   `config/geometry.yaml` and `urdf/roam_e.urdf`. It leaves `calibration.yaml` as
   it is unless given `--reset-calibration`.
3. The validator checks the meshes against the manifest and the `.blend`, the TF
   tree with and without the optical frames, the sensor frame conventions, the
   wheel geometry, that the calibration file drives the model, and that
   `urdf/roam_e.urdf` is up to date. It writes `build/description/validation.json`.

The tests run with `colcon test --packages-select roam_e_description`, or without
colcon with `python3 -m pytest src/roam_e_description/test`.
