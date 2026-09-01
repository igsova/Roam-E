# Roam-E ROS 2 description

This package contains the RViz/URDF description generated from
`Roam_E_ROS.FCStd`.

The robot follows the ROS coordinate convention:

- `+X` forward, toward the caster
- `+Y` left
- `+Z` up
- `base_link` at the midpoint of the drive-wheel axes
- `base_footprint` on the floor directly below `base_link`

The detailed CAD geometry is kept as optimized meshes for the rigid base,
wheels, LiDAR, and IMU board. The wheel separation and radius come from
physical measurements rather than the CAD bounding boxes.

## Build and display

```bash
cd ~/Roam-E
colcon build --symlink-install --packages-select roam_e_description
source install/setup.bash
ros2 launch roam_e_description display.launch.py
```

Set `rviz:=false` when only the TF tree is needed. Once the encoder driver
publishes `/joint_states`, set `publish_joints:=false`.

For live hardware, configure the RPLIDAR message `frame_id` as `laser_frame`
and the IMU message `frame_id` as `imu_link`.

## Regenerate meshes on macOS

From the workspace root:

```bash
/Applications/FreeCAD.app/Contents/Resources/bin/python \
  tools/export_freecad_meshes.py \
  /Users/igorvaz/Downloads/Roam_E_ROS.FCStd \
  src/roam_e_description
```

The exporter converts FreeCAD millimetres and axes into ROS-oriented local
mesh coordinates. URDF applies the final millimetre-to-metre scale.

