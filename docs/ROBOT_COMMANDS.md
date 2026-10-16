# Roam-E quick commands (R19, October 2026)

Run these **on the Pi** (SSH in as `robot`). Start every new terminal with:

```bash
cd ~/Roam-E && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

Only run **one** of the launches in sections 1–3 at a time. Each one starts its
own Foxglove bridge and owns the motor controller. Stop one (Ctrl+C) before
starting another. The cameras (section 4) can run alongside any of them.

## 1. Look only — Foxglove, sensors, motors disabled

```bash
ros2 launch roam_e drive.launch.py arm_motors:=false
```

The robot cannot move. You get the model, lidar, IMU and odometry in Foxglove.

## 2. Drive by hand — Foxglove Teleop, motors enabled

```bash
ros2 launch roam_e drive.launch.py arm_motors:=true
```

In Foxglove, use the Teleop panel on `/cmd_vel` (Twist). Robot-side limits
are 0.43 m/s and 0.6 rad/s.

## 3. Navigation (Nav2) on the office map — motors enabled

```bash
ros2 launch roam_e navigation.launch.py arm_motors:=true auto_localize:=true \
  map:=/home/robot/Roam-E/maps/office_02.yaml
```

- `auto_localize:=true` **moves the robot** at startup (20 cm forward, a 90°
  left turn, 20 cm forward) to find itself on the map. Leave room. Use
  `auto_localize:=false` to set the position yourself with Foxglove's
  **2D Pose Estimate** on `/initialpose`.
- Send targets with Foxglove's **2D Goal Pose** on `/goal_pose` (fixed frame `map`).
- Do **not** use Teleop while Nav2 runs: it bypasses the collision monitor.
- `arm_motors:=false` plans without moving (goals then time out).

## 4. Cameras (optional, alongside 1–3)

```bash
ros2 launch roam_e astra_camera.launch.py use_foxglove:=false
```

Image panels: `/camera/color/image_raw/compressed` and
`/camera/depth_preview/compressed`. The cameras add load on the Pi's 5 V supply.
The Geekworm jumper (+0.2 V) fixed the freezes this caused before.

With the cameras and navigation (section 3) both running, the depth camera
marks low obstacles (5–35 cm high, 0.45–2.5 m away) in Nav2's local costmap.
In Foxglove's 3D panel add `/local_costmap/costmap`,
`/local_costmap/voxel_marked_cloud` (the 3D obstacle voxels) and
`/camera/obstacle_points` (raw depth points). The local costmap covers only
2 × 2 m around the robot.

## 5. 3D mapping and navigation with RTAB-Map

Start the cameras with faster colour decoding first (RTAB-Map needs it;
10 gives about 8 colour frames per second):

```bash
ros2 launch roam_e astra_camera.launch.py use_foxglove:=false color_decode_fps:=10.0
```

The RTAB-Map launch shifts the depth image onto the colour image
(`depth_align_node`, 7 px left and 1 px up, measured 16 Oct 2026 with
`tools/measure_r19_depth_color_offset.py`).

Map by driving with Teleop (motors enabled, 0.10 m/s cap). `new_map:=true`
deletes the old database; leave it out to extend the existing map.

```bash
ros2 launch roam_e rtabmap_mapping.launch.py arm_motors:=true new_map:=true
```

Foxglove (fixed frame `map`): `/rtabmap/cloud_map`, `/rtabmap/map`,
`/rtabmap/mapPath`. The map is saved continuously to
`~/Roam-E/maps/rtabmap/office_rtabmap.db`. Drive slowly and finish where
you started so loops close.

Navigate on that map instead of office_02 (no automatic localization drive):

```bash
ros2 launch roam_e navigation.launch.py localization:=rtabmap arm_motors:=true
```

RTAB-Map cannot recognise places from this camera by itself, so the robot
finds its start position with the XFeat visual relocalizer (motors not needed,
about 10 s per attempt). It runs when RTAB-Map reports it is lost, or on
demand with `ros2 service call /visual_relocalizer/relocalize std_srvs/srv/Trigger`.
It needs the `~/venvs/lightglue` environment (PyTorch CPU, LightGlue, XFeat
in `~/venvs/src/accelerated_features`) and a keyframe cache built from the
map after every new map:

```bash
cd ~/Roam-E/tools
~/venvs/lightglue/bin/python r19_visual_relocalizer.py --ros-args \
  -p keyframes:=/home/robot/Roam-E/maps/rtabmap/office_rtabmap_xfeat.npz
# after a new map (nothing running on the database):
rtabmap-export --images_id --poses_camera --poses_format 10 --output_dir /tmp/kf map_copy.db
~/venvs/lightglue/bin/python r19_visual_keyframes.py /tmp/kf/map_copy keyframes.npz --features xfeat
```

Keep `period` at 0 while the lidar runs: the lidar tracks within about 5 cm,
the camera fix only within 15–30 cm, and a periodic camera correction above
15 cm cancels the active goal.

Dense export of the map (motors off, on a copy of the database):

```bash
rtabmap-export --cloud --decimation 2 --max_range 3.0 --voxel 0.01 office_copy.db
rtabmap-export --mesh --texture --texture_size 4096 --decimation 2 --max_range 3.0 office_copy.db
python3 ~/Roam-E/tools/publish_ply_cloud.py office_copy_cloud.ply --voxel 0.02
```

The last command shows the dense cloud in Foxglove on `/rtabmap/dense_cloud`.

## Stop / disable motors

- **Ctrl+C** in the launch's terminal stops it. The Nano's watchdog stops the
  wheels within 0.35 s when commands stop.
- Cancelling a Nav2 goal is not an electrical stop; use the motor power switch
  for that.
- If a launch was started in the background, stop all ROS launches with:

```bash
pkill -INT -f "ros2 launch roam_e"
```

## Foxglove on the Mac

- Same network: open Foxglove and connect to `ws://robot-pi5.local:8765`.
- Through the SSH tunnel (your usual one on port 2222), run this in a Mac
  terminal and keep it open, then connect to `ws://localhost:18765`:

```bash
ssh -N -p 2222 -L 127.0.0.1:18765:127.0.0.1:8765 robot@127.0.0.1
```

(Port 18765 is used because the Mac's 8765 is taken by another app.)

## Health checks

```bash
vcgencmd measure_temp; vcgencmd get_throttled; vcgencmd pmic_read_adc EXT5V_V
```

`throttled=0x0` is good. EXT5V should stay above about 4.9 V with everything
running.
