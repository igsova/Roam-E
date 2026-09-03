# Roam-E

ROS 2 Jazzy software and Arduino Nano firmware for the Roam-E differential
drive robot.

## Drive base

The Nano owns the time-critical wheel velocity loops and the Raspberry Pi
converts ROS body velocity commands into left and right encoder targets. The
serial protocol uses normalized forward-positive wheel values:

- `VEL,<left_ticks_per_second>,<right_ticks_per_second>` selects closed-loop
  velocity control.
- `STOP` immediately disables both motor outputs.
- `PID,<kp_milli>,<ki_milli>,<kv_milli>,<static_pwm>` updates the controller
  gains while stopped.
- `FF,<left_forward>,<left_reverse>,<right_forward>,<right_reverse>` applies
  the independently measured minimum PWM for each wheel and direction.
- `INFO` reports protocol version, encoder scale, and safety limits.
- Telemetry is
  `T,ms,left_ticks,right_ticks,left_pwm,right_pwm,left_target,right_target,left_measured,right_measured`.

Both safety layers require continuously refreshed commands. ROS stops after
250 ms without a fresh `cmd_vel`; the Nano independently removes motor power
after 350 ms.

The floor-tested DRV8871 calibration is `FF,75,60,75,60`, with `Kv=0` and a
hard PWM ceiling of 100. Repeat the low-speed and maximum-speed tests after
changing the motors, bridge, battery, wheels, or robot mass.

The floor- and IMU-calibrated odometry geometry is a `0.033975 m` effective
wheel radius and `0.175150 m` effective wheel separation. These control values
intentionally remain separate from the physical CAD dimensions in the URDF.

Motor commands are disabled by default. The normal visualization launch is
therefore read-only:

```bash
ros2 launch roam_e sensors_viz.launch.py use_rviz:=false
```

Only enable output after the firmware has been installed, the diagnostics are
healthy, and the wheels are physically lifted clear of the floor:

```bash
ros2 launch roam_e sensors_viz.launch.py \
  use_rviz:=false \
  enable_motor_control:=true
```

The default command type is `geometry_msgs/msg/TwistStamped` on `/cmd_vel`.
Set `use_stamped_cmd_vel:=false` only for a source that publishes the older
unstamped `geometry_msgs/msg/Twist` form. Limits and initial controller gains
are exposed as launch arguments; tune them using lifted-wheel tests before
driving on the floor.

## Local odometry fusion

The normal sensor and drive launches start a `robot_localization` EKF. Raw
encoder integration remains available on `/wheel/odometry`; the fused local
estimate is published on `/odometry/filtered`. The EKF is the only publisher
of the `odom -> base_footprint` transform. Dynamic process-noise scaling keeps
the pose covariance from growing unnecessarily while the robot is stationary.

The planar filter uses forward and lateral wheel velocity plus wheel yaw rate,
and relative IMUPLUS yaw plus yaw rate. It intentionally excludes the
encoder-integrated pose and IMU acceleration so correlated wheel measurements
are not counted twice and uncharacterized acceleration bias does not enter the
estimate.

To diagnose the raw wheel estimate without the EKF, launch with:

```bash
ros2 launch roam_e sensors_viz.launch.py \
  use_rviz:=false \
  use_ekf:=false
```

In that fallback mode the encoder node resumes publishing the odometry TF.

## Keyboard and Foxglove driving

The dedicated drive launch uses the common `geometry_msgs/msg/Twist` form so
both Foxglove Teleop and `teleop_twist_keyboard` can publish on `/cmd_vel`.
It starts disarmed unless the motor gate is explicitly enabled:

```bash
ros2 launch roam_e drive.launch.py arm_motors:=true
```

The drive launch starts the sensors and Foxglove bridge, limits the robot to
0.20 m/s, 0.6 rad/s, and 700 wheel ticks/s, and retains the 250 ms ROS command
timeout plus the 350 ms Nano watchdog. The Nano independently caps all motor
output at PWM 100.

For keyboard control, open a second SSH terminal and run:

```bash
source /opt/ros/jazzy/setup.zsh
source ~/Roam-E/install/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Hold `i` for forward, `,` for reverse, `j`/`l` to turn, and press `k` (or any
unmapped key) for an immediate stop. The keyboard package's larger requested
speeds are clamped by the drive node. Releasing a key also causes the robot to
stop within 250 ms when key repeat ends.

In Foxglove, add a Teleop panel and configure:

- Topic: `/cmd_vel`
- Publish rate: `10 Hz`
- Stop on release: enabled
- Up/down: `linear.x` = `0.06` / `-0.06`
- Left/right: `angular.z` = `0.4` / `-0.4`

That is the Normal panel. Duplicate it for a Fast panel and change only:

- Up/down: `linear.x` = `0.18` / `-0.18`

The Fast value corresponds to the measured PWM-100 capability while leaving
enough margin for the encoder controller to keep the wheels synchronized.

Use only one teleoperation source at a time. Stop the launch with `Ctrl-C` to
send `STOP` and close the serial connection cleanly.

## Building a map

The mapping launch combines the complete sensor and drive stack, EKF odometry,
Foxglove bridge, and asynchronous SLAM Toolbox. Run every command below from
the workspace after sourcing ROS and the workspace overlay:

```bash
cd ~/Roam-E
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
```

### Start a new map

The launch starts with motor output disabled, which is useful for checking the
sensors, transforms, and SLAM before driving:

```bash
ros2 launch roam_e mapping.launch.py
```

In Foxglove, use `map` as the fixed frame and display `/map`, `/scan`, the
URDF, and transforms. After the static map and `map -> odom` transform appear,
restart the launch for a supervised mapping drive:

```bash
ros2 launch roam_e mapping.launch.py arm_motors:=true
```

Use the Normal Teleop panel (`0.06 m/s`, `0.4 rad/s`), drive slowly around the
outer boundary, observe corners from more than one direction, and finish near
the starting position so SLAM Toolbox can close the loop. Mapping mode clamps
the robot to `0.10 m/s`, `0.5 rad/s`, and 500 wheel ticks/s.

Starting a separate mapping process creates a separate coordinate system. A
room mapped on its own does not automatically attach to another room map. For
a single apartment map, load the saved apartment pose graph and drive
continuously through each doorway while some previously mapped geometry stays
visible to the LiDAR.

### Save a map

Keep the mapping launch running and open a second SSH terminal. Source ROS and
the workspace there, then save the lossless pose graph first:

```bash
cd ~/Roam-E
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
mkdir -p /home/robot/Roam-E/maps

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/robot/Roam-E/maps/MAP_NAME'}"
```

`result=0` means serialization succeeded. The filename is a base name without
an extension; SLAM Toolbox creates `MAP_NAME.posegraph` and `MAP_NAME.data`.
Next export the occupancy image used by Nav2:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/robot/Roam-E/maps/MAP_NAME \
  --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=20.0
```

The explicit timeout allows for SLAM Toolbox's map publication interval. Keep
the decimal in `20.0` so ROS interprets it as a floating-point parameter.
Verify the complete checkpoint before stopping the mapping launch:

```bash
ls -lh /home/robot/Roam-E/maps/MAP_NAME.*
```

A complete checkpoint contains `.posegraph`, `.data`, `.pgm`, and `.yaml`.
Only after all four files exist should the original mapping terminal be
stopped with `Ctrl-C`. If the robot moved after pose-graph serialization,
serialize it again before stopping so both saved forms describe the same run.

### Continue a saved map

The most reliable restart is to place the robot at the original starting
position of that map, facing approximately the same direction. Start the
mapping stack but do not drive yet:

```bash
ros2 launch roam_e mapping.launch.py arm_motors:=true
```

From a second sourced terminal, load the checkpoint using its base filename:

```bash
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/robot/Roam-E/maps/MAP_NAME', match_type: 1, initial_pose: {x: 0.0, y: 0.0, theta: 0.0}}"
```

`match_type: 1` matches the current scan against the first pose-graph node. If
the robot must start elsewhere, use `match_type: 2` and supply a known
approximate pose in the map frame, in metres and radians:

```bash
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '/home/robot/Roam-E/maps/MAP_NAME', match_type: 2, initial_pose: {x: X_METRES, y: Y_METRES, theta: YAW_RADIANS}}"
```

In Foxglove, set the fixed frame to `map` and display `/map`, `/scan`, the
URDF, and transforms. Do not drive until the live scan aligns with known walls
in the saved occupancy map. If it does not align, stop and retry from a better
known pose rather than adding misregistered scans to the pose graph.

To check the transform from a terminal:

```bash
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint
```

## Localizing in a saved map

Use AMCL when the robot may have been carried to an unknown position while it
was powered off. Unlike SLAM pose-graph continuation, this mode loads the
static occupancy map and estimates the robot pose without adding new map
cells.

Start the localization stack with the required map. Motor output remains
disabled unless it is explicitly armed:

```bash
cd ~/Roam-E
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh

ros2 launch roam_e localization.launch.py arm_motors:=true
```

The launch defaults to `maps/office_02.yaml`, the first map captured after
correcting the RPLidar native clockwise/rear-zero angles to ROS
counter-clockwise/+X angles. Historical maps captured before that correction
are retained as development records but must not be used or resumed for
localization.

Leave the Teleop controls untouched until both lifecycle nodes report
`active`:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

From a second sourced terminal, request global localization. AMCL will
distribute pose hypotheses throughout all free cells in the loaded map, so no
initial coordinates are required:

```bash
ros2 service call /reinitialize_global_localization \
  std_srvs/srv/Empty "{}"
```

In Foxglove, use `map` as the fixed frame and display `/map`, `/scan`,
`/particle_cloud`, `/amcl_pose`, the URDF, and transforms. Use the Teleop panel
to rotate slowly in place, initially around `0.2 rad/s`, so AMCL can compare
several LiDAR views with the map. Do not drive forward until:

- The particle cloud has collapsed to one compact cluster.
- The live `/scan` points align with the saved walls.
- `/amcl_pose` is stable and the robot model is in the correct location.

If several clusters remain or the scan is misaligned, stop, call
`/reinitialize_global_localization` again, and rotate through views containing
distinctive geometry. A small or symmetric room can be ambiguous, and moved
furniture or people visible to the LiDAR can slow convergence.

Once localized, AMCL publishes `map -> odom` while the EKF continues to
publish `odom -> base_footprint`. Carrying the robot again invalidates that
estimate; request global localization again after any such move. Stop the
launch with `Ctrl-C` when finished.

## Autonomous office navigation

The navigation launch reuses the `office_02` map, AMCL, EKF odometry, LiDAR,
robot description, and Foxglove bridge. It adds a lightweight Nav2 stack for
the Raspberry Pi: NavFn planning, Regulated Pure Pursuit path following, a
velocity smoother, and Collision Monitor. It starts with motor output
disabled:

```bash
cd ~/Roam-E
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch roam_e navigation.launch.py
```

The costmaps use the CAD-derived chassis-and-wheel polygon rather than a
circle. Its physical extent is approximately `141 mm` front-to-back and
`203 mm` across the wheels, with another `10 mm` of Nav2 padding. Collision
Monitor independently applies a slowdown zone around that footprint and a
smaller stop zone. Its command path is:

```text
Nav2 controller or recovery behavior
  -> /cmd_vel_nav
  -> velocity smoother
  -> /cmd_vel_smoothed
  -> Collision Monitor
  -> /cmd_vel
  -> guarded motor controller
```

First confirm the map and AMCL lifecycle nodes are active:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

Then localize the robot as described above. For an approximate known pose,
use Foxglove's `2D pose estimate` publisher on `/initialpose`. For an unknown
pose, call global localization and use a short, supervised rotation to let
AMCL converge. The global costmap intentionally waits for AMCL to create the
`map -> odom` transform.

The navigation lifecycle manager also waits for the complete
`map -> base_footprint` transform. As soon as AMCL accepts the initial pose,
the goal bridge requests Nav2 startup automatically. This avoids leaving the
planner and navigator inactive when the Raspberry Pi takes longer than usual
to initialize. Use `autostart:=false` only when lifecycle transitions will be
managed manually.

Once the scan aligns with the map, confirm the navigation lifecycle nodes are
active before sending a goal:

```bash
for node in controller_server planner_server behavior_server \
  bt_navigator velocity_smoother collision_monitor
do
  ros2 lifecycle get "/$node"
done
```

In Foxglove, use `map` as the fixed frame and display these topics:

- `/map` and `/scan`
- `/global_costmap/costmap` and `/local_costmap/costmap`
- `/global_costmap/published_footprint` and
  `/local_costmap/published_footprint`
- `/plan` and `/local_plan`
- `/collision_monitor/stop_zone`, `/collision_monitor/slowdown_zone`, and
  `/collision_monitor_state`

For click-to-go control, add Foxglove's `2D pose` publisher, set its topic to
`/goal_pose`, and click-drag on free space to set both position and heading.
The Roam-E goal bridge forwards that `PoseStamped` message to Nav2's
`/navigate_to_pose` action. With motors disarmed, this is a safe way to check
that the global path, local costmap, and collision zones appear. The
controller will report lack of progress because the robot cannot move; stop
the launch after the visualization check.

Do not use the Foxglove Teleop panel on `/cmd_vel` while autonomous navigation
is running. That is the final post-monitor topic, so a second publisher there
would compete with the safety chain. Stop the navigation launch before using
the separate manual `drive.launch.py` workflow.

Only after the footprint, scans, costmaps, plan, and localization all look
correct should the launch be restarted with motor output armed:

```bash
ros2 launch roam_e navigation.launch.py arm_motors:=true
```

Keep a hand at the robot's physical power cutoff and begin with a clear,
straight `0.5 m` goal at the configured maximum of `0.08 m/s`. Then test, in
order: a requested final heading, an obstacle in the slowdown zone, an
obstacle in the stop zone, replanning around an obstacle, a blocked-goal
failure, and global localization from several carried starting poses.
Collision Monitor is an additional software layer, not a certified emergency
stop and not a substitute for a physical power cutoff around people or pets.

## Raspberry Pi 5 migration

The supported target is a Raspberry Pi 5 with 8 GB of RAM running the 64-bit
Ubuntu 24.04 Server image. Use active cooling and a stable 5 V / 5 A supply on
the robot. Do not move the old Pi 3 filesystem or its generated `build/` and
`install/` directories to the Pi 5; start with a clean OS and rebuild there.

Before replacing the Pi, create a migration backup on the working robot:

```bash
cd ~/Roam-E
scripts/setup/backup_robot_state.sh
```

The archive is written outside the repository under `~/roam-e-backups`. It
contains maps, calibration, firmware, the BNO055 systemd unit, Raspberry Pi
boot settings, stable device names, package inventories, and checksums. It
deliberately excludes Wi-Fi credentials, SSH private keys, build products,
and ROS logs. Copy both the `.tar.gz` file and its `.sha256` file to another
computer before removing the old storage.

On a fresh Pi 5, create the same `robot` account, configure networking and
SSH, then clone and bootstrap the repository:

```bash
sudo apt-get update
sudo apt-get install -y git
git clone https://github.com/igsova/Roam-E.git ~/Roam-E
cd ~/Roam-E
scripts/setup/bootstrap_pi5.sh --upgrade-os
sudo reboot
```

The bootstrap is safe to rerun. It verifies Ubuntu Noble on 64-bit ARM,
installs the official ROS 2 Jazzy apt source, ROS Base and all dependencies
declared by the workspace, installs the validated Python hardware drivers,
creates the small calibration virtual environment, enables I2C bus 3 on GPIO
17/27, adds the current user to the hardware groups, builds the workspace,
and installs the BNO055 calibration restore service. It never launches the
robot and never arms or commands the motors.

After reconnecting the Arduino Nano, RPLidar, and BNO055, run the non-moving
validation:

```bash
cd ~/Roam-E
scripts/setup/validate_hardware.sh
```

The persistent USB names should still be:

- Arduino Nano: `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- RPLidar: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- BNO055: `/dev/i2c-3`, address `0x29`

If either USB identifier changes, update the defaults in
`sensors_viz.launch.py` rather than switching to unstable `/dev/ttyUSB*`
names. Once the static validation passes, start the ordinary stack with motor
output disabled and check live messages from another terminal:

```bash
# Terminal 1
cd ~/Roam-E
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch roam_e sensors_viz.launch.py use_rviz:=false

# Terminal 2
cd ~/Roam-E
scripts/setup/validate_hardware.sh --runtime
```

Only proceed to supervised motor and navigation tests after the static and
runtime checks pass. The migration backup is a recovery copy; the committed
`office_02` map and BNO055 calibration also arrive with the Git clone.
