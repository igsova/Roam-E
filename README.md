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
