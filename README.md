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

The initial floor calibration is `FF,45,36,40,39`. These values are starting
points for the current motors, bridge, battery, and tires; repeat low-speed
tests after changing any of that hardware.

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
