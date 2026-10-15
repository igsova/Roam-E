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
