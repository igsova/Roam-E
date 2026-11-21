<p align="center">
  <img src="docs/images/roam_e.jpg" alt="Render of the Roam-E robot" width="760">
</p>

# Roam-E

[![CI](https://github.com/igsova/Roam-E/actions/workflows/ci.yml/badge.svg)](https://github.com/igsova/Roam-E/actions/workflows/ci.yml)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue)](LICENSE)

Roam-E is an indoor mobile robot built from scratch: a 3D-printed
differential-drive base with a 2D lidar, an RGB-D camera and an IMU, running
ROS 2 Jazzy on a Raspberry Pi 5. It maps rooms, localizes itself on a saved
map and drives to goals on its own while avoiding obstacles.

This repository holds the whole robot: the ROS 2 software, the Arduino motor
firmware, the robot description (URDF), and the Blender model and STL files of
the printed body.

**Highlights**

- Designed and built end to end: CAD model, 3D-printed body, electronics,
  wheel-control firmware and the ROS 2 navigation stack.
- Autonomous navigation with Nav2 on 2D lidar maps (AMCL) and on 3D RTAB-Map
  maps, with depth-camera obstacles below the lidar plane.
- Learned-feature visual relocalization (XFeat + LighterGlue + PnP) that runs
  on the Pi 5 CPU.
- Measured calibration throughout: camera intrinsics, depth-to-colour
  registration, camera mount angle against the lidar, IMU mounting and wheel
  odometry.
- Over 200 automated test cases in CI, including the firmware compiled against a
  mocked Arduino API and a validator for the CAD-generated robot model.

## What it does

- **Wheel control.** An Arduino Nano runs a closed-loop velocity controller
  for each wheel (PI with per-wheel feed-forward and a breakaway assist for
  starting from standstill), up to 0.43 m/s. Motors stop if commands stop
  arriving: after 250 ms on the ROS side and 350 ms in the firmware.
- **Odometry.** Wheel-encoder odometry and the BNO055 gyro are fused with an
  EKF (`robot_localization`).
- **Mapping.** 2D lidar SLAM with `slam_toolbox`, and 3D mapping with RTAB-Map
  using the RGB-D camera, the lidar and the wheel odometry.
- **Localization.** AMCL on saved 2D maps, with an optional automatic
  global-localization maneuver, or RTAB-Map localization on its 3D map. When
  RTAB-Map loses track, a learned-feature relocalizer (XFeat + LighterGlue +
  PnP) matches the camera image against keyframes of the map and resets the
  pose.
- **Navigation.** Nav2 with Regulated Pure Pursuit, a velocity smoother and a
  collision monitor. The depth camera adds obstacles below the lidar plane to
  the local costmap.
- **Teleoperation and visualization.** Foxglove for driving, sending goals and
  viewing the robot; keyboard teleop over SSH.

## Hardware

<p align="center">
  <img src="docs/images/roam_e_tour.webp" alt="See-through renders of Roam-E cycling through its sensors, actuators, compute and power components" width="760">
</p>

| | Component | Role |
|---|---|---|
| **Sensors** | Slamtec RPLIDAR A1 | 360° 2D lidar |
| | Orbbec Astra Pro | RGB-D camera, facing forward |
| | Bosch BNO055 | IMU, used in IMU-only fusion mode (no magnetometer) |
| | Quadrature encoders on both motors | Wheel odometry |
| | 8 × VL53L0X | Time-of-flight sensors around the base; mounted, not yet used by the software |
| **Actuators** | 2 × 12 V geared DC motors | Differential drive, with a passive caster at the rear |
| | 2 × DRV8871 | H-bridge motor drivers |
| **Compute** | Raspberry Pi 5 (8 GB) | Main computer, Ubuntu 24.04 + ROS 2 Jazzy |
| | Arduino Nano | Wheel velocity control and encoder counting, USB serial to the Pi |
| | TCA9548A | I²C multiplexer for the ToF sensors |
| **Power** | 3S LiPo, 11.1 V, 3300 mAh | Battery, in a slide-out drawer |
| | Geekworm Pi5-5V5APD | 5 V / 5 A supply for the Pi |
| | 2 × rocker switches | Battery and wheel power |

The body is about 240 × 215 × 235 mm and made of 47 different printed parts,
mostly PETG, printed on a Creality K1C. The skirt panels slide off
horizontally for service and the battery sits in a drawer at the rear. The Blender assembly, all STL files and the parts list are in
[`mechanical/`](mechanical/).

## Software

```mermaid
flowchart LR
    NANO["Arduino Nano<br/>wheel PI control"] <-->|"serial: VEL / telemetry"| ENC["encoder_odometry_node"]
    IMU["BNO055"] --> IMUN["bno055_imu_node"]
    LIDAR["RPLIDAR A1"] --> LIDN["rplidar_a1_node"]
    CAM["Astra Pro"] --> CAMN["openni2_camera<br/>depth nodes"]
    ENC -->|"wheel odometry"| EKF["robot_localization<br/>EKF"]
    IMUN -->|"IMU"| EKF
    EKF -->|"odom → base_footprint"| LOC["slam_toolbox / AMCL /<br/>RTAB-Map"]
    LIDN -->|"scan"| LOC
    CAMN -->|"RGB-D"| LOC
    LOC -->|"map → odom"| NAV["Nav2"]
    LIDN -->|"scan"| NAV
    CAMN -->|"depth cloud"| NAV
    NAV -->|"cmd_vel"| ENC
```

The Pi turns body velocity commands into wheel targets in ticks per second and
sends them to the Nano, which closes the loop on the encoders and streams
telemetry back ([serial protocol](firmware/README.md#serial-protocol)).
Everything above the Nano is ROS 2.

| Launch file | Starts |
|---|---|
| `sensors_viz.launch.py` | Sensors, wheel odometry, EKF and robot model. Motors stay disarmed |
| `drive.launch.py` | The above plus the Foxglove bridge, with the motors armable for teleop |
| `mapping.launch.py` | 2D SLAM with `slam_toolbox` |
| `localization.launch.py` | AMCL on a saved map |
| `navigation.launch.py` | Nav2 on a saved map, localized with AMCL or RTAB-Map |
| `rtabmap_mapping.launch.py` | RTAB-Map 3D mapping, or localization on a saved database |
| `astra_camera.launch.py` | RGB-D camera and the depth obstacle cloud |

### Engineering notes

- **Layered motion safety.** Velocity commands pass through Nav2's velocity
  smoother and collision monitor, are clamped by the ROS bridge, and expire
  after 250 ms there and after 350 ms in the firmware. Motors are disarmed
  unless a launch argument arms them.
- **Testable cores.** Kinematics, odometry, lidar binning, localization
  convergence and IMU health checks are plain Python modules
  (`motor_control.py`, `wheel_odometry.py`, `lidar_scan.py`, …) with unit tests;
  the ROS nodes are thin wrappers around them. The firmware is tested the same
  way, against a mocked Arduino API with a controllable clock.
- **Robot model from CAD.** The URDF meshes and nominal sensor frames are
  exported from the Blender assembly, and measured calibration (camera yaw, IMU
  mounting, wheel geometry) overrides them in one YAML file. A validator checks
  the TF tree, mesh integrity and that the generated URDF is up to date.
- **Calibrated odometry.** The effective wheel radius and separation were
  fitted against the IMU and floor runs and kept separate from the CAD
  dimensions. Per-wheel, per-direction feed-forward in the firmware compensates
  for the two gearmotors' different starting duty.
- **Camera on demand.** The camera relocalizer is only used when RTAB-Map is
  lost or on request: the lidar tracks within about 5 cm and the camera fix
  only within 15–30 cm, so periodic camera corrections would add error and
  interrupt active goals.

### Repository layout

```text
src/roam_e/               ROS 2 package: nodes, launch files, EKF / SLAM / AMCL / Nav2 config
src/roam_e_description/   URDF / xacro robot model, TF frames and RViz setup
firmware/                 Arduino Nano wheel controller, AVR build script and host-side tests
mechanical/               Blender assembly, STL print files and parts list
maps/                     Saved slam_toolbox map
scripts/setup/            Raspberry Pi 5 bootstrap, backup and hardware validation
scripts/bno055/           IMU calibration and checks
scripts/camera/           RGB and depth camera calibration tools
tools/                    Visual relocalizer, camera checks, robot-model export from CAD
config/                   IMU calibration and systemd unit
docs/                     Usage guide, command sheet, camera and research notes, images
```

## Getting started

On a Raspberry Pi 5 with Ubuntu 24.04 Server (64-bit):

```bash
git clone https://github.com/igsova/Roam-E.git ~/Roam-E
cd ~/Roam-E
scripts/setup/bootstrap_pi5.sh --upgrade-os
sudo reboot
```

The bootstrap installs ROS 2 Jazzy and the workspace dependencies, enables the
IMU's I²C bus and builds the workspace. It never moves the motors. After the
reboot, check the hardware without moving, then start the sensors:

```bash
cd ~/Roam-E
scripts/setup/validate_hardware.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch roam_e sensors_viz.launch.py use_rviz:=false
```

Driving, mapping, localization and navigation are covered step by step in
[`docs/usage.md`](docs/usage.md).

## Tests

CI builds both ROS packages in a ROS 2 Jazzy container, runs every test suite
and the robot-model validator, runs the firmware tests and builds the firmware
for the Nano. None of the tests need the robot. With ROS 2 Jazzy installed:

```bash
colcon build
source install/setup.bash
python3 -m pytest
python3 -m unittest firmware/tests/test_firmware.py
```

## Documentation

- [Usage guide](docs/usage.md): driving, mapping, localization, navigation and
  Pi setup.
- [Quick commands](docs/quick_commands.md): one-page cheat sheet for driving,
  Nav2, cameras, RTAB-Map 3D mapping and health checks.
- [Wheel controller firmware](firmware/README.md): control loop, wiring,
  serial protocol, build and tests.
- [Robot description](src/roam_e_description/README.md): TF frames,
  calibration overrides and regenerating the model from CAD.
- Camera: [setup on the Pi](docs/camera/setup.md),
  [RGB calibration](docs/camera/rgb_calibration.md),
  [depth-to-colour registration](docs/camera/depth_registration.md) and the
  [calibration tools](scripts/camera/README.md).
- [IMU calibration](scripts/bno055/README.md) and [maps](maps/README.md).
- Visual SLAM research notes: [what runs on a Pi 5 CPU](docs/research/cpu_visual_slam.md)
  and a [survey of current methods](docs/research/visual_slam_survey.md).
- [Mechanical design](mechanical/README.md): parts, printing and assembly.
