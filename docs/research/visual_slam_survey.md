# Visual SLAM for Roam-E: survey, October 2026

Goal: navigate in 3D using the RGB camera, the depth camera, or both. This
note surveys the methods available in October 2026, checks them against
Roam-E's hardware and proposes a staged path. It is desk research. The only
robot numbers in it are measurements recorded earlier (RTAB-Map RGB-D
odometry in September, a depth-versus-lidar check in October); none of the
surveyed methods was run on the robot for this note. The follow-up,
[cpu_visual_slam.md](cpu_visual_slam.md), does test methods on the robot.

## Starting point: constraints from the hardware

| Item | Roam-E | Why it matters |
|---|---|---|
| Compute | Raspberry Pi 5, 8 GB, **no CUDA GPU** | Rules out GPU-only SLAM onboard; the CPU is already loaded by Nav2 and the cameras (load average 5–14 observed) |
| Camera | Orbbec Astra Pro: OpenNI depth 640×480 ~30 Hz; RGB as a separate UVC device (~25 Hz, rolling shutter); no hardware RGB–depth sync | RGB-D works, but frames are paired in software; older OpenNI device |
| Other sensors | RPLIDAR A1 (2D), BNO055 IMU (low cost, relative yaw), calibrated wheel odometry | Strong odometry and 2D localization already; IMU too noisy for good VIO |
| Measured on the Pi | RTAB-Map `rgbd_odometry` on the Astra: ~1.4 Hz at a 2 Hz cap, 96 ms median compute per update, no lost frames in a 20 s test ([on-board registration test](../camera/onboard_registration_test.md)) | Visual odometry runs on the Pi, but slowly |
| Calibration | Registered depth within ~1 cm of the lidar up to 1.5 m; camera mount corrected by 2.1° pitch and 2.8° yaw | Depth is usable for mapping and obstacles |

## The landscape

### 1. Classical feature-based SLAM (CPU)

- **ORB-SLAM3** (mono, stereo, RGB-D, inertial): still the accuracy reference
  for sparse SLAM, with multi-map relocalization. No official ROS 2 package;
  Jazzy needs a patched community wrapper. Produces a sparse map, not a
  navigation map.
- **stella_vslam** (ORB-SLAM family, mono/stereo/RGB-D): maintained docs and a
  ROS 2 package; Jazzy support not stated.
- Verdict: good for localization research, poor for navigation (no dense or
  occupancy output), and Jazzy integration is custom work.

### 2. RTAB-Map: graph SLAM built for robots (CPU)

- Mature and actively maintained; `rtabmap_ros` 0.23.x is released for
  **Jazzy** (package index, updated June 2026).
- Takes RGB-D (or stereo, or mono with external odometry) **plus wheel
  odometry and lidar**, does appearance-based loop closure, and outputs a 3D
  point cloud or OctoMap **and a 2D occupancy grid that Nav2 can use**. It can
  also localize on a saved map.
- On a Pi 5 it is slow (the 1.4 Hz visual odometry above; one vendor runs it
  off the Pi entirely). Using wheel/EKF odometry instead of visual odometry
  removes the heaviest part, which is how small robots usually run it.
- Verdict: **the practical, ROS-2-native choice for Roam-E today.**

### 3. Visual-inertial odometry (OpenVINS, VINS-Fusion, Basalt)

- A 2025 ROS 2 comparison found OpenVINS best on several metrics, RTAB-Map and
  VINS-Fusion best on others. Basalt has a maintained ROS wrapper (MOLA), but
  Jazzy is not listed for it.
- All need a decent, time-synchronised IMU and ideally a global-shutter camera.
- Verdict: **not a good fit** with the BNO055 and the rolling-shutter Astra RGB.

### 4. GPU-accelerated geometric SLAM: NVIDIA cuVSLAM and nvblox

- cuVSLAM (Isaac ROS Visual SLAM) is the strongest real-time embedded option.
  It runs 1–32 cameras with optional IMU and depth; Isaac ROS 4.1 added RGB-D
  support. **nvblox** builds a dense 3D map from depth and publishes a costmap
  for Nav2.
- Requires an NVIDIA Jetson (Orin Nano Super and up) or an RTX PC. NVIDIA
  notes that Orin Nano Super performance drops when the GPU is shared.
- Verdict: the **"real 3D navigation" path if a Jetson is added.**

### 5. Learned dense SLAM (research frontier)

- **DROID-SLAM**, **MASt3R-SLAM** (CVPR 2025, ~15 FPS on a high-end GPU),
  **VGGT-SLAM / VGGT-SLAM++** (2025–2026): RGB-only, even uncalibrated, dense
  SLAM. Recent comparisons rank MASt3R-SLAM most consistent with DROID-SLAM
  close behind; VGGT-SLAM++ reports the best indoor (7-Scenes) numbers.
- **Gaussian-splatting SLAM** (SplaTAM, Splat-SLAM, MonoEM-GS 2026) gives
  photorealistic dense maps. A 2026 embedded-SLAM survey benchmarks these on a
  Jetson AGX Orin; memory grows with scene size.
- All need a desktop-class or Jetson-AGX-class GPU and have no standard Nav2
  integration. Verdict: **promising, offboard GPU only, research grade for
  navigation.**

### 6. Monocular depth for navigation (RGB camera to depth)

- Nav2's official tutorial runs **Depth Anything 3** (TensorRT) on a plain USB
  camera, converts depth to a point cloud and feeds a voxel costmap layer.
- Learned depth is relative and degrades with range: a Depth Anything V2 study
  found 89 % of errors within ±0.5 m under 2 m. It needs an NVIDIA GPU; no
  Hailo or Pi AI HAT port was found.
- Verdict: an RGB-only path, but for **obstacles**, not mapping; needs a GPU.

### 7. 3D obstacles in Nav2 from a depth camera (on the Pi today)

- The **Spatio-Temporal Voxel Layer (STVL)** is available for Jazzy and
  supports a depth-camera frustum. Nav2's own VoxelLayer also accepts point
  clouds.
- This gives Nav2 3D awareness (tables, overhangs, low objects the 2D lidar
  misses) without full visual SLAM.

## Recommended path

1. **Depth camera to Nav2 obstacles (Pi only, small).** Add the Astra depth as
   a voxel or STVL costmap source, using the calibrated mount, decimated to a
   few Hz. Immediate 3D obstacle avoidance at low risk; also tests CPU and
   power headroom.
2. **RTAB-Map RGB-D SLAM with wheel odometry and lidar (Pi only, medium).** Map
   the office in 3D, project a 2D grid, compare it with the existing
   slam_toolbox map and AMCL, then try RTAB-Map localization for Nav2 (only one
   `map→odom` owner). Run at 1–2 Hz and measure CPU, temperature and the 5 V
   rail while mapping.
3. **Decide on hardware for real-time 3D or RGB-only methods.** If steps 1–2
   hit the Pi's limits, add an NVIDIA Jetson Orin Nano Super for cuVSLAM +
   nvblox (or Depth Anything 3), keeping the Pi for drivers and Nav2. A modern
   RGB-D camera (global shutter, hardware sync, built-in IMU) would help any
   visual method; the Astra Pro is not supported by Orbbec's current ROS 2
   SDK v2.
4. **Learned RGB-only SLAM (MASt3R, VGGT, Gaussian splatting)** as an offboard
   experiment on a GPU PC with recorded bags, not for onboard navigation yet.

## Open questions at the time

- No published Raspberry Pi 5 RGB-D SLAM benchmarks were found; measurements
  on the robot are the best guide.
- Whether STVL's Jazzy binary and the Astra depth rate fit the Pi's CPU budget
  alongside Nav2 and the cameras.

## Outcome

Steps 1 and 2 were built on the Pi; no extra hardware was added.

- **Depth obstacles:** the depth camera marks low obstacles (5–35 cm high,
  0.45–2.5 m away) in Nav2's local costmap through a decimated point cloud and
  Nav2's voxel layer.
- **RTAB-Map:** 3D mapping with registered depth, lidar ICP and EKF odometry,
  and Nav2 navigation on RTAB-Map localization. Depth is aligned to the colour
  image first ([depth_registration.md](../camera/depth_registration.md)).
- **Camera relocalization:** RTAB-Map's own ORB/SIFT place recognition never
  accepted a match from this camera, so the follow-up note
  ([cpu_visual_slam.md](cpu_visual_slam.md)) led to an XFeat + LighterGlue +
  PnP relocalizer in [`tools/rtabmap/`](../../tools/rtabmap/) (see
  [`tools/README.md`](../../tools/README.md)). It sets RTAB-Map's initial pose
  when RTAB-Map reports it is lost, or on demand. Periodic camera correction
  is off (`period` 0): with the lidar, RTAB-Map tracks within about 5 cm,
  while a camera fix is good to only 15–30 cm.

Commands are in [`docs/quick_commands.md`](../quick_commands.md), sections 4
and 5.

## Sources

- RTAB-Map ROS index: https://index.ros.org/p/rtabmap_ros ; RTAB-Map paper: https://arxiv.org/pdf/2403.06341
- MASt3R-SLAM (CVPR 2025 supplement): https://openaccess.thecvf.com/content/CVPR2025/supplemental/Murai_MASt3R-SLAM_Real-Time_Dense_CVPR_2025_supplemental.pdf
- VGGT-SLAM: https://arxiv.org/pdf/2505.12549 ; VGGT-SLAM++: https://arxiv.org/pdf/2604.06830 ; VGGT-Long: https://arxiv.org/pdf/2507.16443
- Monocular SLAM UAV evaluation (2026): https://arxiv.org/pdf/2608.18632
- MonoEM-GS (2026): https://arxiv.org/pdf/2604.10593 ; Embedded semantic SLAM survey (2026): https://arxiv.org/abs/2505.12384
- Spatial memory for robot navigation survey (2026): https://arxiv.org/pdf/2604.16482
- cuVSLAM paper: https://arxiv.org/html/2506.04359v3 ; Isaac ROS 4.1: https://discourse.openrobotics.org/t/nvidia-isaac-ros-4-1-for-thor-has-arrived/52446
- nvblox on Orin Nano Super (forum): https://forums.developer.nvidia.com/t/nvblox-is-fully-supported-on-jetson-orin-nano-super/323631
- STVL (Jazzy): https://docs.ros.org/en/ros2_packages/jazzy/api/spatio_temporal_voxel_layer
- Nav2 Depth Anything tutorial: https://docs.nav2.org/tutorials/docs/depth_ai_integration.html
- VIO comparison (2025): https://arxiv.org/pdf/2507.11241 ; Nav2 VIO tutorial: https://docs.nav2.org/tutorials/docs/integrating_vio.html
- ORB-SLAM3 ROS 2 wrapper (Humble): https://github.com/guyuehome/ORB_SLAM3_ROS2 ; stella_vslam ROS 2: https://stella-cv.readthedocs.io/en/stable/ros2_package.html
- OrbbecSDK ROS 2 (orbbec_camera, Jazzy): https://index.ros.org/p/orbbec_camera/
- Pi 5 RTAB-Map vendor note (Yahboom): https://www.yahboom.net/public/upload/upload-html/1713872512/7.%20RTAB-Map%203D%20mapping%20navigation.html
