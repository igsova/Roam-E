# Visual SLAM for Roam-E — state of the art, October 2026

Goal (15 Oct 2026): navigate in 3D using the RGB camera, the depth camera,
or both. This note surveys what exists now, what runs on Roam-E's hardware, and
proposes a staged path. It is desk research plus our own recorded Pi results.
Nothing here was run on the robot yet.

## Roam-E's starting point (facts that constrain the choice)

| Item | What we have | Why it matters |
|---|---|---|
| Compute | Raspberry Pi 5, 8 GB, **no CUDA GPU** | Rules out GPU-only SLAM onboard; CPU already loaded by Nav2 + camera (load 5–14 seen) |
| Camera | Orbbec Astra Pro: OpenNI depth 640×480 ~30 Hz; RGB as a separate UVC device (~25 Hz, rolling shutter), no hardware RGB↔depth sync | RGB-D works, but timing is software-paired; older OpenNI device |
| Other sensors | RPLIDAR A1 (2D), BNO055 IMU (low-cost, relative yaw), wheel odometry (calibrated 14 Oct) | Strong odometry/2D localization already; IMU too noisy for good VIO |
| Measured on our Pi (Sept) | RTAB-Map `rgbd_odometry` on the Astra: ~1.2–1.4 Hz, ~140 ms/frame, zero lost frames in short tests | Visual SLAM *does* run on the Pi, but slowly |
| Calibration (15 Oct) | Depth within ~1 cm of lidar to 1.5 m; camera mount corrected (2.1° up, 2.8° left) | Depth is usable for mapping and obstacles |

## The landscape in October 2026

### 1. Classical feature-based SLAM (CPU)

- **ORB-SLAM3** (mono / stereo / RGB-D / inertial): still the accuracy reference
  for sparse SLAM, multi-map relocalization. No official ROS 2; Jazzy needs a
  patched community wrapper. Produces a sparse map, not a navigation map.
- **stella_vslam** (ORB-SLAM-family, mono/stereo/RGB-D): maintained docs and a
  ROS 2 package; Jazzy support not stated.
- Verdict: good for *localization* research, poor for *navigation* (no dense or
  occupancy output), and integration on Jazzy is DIY.

### 2. RTAB-Map — graph SLAM built for robots (CPU)

- Mature, actively maintained; `rtabmap_ros` 0.23.x released for **Jazzy**
  (package index, updated June 2026).
- Takes RGB-D (or stereo / mono with external odometry) **plus wheel odometry
  and lidar**, does appearance-based loop closure, and outputs a 3D point cloud /
  OctoMap **and a 2D occupancy grid that Nav2 can use** (or runs localization on
  a saved map).
- On a Pi 5 it is slow (our 1.2–1.4 Hz odometry; one vendor offloads it from the
  Pi entirely). Using our wheel/EKF odometry instead of visual odometry removes
  the heaviest part and is how it is usually run on small robots.
- Verdict: **the practical, ROS-2-native choice for Roam-E today.**

### 3. Visual-inertial odometry (OpenVINS, VINS-Fusion, Basalt)

- A 2025 ROS 2 comparison found OpenVINS best on several metrics, RTAB-Map and
  VINS-Fusion best on others. Basalt has a maintained ROS wrapper (MOLA), but
  Jazzy isn't listed for it.
- All need a decent, time-synchronized IMU and ideally a global-shutter camera.
- Verdict: **not a good fit** with the BNO055 and the rolling-shutter Astra RGB.

### 4. GPU-accelerated geometric SLAM: NVIDIA cuVSLAM + nvblox

- cuVSLAM (Isaac ROS Visual SLAM) is the strongest *real-time embedded* option.
  It runs 1–32 cameras with optional IMU/depth, and Isaac ROS 4.1 added RGB-D
  support. **nvblox** builds a dense 3D map from depth and publishes a costmap
  for Nav2.
- Requires an NVIDIA Jetson (Orin Nano Super and up) or an RTX PC. NVIDIA says
  Orin Nano Super performance drops when the GPU is shared. A secondary source
  reports **Isaac ROS 5.0** released at ROSCon (Sept 2026) with breaking API
  changes; not verified against official notes.
- Verdict: the **"real 3D navigation" path if we add a Jetson.**

### 5. Learned dense SLAM (research frontier)

- **DROID-SLAM**, **MASt3R-SLAM** (CVPR 2025, ~15 FPS on a high-end GPU),
  **VGGT-SLAM / VGGT-SLAM++** (2025–2026): RGB-only (even uncalibrated) dense
  SLAM. Recent comparisons rank MASt3R-SLAM most consistent, DROID-SLAM close
  behind; VGGT-SLAM++ reports the best indoor (7-Scenes) numbers.
- **Gaussian-splatting SLAM** (SplaTAM, Splat-SLAM, MonoEM-GS 2026) gives
  photorealistic dense maps. A 2026 embedded-SLAM survey benchmarks these on a
  Jetson AGX Orin; memory grows with scene size.
- All need a desktop-class or Jetson-AGX-class GPU and have no standard Nav2
  integration. Verdict: **exciting, offboard-GPU only, research-grade for
  navigation.**

### 6. Monocular depth for navigation (RGB camera → depth)

- Nav2's official tutorial runs **Depth Anything 3** (TensorRT) on a plain USB
  camera, converts depth to a point cloud and feeds a voxel costmap layer.
- Learned depth is relative and gets worse with range: a Depth Anything V2
  study found 89% of errors within ±0.5 m under 2 m. It needs an NVIDIA GPU, and
  we found no Hailo / Pi AI HAT port.
- Verdict: an "RGB-only" path, but for **obstacles**, not mapping; needs GPU.

### 7. 3D obstacles in Nav2 from a depth camera (today, on the Pi)

- **Spatio-Temporal Voxel Layer (STVL)** is available for Jazzy and supports a
  depth-camera frustum. Nav2's own VoxelLayer also accepts point clouds.
- This gives Nav2 "3D awareness" (tables, overhangs, low objects the 2D lidar
  misses) without full visual SLAM.

## Recommended path for Roam-E

1. **Depth camera → Nav2 obstacles (Pi only, small).** Add the Astra depth as
   a voxel/STVL costmap source, using the calibrated mount, decimated to a few
   Hz. Immediate 3D obstacle avoidance; low risk; tests CPU/power headroom.
2. **RTAB-Map RGB-D SLAM with wheel odometry + lidar (Pi only, medium).** Map
   the office in 3D, project a 2D grid, compare against `office_02`/AMCL,
   then try RTAB-Map localization for Nav2 (only one `map→odom` owner). Run at
   1–2 Hz; measure CPU, temperature and 5 V while mapping.
3. **Decide on hardware for real-time 3D / RGB-only.** If steps 1–2 hit the
   Pi's limits, add an NVIDIA Jetson Orin Nano Super for cuVSLAM + nvblox (or
   Depth Anything 3), with the Pi kept for drivers/Nav2. A modern RGB-D camera
   (global-shutter, hardware-synced, built-in IMU) would help any visual
   method. The Astra Pro isn't supported by Orbbec's current ROS 2 SDK v2.
4. **Learned RGB-only SLAM (MASt3R/VGGT/Gaussian splatting)** as an offboard
   experiment on a GPU PC with recorded bags, not for onboard navigation yet.

## Unknowns to check before committing

- No published Raspberry Pi 5 RGB-D SLAM benchmarks were found; our own
  measurements are the best guide.
- Exact Isaac ROS 5.0 platform and ROS-distro support (secondary source only).
- Whether STVL's Jazzy binary and our Astra depth rate fit the Pi's CPU budget
  alongside Nav2 and the cameras.

## Sources

- RTAB-Map ROS index: https://index.ros.org/p/rtabmap_ros ; RTAB-Map paper: https://arxiv.org/pdf/2403.06341
- MASt3R-SLAM (CVPR 2025 supplement): https://openaccess.thecvf.com/content/CVPR2025/supplemental/Murai_MASt3R-SLAM_Real-Time_Dense_CVPR_2025_supplemental.pdf
- VGGT-SLAM: https://arxiv.org/pdf/2505.12549 ; VGGT-SLAM++: https://arxiv.org/pdf/2604.06830 ; VGGT-Long: https://arxiv.org/pdf/2507.16443
- Monocular SLAM UAV evaluation (2026): https://arxiv.org/pdf/2608.18632
- MonoEM-GS (2026): https://arxiv.org/pdf/2604.10593 ; Embedded semantic SLAM survey (2026): https://arxiv.org/abs/2505.12384
- Spatial memory for robot navigation survey (2026): https://arxiv.org/pdf/2604.16482
- cuVSLAM paper: https://arxiv.org/html/2506.04359v3 ; Isaac ROS 4.1: https://discourse.openrobotics.org/t/nvidia-isaac-ros-4-1-for-thor-has-arrived/52446
- Isaac ROS 5.0 (secondary): https://windowsforum.com/news/nvidia-isaac-ros-5-0-replaces-nitros-apis-with-rosidl-buffer.445539/
- nvblox on Orin Nano Super (forum): https://forums.developer.nvidia.com/t/nvblox-is-fully-supported-on-jetson-orin-nano-super/323631
- STVL (Jazzy): https://docs.ros.org/en/ros2_packages/jazzy/api/spatio_temporal_voxel_layer
- Nav2 Depth Anything tutorial: https://docs.nav2.org/tutorials/docs/depth_ai_integration.html
- VIO comparison (2025): https://arxiv.org/pdf/2507.11241 ; Nav2 VIO tutorial: https://docs.nav2.org/tutorials/docs/integrating_vio.html
- ORB-SLAM3 ROS 2 wrapper (Humble): https://github.com/guyuehome/ORB_SLAM3_ROS2 ; stella_vslam ROS 2: https://stella-cv.readthedocs.io/en/stable/ros2_package.html
- OrbbecSDK ROS 2 (orbbec_camera, Jazzy): https://index.ros.org/p/orbbec_camera/
- Pi 5 RTAB-Map vendor note (Yahboom): https://www.yahboom.net/public/upload/upload-html/1713872512/7.%20RTAB-Map%203D%20mapping%20navigation.html
