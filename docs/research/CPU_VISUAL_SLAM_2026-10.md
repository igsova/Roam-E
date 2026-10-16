# CPU-only RGB / RGB-D visual SLAM for Roam-E — follow-up, 16 October 2026

Question: which current methods give usable visual SLAM or camera
localization on the Pi 5 CPU alone? This follows the 15 Oct survey
([VISUAL_SLAM_SOTA_2026-10.md](VISUAL_SLAM_SOTA_2026-10.md)) after RTAB-Map's
own camera relocalization failed on the robot.

## What we measured on Roam-E (16 Oct)

Same data for every method: 3 live colour frames from one robot position,
matched against the 97 keyframes of the 16 Oct RTAB-Map map (aligned depth),
then PnP-RANSAC with keyframe depth (4 px). "Inliers" are matches that agree
on one camera pose.

| Features + matcher | Best inliers | Pose from top-2 keyframes | Pi 5 time |
|---|---|---|---|
| ORB + ratio test | 9–10 | scattered, unusable | fast |
| SIFT + ratio test | 5–7 | scattered, unusable | — |
| RTAB-Map built-in (GFTT/ORB, SIFT) | 0–13 of 20 needed | rejected | ~1 Hz |
| SuperPoint + LightGlue | 45–51 | agree within ~25 cm | 6 s extract + 0.5 s/pair |
| XFeat + mutual-NN | 36–43 | agree within ~8 cm | **0.6 s extract + 0.02 s/pair** |
| **XFeat + LighterGlue** | **67–74** | **agree within ~3 cm** | 0.6 s extract + 1.4 s/pair |

Pi timings with the cameras and Nav2 running, PyTorch 2.14 CPU, 640×480,
up to 1024 XFeat / 512 SuperPoint keypoints. Live relocalization with the
SuperPoint version on the robot gave a pose that the lidar confirmed (80 % of
scan points within 10 cm of map walls; best nearby alternative 82 %).

Conclusion: learned features fix the matching problem, not the camera. XFeat
is ~10× faster than SuperPoint on the Pi and matched better on our indoor
frames, consistent with its paper (XFeat 1.8 FPS vs SuperPoint 0.16 FPS on a
Cortex-A53; better indoor pose accuracy than DISK/ALIKE).

## Literature and software (October 2026)

- **XFeat (CVPR 2024)** — lightweight learned detector/descriptor (64-D),
  real-time on a laptop CPU, the only learned method over 1 FPS on a small ARM
  board in its tests. LighterGlue is its small LightGlue matcher.
- **xfeatSLAM** — XFeat inside ORB-SLAM3 (C++/libtorch). Experimental; no
  published accuracy or timing.
- **SuperPoint-SLAM3 (2025)** — ORB-SLAM3 with SuperPoint, adaptive NMS and a
  NetVLAD loop-closure head; large KITTI/EuRoC gains; timing hardware not
  stated (likely GPU). Too heavy for the Pi given our 6 s SuperPoint time.
- **ORB-SLAM3 on Raspberry Pi 5** — a 2026 preprint evaluates all modes on a
  Pi 5 (not accessible here). ROS 2 wrappers are Foxy/Humble-era; none is
  maintained for Jazzy, so it would be a port.
- **Loc-SLAM (2026)** — real-time RGB-D SLAM for dynamic scenes on low-power
  embedded devices; no ROS package found.
- **RTAB-Map** — supports learned features only through a Python plugin
  (PyDetector, feature type 15, e.g. SuperPoint) in source builds; the Jazzy
  apt build has no Python/PyTorch support (checked on the Pi). No XFeat or
  LightGlue support found.
- **Place recognition on CPU** — training-free/lightweight global descriptors
  (e.g. SSR-VLAD) run without GPU; a 2026 ISPRS study compares modern ones
  (MixVPR, SALAD, MegaLoc) as retrieval front-ends but gives no CPU timings.
- **cuVSLAM / Isaac ROS** — fastest option, GPU only (Jetson).

## Recommended path on the Pi

1. Replace SuperPoint in the relocalizer with XFeat: mutual-NN against all
   keyframes for retrieval (~2.5 s), LighterGlue on the top 2 for the pose
   (~3 s). About 5–6 s per fix instead of 25 s.
2. Run it periodically (e.g. every 10–15 s while driving) as camera-based
   correction on top of wheel odometry; keep the lidar for robustness where
   available.
3. Keep RTAB-Map for mapping (lidar ICP + depth) and the 3D map.
4. A full learned-feature SLAM front end (xfeatSLAM / ORB-SLAM3 port) is a
   larger project; worth it only if the periodic relocalizer is not enough.

## Sources

- XFeat paper: https://arxiv.org/pdf/2404.19174 ; code: https://github.com/verlab/accelerated_features
- xfeatSLAM: https://github.com/udaysankar01/xfeatSLAM
- SuperPoint-SLAM3: https://arxiv.org/abs/2506.13089
- ORB-SLAM3 on Raspberry Pi 5 (preprint): https://www.researchgate.net/publication/400559478
- Loc-SLAM (2026): https://link.springer.com/article/10.1007/s11370-026-00714-5
- Industrial cuVSLAM benchmark (2026): https://arxiv.org/html/2603.16240v1
- Embedded semantic SLAM survey: https://arxiv.org/pdf/2505.12384
- RTAB-Map SuperPoint Python detector: https://docs.ros.org/en/api/rtabmap/html/rtabmap__superpoint_8py_source.html
- VPR on embedded platforms: https://arxiv.org/pdf/2109.11002 ; ISPRS 2026 VPR study: https://isprs-annals.copernicus.org/articles/XI-2-2026/647/2026/
- ORB-SLAM3 ROS 2 wrappers: https://github.com/zang09/ORB_SLAM3_ROS2 , https://github.com/guyuehome/ORB_SLAM3_ROS2
- LightGlue ONNX: https://github.com/fabio-sim/LightGlue-ONNX
