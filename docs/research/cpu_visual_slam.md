# CPU-only visual SLAM and relocalization on a Raspberry Pi 5

October 2026. Question: which current methods give usable visual SLAM or
camera relocalization on the Pi 5 CPU alone? This follows the
[visual SLAM survey](visual_slam_survey.md), after RTAB-Map's own camera
relocalization never accepted a match on the robot.

## Measurements on Roam-E

Same data for every method: three live colour frames from one robot position,
matched against the 97 keyframes of an RTAB-Map map of the office (with
registered depth), then PnP-RANSAC using the keyframe depth (4 px threshold).
"Inliers" are matches consistent with one camera pose.

| Features + matcher | Best inliers | Poses from the top-2 keyframes | Pi 5 time |
|---|---|---|---|
| ORB + ratio test | 9–10 | scattered, unusable | fast |
| SIFT + ratio test | 5–7 | scattered, unusable | — |
| RTAB-Map built-in (GFTT/ORB, SIFT) | 0–13 of 20 needed | rejected | ~1 Hz |
| SuperPoint + LightGlue | 45–51 | agree within ~25 cm | 6 s extract + 0.5 s/pair |
| XFeat + mutual nearest neighbour | 36–43 | agree within ~8 cm | **0.6 s extract + 0.02 s/pair** |
| **XFeat + LighterGlue** | **67–74** | **agree within ~3 cm** | 0.6 s extract + 1.4 s/pair |

Timings on the Pi with the cameras and Nav2 running, PyTorch (CPU build),
640×480 images, up to 1024 XFeat or 512 SuperPoint keypoints. A live
relocalization on the robot with the SuperPoint version gave a pose consistent
with the lidar: 80 % of scan points lay within 10 cm of the map's walls.

Conclusion: the failures came from feature matching, not from the camera, and
learned features fix them. XFeat is about 10× faster than SuperPoint on the Pi and matched
better on these indoor frames, in line with its paper (XFeat 1.8 FPS against
SuperPoint 0.16 FPS on a Cortex-A53, and better indoor pose accuracy than
DISK or ALIKE).

## Literature and software (October 2026)

- **XFeat (CVPR 2024)**: lightweight learned detector and descriptor (64-D),
  real time on a laptop CPU, and the only learned method above 1 FPS on a
  small ARM board in its tests. LighterGlue is its small LightGlue matcher.
- **xfeatSLAM**: XFeat inside ORB-SLAM3 (C++/libtorch). Experimental; no
  published accuracy or timing.
- **SuperPoint-SLAM3 (2025)**: ORB-SLAM3 with SuperPoint, adaptive NMS and a
  NetVLAD loop-closure head; large KITTI/EuRoC gains; timing hardware not
  stated (likely GPU). Too heavy for the Pi, given the 6 s SuperPoint
  extraction measured above.
- **ORB-SLAM3 on a Raspberry Pi 5**: a 2026 preprint evaluates all modes on a
  Pi 5. The available ROS 2 wrappers target Foxy/Humble and none is maintained
  for Jazzy, so using it would mean a port.
- **Loc-SLAM (2026)**: real-time RGB-D SLAM for dynamic scenes on low-power
  embedded devices; no ROS package found.
- **RTAB-Map**: supports learned features only through a Python plugin
  (PyDetector, feature type 15, e.g. SuperPoint) in source builds. The Jazzy
  apt build on the Pi has no Python/PyTorch support. No XFeat or LightGlue
  support found.
- **Place recognition on CPU**: training-free or lightweight global
  descriptors (e.g. SSR-VLAD) run without a GPU; a 2026 ISPRS study compares
  modern ones (MixVPR, SALAD, MegaLoc) as retrieval front ends but reports no
  CPU timings.
- **cuVSLAM / Isaac ROS**: fastest option, GPU only (Jetson).

## Recommended path at the time

1. Replace SuperPoint in the relocalizer with XFeat: mutual nearest-neighbour
   matching against all keyframes for retrieval (~2.5 s), LighterGlue on the
   best candidates for the pose (~3 s). About 5–6 s per fix instead of 25 s.
2. Run it periodically (e.g. every 10–15 s while driving) as a camera-based
   correction on top of wheel odometry, keeping the lidar where available.
3. Keep RTAB-Map for mapping (lidar ICP + depth) and the 3D map.
4. Treat a full learned-feature SLAM front end (xfeatSLAM or an ORB-SLAM3 port)
   as a larger project, worth it only if the relocalizer is not enough.

## Outcome

Steps 1 and 3 were built; step 2 was built but is disabled by default. The
XFeat + LighterGlue relocalizer is in [`tools/rtabmap/`](../../tools/rtabmap/)
(see [`tools/README.md`](../../tools/README.md)) and runs alongside RTAB-Map
localization on its saved map:

- `visual_keyframes.py` builds a keyframe cache from an RTAB-Map export: XFeat
  keypoints and descriptors per keyframe, with 3D map points wherever the
  keyframe has depth.
- `visual_relocalizer.py` matches the live rectified colour image against
  every keyframe with fast mutual nearest-neighbour matching and PnP, re-matches
  the best three with LighterGlue, solves one combined PnP and checks it
  against the per-keyframe poses. The pose is carried from the image time to
  the present with odometry and published on `/rtabmap/initialpose`. It never
  commands motion.
- It runs when RTAB-Map reports it is lost, or on demand through
  `~/relocalize` (`std_srvs/Trigger`); one attempt takes about 10 s on the Pi.

The periodic correction from step 2 exists (`period` > 0) but stays at
`period` 0 on the robot. With the lidar running, RTAB-Map tracks within about
5 cm, while camera fixes land within 15–30 cm of the lidar pose. A periodic
camera correction would trade the lidar's accuracy for the camera's and, above
15 cm, cancels the active navigation goal. Commands are in
[`docs/quick_commands.md`](../quick_commands.md), section 5.

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
