# Firmware-registered RGB-D: isolated point cloud and tracking test

Status, 2026-09-18: **initial live integration test passed**. A colored point
cloud and RTAB-Map RGB-D odometry were produced. Motion accuracy, scale, loop
closure, mapping, relocalization, and navigation integration are **not validated**.
Calibration investigation is paused; no additional chessboard capture is needed
for this experiment.

## Scope and isolation

- Existing `astra_camera.launch.py` remains running, unchanged. It supplies
  firmware-registered depth and the separate libuvc RGB stream.
- Nothing here opens USB, flashes firmware, changes camera parameters, publishes
  velocity commands, launches a motor driver, starts Nav2, or writes a map.
- All derived topics use `/rgbd_test`. RTAB-Map uses `publish_tf: false` and
  `odom_frame_id: rgbd_test_odom`, consuming only existing camera transforms.
  It never competes with the EKF/AMCL transform publishers.
- `camera_link` is the odometry body frame for this camera-only experiment. It
  is **not** a measured camera-to-robot mounting calibration.
- A second read-only Foxglove bridge listens on Pi **127.0.0.1:8766**. Original
  port 8765 is untouched. Services, parameter access, and client publishing are
  disabled on this experiment bridge.

## Code and data flow

| Source | Processing | Derived output |
|---|---|---|
| `/camera/color/image_raw/compressed` | Decode existing ~5 Hz native JPEG preview; match exact-stamp RGB CameraInfo | `/rgbd_test/rgb/image_rect`, `rgb8`, 640x480 |
| `/camera/depth_raw/image` | Require firmware registration; nearest header within 25 ms; joint rectification | `/rgbd_test/depth/image_rect`, `16UC1` millimetres, 640x480 |
| Measured RGB CameraInfo | Shared K/P, identity R, zero D after rectification | `/rgbd_test/{rgb,depth}/camera_info` |
| Rectified RGB + depth + K | Back-project at stride 4; valid 0.3–5 m points; ~1 Hz | `/rgbd_test/points`, colored `PointCloud2` |
| Full-resolution derived RGB-D | RTAB-Map visual odometry, maximum 2 updates/s by default | `/rgbd_test/odom`, `/rgbd_test/odom_info_lite` |

Files:

- `firmware_rgbd_math.py`: numerical camera validation, joint rectification,
  depth decoding (including endianness/row padding), bounded pairing, XYZ/RGB.
- `firmware_rgbd_test.py`: live adapter with stale-data checks, an ongoing
  read-only `depth_registration` parameter check, exact-stamp RGB metadata, and
  `/rgbd_test/status` JSON diagnostics.
- `firmware_rgbd_test.launch.py`: independently switch adapter, bridge, and
  tracking. Camera/robot bringup is deliberately not included.
- `check_firmware_rgbd.py`: bounded observer; saves metadata/rate/pose statistics,
  a numeric cloud archive, rectified pair, and two projected cloud views to an
  explicitly supplied **new private directory outside Git**.
- `test_firmware_rgbd.py`: seven offline numerical tests; no camera/ROS needed.

## The explicit geometric and timing assumptions

The OpenNI topic is named `depth_raw/image` because it carries integer depth,
not because registration is disabled. The running driver's `depth_registration`
is true and the output frame is `camera_rgb_optical_frame`.

Its generic OpenNI CameraInfo (roughly fx=fy=570.34, cx=319.5, cy=239.5, D=0)
differs from the measured RGB calibration. Do not claim those models are equal.
This baseline **assumes the firmware-registered depth pixels approximately
correspond to the raw UVC RGB pixels**. It rectifies both through the same
measured RGB K/D mapping, with linear interpolation for RGB and nearest-neighbor
sampling for metric depth. The resulting test images share one rectified
pinhole model. This makes the metadata internally consistent; it does **not**
establish new calibrated depth intrinsics/extrinsics or erase registration error.

The private offline RGB–IR candidate is not applied. Applying it again to depth
that the firmware already registered would be a second geometric registration.
No arbitrary pixel offset, depth scale correction, or metric bias is added.
Zero/invalid depth remains zero; it is excluded from the preview cloud.
Colored depth preview JPEGs are never used as metric depth for tracking.

The adapter waits briefly for neighboring depth frames and accepts a nearest
header within 25 ms, with fresh exact-stamp RGB CameraInfo. Both derived images
receive the RGB timestamp for downstream exact message sync; the original RGB
and depth stamps/difference are retained in `/rgbd_test/status`. This is
**software header pairing, not hardware exposure synchronization**. The existing
depth timestamp offset is unchanged. Rapid movement can still expose timing
error; no such correction is inferred from small header deltas.

## Packages and commands

Existing dependencies: `ros-jazzy-foxglove-bridge`, `python3-opencv`,
`python3-numpy`, ROS message types and the working camera stack documented in
`../../CAMERA_CAPTURE.md`. The tracking package was installed with:

```sh
sudo apt install --no-install-recommends ros-jazzy-rtabmap-odom
```

APT also brings RTAB-Map core/messages/conversions/sync/util and substantial
PCL/VTK development dependencies. Verified installed versions in this test:
`rtabmap-odom 0.23.7-1noble.20260903.052744`,
`rtabmap 0.23.7-1noble.20260903.015447`,
`rtabmap-msgs 0.23.7-1noble.20260902.141653`.

With the normal **camera-only** launch already running, in another Pi terminal:

```sh
cd ~/Roam-E
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch scripts/camera/firmware_rgbd_test.launch.py tracking:=false
```

Then, in another Pi terminal, start a bounded tracking test without duplicating
the adapter or bridge:

```sh
cd ~/Roam-E
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
timeout --signal=INT --kill-after=10s 60s \
  ros2 launch scripts/camera/firmware_rgbd_test.launch.py \
  tracking:=true adapter:=false bridge:=false tracking_max_rate:=2.0
```

Exit code 124 is expected after this timeout if shutdown is otherwise clean.
Check that the `rgbd_odometry` process exits; do not leave multiple odometry
instances running. `OMP_NUM_THREADS=1` and `OPENBLAS_NUM_THREADS=1` apply only
to the experimental tracking child. Actual updates may be below the cap.

For a bounded observation while the above test runs (choose a fresh output name):

```sh
cd ~/Roam-E/scripts/camera
python3 check_firmware_rgbd.py --seconds 20 --tracking \
  --output /home/robot/roam-rgbd-private-test-01
```

Omit `--tracking` for the cloud-only phase. The observer's tracking smoke result
requires actual finite poses and non-lost odometry diagnostics; it is not a
motion-accuracy certificate.

## Foxglove

For the current reverse-SSH tunnel, forward on the Mac (not the Pi):

```sh
ssh -N -p 2222 -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:18766:127.0.0.1:8766 robot@127.0.0.1
```

Port 18766 was chosen because local port 8766 was already occupied; do not kill
another process to reuse it. Connect Foxglove to **`ws://localhost:18766`**, add
a 3D panel, set fixed/display frame to `camera_link`, and enable
`/rgbd_test/points` with its `rgb` color field. Original RGB and depth-preview
JPEGs are also available on this bridge. Do not select `map` or `odom`: the
camera-only test intentionally has no navigation/global frame.

To inspect odometry, use a Plot or Raw Messages panel on `/rgbd_test/odom` and
`/rgbd_test/odom_info_lite`. No `rgbd_test_odom` TF is broadcast, so an odometry
pose overlay in the existing camera frame will not resolve automatically.
The cloud is a live camera-relative cloud, **not an accumulated SLAM map**.

## Observed results on 2026-09-18

Cloud-only private sample: `/home/robot/roam-rgbd-firmware-cloud-20260918-01`.
During 15 seconds, 44 complete image+CameraInfo bundles were observed; all 44
had matching 640x480 dimensions, frames, timestamps, and rectified camera models.
The adapter published around 4.3–4.6 pairs/s; observer arrivals were lower for
large images with best-effort QoS. Cloud rate was about 0.92 Hz. The saved cloud
had 10,043 finite colored points, 0.58–3.58 m depth, 6,560 distinct colors, and
160,688 bytes. Other moments had ~15,800 points; valid depth varies by scene.
Projected cloud views were visually inspected. Missing foreground surfaces and
depth edge holes remain; this is not proof of pixel-perfect alignment.

Tracking private sample: `/home/robot/roam-rgbd-firmware-tracking-20260918-01`.
Within the 60-second launch, a 20-second observer received:

- 27 odometry messages, about **1.36 Hz** with the maximum set to 2 Hz.
- 27 odometry diagnostics, **zero `lost` messages** in that observation window.
- Median **88 inliers**, range 51–126.
- Median tracking compute **95.9 ms**, p95 107.6 ms.
- Median odometry header age about **204 ms**, p95 232 ms.
- 31 complete RGB-D bundles, all metadata-consistent. Large best-effort image
  topics dropped some samples at this observer; counts are not hardware FPS.

Observed pose varied over approximately 2.8/3.9/2.4 cm on its three axes, with
2.3 cm first-to-last separation. **Camera motion and scene stillness were not
independently measured**, and a person occupied much of the RGB view. Do not
call that an accuracy/drift measurement. Tracking initialized and ran without
loss in the observed interval; controlled motion remains the next validation.
The timeout then stopped RTAB-Map cleanly. Camera capture and cloud remained on.

The Pi reached 82.9 C and reported `0x80008` around installation/inspection,
then measured 77.9 C during the tracking sample and 76.3 C afterward, both with
`0x80000`. Thermal headroom is limited; these results are not a sustained-load
qualification. Cooling is needed before long combined navigation/vision runs.

## Next validation, not performed yet

Keep motors off. With the camera securely supported and cable slack, first
observe a stationary textured scene with people out of view; then make a small,
slow, measured translation/rotation and return. Compare the reported motion,
inliers, lost flags, latency, and temperature. No chessboard is required.
Do not conflate moving objects in front of a stationary camera with camera
motion. Mapping/loop closure and robot-frame integration come afterward.

## Primary implementation references

- [RTAB-Map RGB-D odometry](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/src/nodelets/rgbd_odometry.cpp): RGB/depth/CameraInfo inputs and synchronization.
- [RTAB-Map odometry ROS wrapper](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/src/OdometryROS.cpp): `publish_tf`, frame IDs, update cap, diagnostics, and loss behavior.
- [ROS depth_image_proc](https://docs.ros.org/en/jazzy/p/depth_image_proc/doc/index.html): metric float/OpenNI uint16 depth conventions.
