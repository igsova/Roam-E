# On-board registration test: point cloud and RGB-D odometry

The Astra Pro can register depth to its colour camera on board
(`depth_registration=true` in the driver). This test checked whether that
registered depth, combined with the measured colour calibration, is good
enough to build a coloured point cloud and run RTAB-Map RGB-D odometry on the
Pi, without the offline RGB–IR calibration from
[depth_registration.md](depth_registration.md).

**Result (September 2026).** A metadata-consistent coloured cloud and RTAB-Map
visual odometry ran on the Pi with no lost frames in a 20 s window, at about
1.4 updates per second and 96 ms median compute per update. Motion accuracy,
scale, loop closure and mapping were outside the scope of this test.

The scripts keep their original `firmware_rgbd_` prefix. Here "firmware"
means the camera's on-board registration, not the Arduino code in
[`firmware/`](../../firmware/).

## Scope and isolation

- Runs next to the normal camera launch and reuses its topics. Nothing opens
  the USB device, changes camera parameters, commands the motors, starts Nav2
  or writes a map.
- All derived topics live under `/rgbd_test`. RTAB-Map runs with
  `publish_tf: false` and `odom_frame_id: rgbd_test_odom`, so it never
  competes with the robot's EKF or localization transforms.
- `camera_link` serves as the odometry body frame; it is not a measured
  camera-to-robot calibration.
- A second Foxglove bridge on the Pi's loopback port 8766 exposes only the
  test topics; services, parameters and client publishing are disabled.

## Data flow

| Source | Processing | Output |
|---|---|---|
| `/camera/color/image_raw/compressed` | Decode the ~5 Hz JPEG preview; match the exact-stamp colour CameraInfo | `/rgbd_test/rgb/image_rect` (`rgb8`, 640×480) |
| `/camera/depth_raw/image` | Require on-board registration; nearest depth header within 25 ms; rectify with the colour model | `/rgbd_test/depth/image_rect` (`16UC1`, mm, 640×480) |
| Colour CameraInfo | Same K for both images; D = 0, R = I and P = [K 0] after rectification | `/rgbd_test/{rgb,depth}/camera_info` |
| Rectified colour + depth | Back-project every 4th pixel, keep 0.3–5 m, about 1 Hz | `/rgbd_test/points` (coloured `PointCloud2`) |
| Full-resolution rectified pair | RTAB-Map `rgbd_odometry`, at most 2 updates/s | `/rgbd_test/odom`, `/rgbd_test/odom_info_lite` |

| File | Role |
|---|---|
| [`firmware_rgbd_math.py`](../../scripts/camera/firmware_rgbd_math.py) | Camera-model validation, joint rectification, depth decoding (endianness, row padding), header pairing, XYZ/RGB cloud packing |
| [`firmware_rgbd_test.py`](../../scripts/camera/firmware_rgbd_test.py) | Live adapter node; JSON diagnostics on `/rgbd_test/status` |
| [`firmware_rgbd_test.launch.py`](../../scripts/camera/firmware_rgbd_test.launch.py) | Starts the adapter, bridge and odometry, each switchable |
| [`check_firmware_rgbd.py`](../../scripts/camera/check_firmware_rgbd.py) | Bounded observer: rates, metadata consistency, pose and inlier statistics, a saved cloud and two rendered views |
| [`test_firmware_rgbd.py`](../../scripts/camera/test_firmware_rgbd.py) | Seven offline numerical tests; no camera or ROS needed |

## Assumptions

**Geometry.** The test assumes that on-board registered depth pixels line up
approximately with the raw colour pixels. Both images are rectified through
the same measured colour K and D (bilinear for colour, nearest-neighbour for
depth so that surfaces and invalid zeros are not blended), giving one shared
pinhole model. This makes the metadata consistent; it does not calibrate the
depth camera or remove registration error. No pixel offset or depth
correction is applied, and invalid depth stays 0 and is left out of the
cloud. The offline RGB–IR model is deliberately not applied: the depth is
already registered, and registering it again would be wrong.

**Timing.** For each colour frame the adapter waits up to 180 ms for
exact-stamp CameraInfo and for a later depth frame, then takes the nearest
depth header within 25 ms. Both derived images carry the colour stamp, so
RTAB-Map can use exact synchronisation; the original depth-minus-colour delta
is reported on `/rgbd_test/status`. This is header pairing, not exposure
synchronisation.

**Guards.** Every 2 s the adapter reads the driver's `depth_registration`
parameter and drops pairs unless it is true. It also drops data older than
0.6 s, mismatched frame IDs and a missing or stale depth CameraInfo, and it
rejects every pair if the colour calibration changes mid-run.

**Odometry settings.** Frame-to-map visual odometry (`Odom/Strategy 0`,
`Reg/Strategy 0`), exact sync, `Rtabmap/ImagesAlreadyRectified true`,
`publish_null_when_lost true`, no automatic reset (`Odom/ResetCountdown 0`),
and one OpenMP/BLAS thread.

## Running

Needs the working camera stack from [setup.md](setup.md) plus RTAB-Map
odometry (tested with rtabmap 0.23.7):

```bash
sudo apt install --no-install-recommends ros-jazzy-rtabmap-odom
```

With the camera launch already running, start the adapter and bridge in one
terminal:

```bash
cd ~/Roam-E && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch scripts/camera/firmware_rgbd_test.launch.py tracking:=false
```

In a second terminal, start a bounded odometry run without duplicating them:

```bash
timeout --signal=INT --kill-after=10s 60s \
  ros2 launch scripts/camera/firmware_rgbd_test.launch.py \
  tracking:=true adapter:=false bridge:=false tracking_max_rate:=2.0
```

Exit code 124 after the timeout is expected. Make sure the `rgbd_odometry`
process has exited before starting another. While it runs, observe from a
third terminal (omit `--tracking` for a cloud-only check):

```bash
cd ~/Roam-E/scripts/camera
python3 check_firmware_rgbd.py --seconds 20 --tracking --output <output-dir>
```

The observer passes only if every exact-stamp bundle of colour, depth and both
CameraInfos is consistent (same frame, size, K and P, zero distortion), a cloud
arrived, and, with `--tracking`, at least two finite poses and no `lost` flag
were received.

**Foxglove.** Forward the port with `ssh -N -L 8766:127.0.0.1:8766 <user>@<pi-host>`
and connect to `ws://localhost:8766`. In a 3D panel, set the display frame to
`camera_link` and show `/rgbd_test/points` coloured by its `rgb` field. Plot
`/rgbd_test/odom` and `/rgbd_test/odom_info_lite` in Plot or Raw Messages
panels; no `rgbd_test_odom` TF is broadcast, and there is no `map` or `odom`
frame in this test. The cloud is the current camera view, not an accumulated
map.

## Results

**Cloud only (15 s observation).** 44 complete image + CameraInfo bundles, all
with matching 640×480 sizes, frames, stamps and rectified models. The adapter
published 4.3–4.6 pairs/s; the cloud arrived at about 0.92 Hz. A saved cloud
had 10,043 finite coloured points between 0.58 and 3.58 m (160,688 bytes);
other moments had about 15,800 points, depending on how much of the scene had
valid depth. Rendered views showed missing foreground surfaces and holes at
depth edges, so this is not evidence of pixel-exact alignment.

**Odometry (20 s observation inside a 60 s run).**

| Metric | Value |
|---|---|
| Odometry messages | 27 (about 1.36 Hz, cap 2 Hz) |
| Lost frames | 0 of 27 |
| Inliers per update | median 88, range 51–126 |
| Compute per update | median 95.9 ms, p95 107.6 ms |
| Odometry header age | median ~204 ms, p95 232 ms |
| Consistent RGB-D bundles | 31 of 31 received |

The reported pose spread over 2.8, 3.9 and 2.4 cm on its three axes, 2.3 cm
first to last. Camera motion was not measured and a person filled much of the
view, so these figures are not a drift measurement. Large best-effort image
topics dropped some messages at the observer; counts are not camera frame
rates. The Pi reached its soft temperature limit during these runs, so
sustained vision plus navigation needs active cooling.

## What followed

The robot's RTAB-Map pipeline builds on on-board registration rather than the
offline stereo model. The production launch
([`rtabmap_mapping.launch.py`](../../src/roam_e/launch/rtabmap_mapping.launch.py))
keeps the same idea, registered depth described by the colour calibration,
with three changes: colour comes from `image_proc`'s rectified image, depth is
shifted by the measured 7 px left and 1 px up instead of being remapped (see
[depth_registration.md](depth_registration.md#deployed-correction-on-board-registration-plus-a-measured-shift)),
and odometry comes from the wheel/IMU EKF with lidar ICP refinement instead of
visual odometry.

## References

- [RTAB-Map RGB-D odometry](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/src/nodelets/rgbd_odometry.cpp): inputs and synchronisation.
- [RTAB-Map odometry ROS wrapper](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/src/OdometryROS.cpp): `publish_tf`, frame IDs, update cap, loss behaviour.
- [ROS depth_image_proc](https://docs.ros.org/en/jazzy/p/depth_image_proc/doc/index.html): depth image conventions.
