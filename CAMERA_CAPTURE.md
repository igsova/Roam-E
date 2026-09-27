# Astra camera capture

## Linux setup (Ubuntu 24.04 ARM64 / ROS 2 Jazzy)

With the official ROS apt repository already configured, install the packages
used during bring-up:

```bash
sudo apt update
sudo apt install -y \
  v4l-utils libusb-1.0-0 libusb-1.0-0-dev libuvc-dev \
  python3-numpy python3-opencv python3-yaml \
  ros-jazzy-openni2-camera ros-jazzy-v4l2-camera ros-jazzy-image-proc \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport \
  ros-jazzy-foxglove-bridge
sudo usermod -aG video robot
```

Log out and reconnect SSH after adding the group. `id -nG` must include
`video`. `libuvc-dev` installs the runtime libuvc library; libusb development
headers were used to compile the diagnostic C probe, not by the Python node.
`v4l-utils` and `ros-jazzy-v4l2-camera` were used for diagnostics/the older RGB
path. The final RGB node uses libuvc directly. The compressed transports are
kept for ROS image transport; Foxglove depth uses our colorized JPEG preview,
not the `compressedDepth` wire format.

Verified versions: libuvc 0.0.7, libusb 1.0.27, OpenCV 4.6.0, NumPy 1.26.4,
openni2_camera 2.2.2, v4l2_camera 0.7.1, image_proc 5.0.12,
image transports 4.0.7.

`scripts/setup/bootstrap_pi5.sh` installs declared ROS/package dependencies
through rosdep, adds the `video` group, and installs the two scoped USB rules.
For manual setup, put this line in `/etc/udev/rules.d/56-orbbec.rules`:

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0403", MODE:="0660", GROUP:="video"
```

Put the RGB rule in `/etc/udev/rules.d/57-roam-astra-rgb.rules`:

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0502", MODE:="0660", GROUP:="video"
```

Review any existing rules before editing them. Run
`sudo udevadm control --reload-rules`, then unplug/reconnect the camera.
Reloading rules alone does not update an already connected device's access.
Do not run the ROS camera stack as root.

## Legacy depth SDK (separate from apt)

The system OpenNI2 package alone does not supply the working legacy Orbbec
driver. Supply the previously validated ARM64 SDK bundle at:

```text
/home/robot/OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin
```

The tested `Samples/Bin` core reports OpenNI 2.3.0 build 63; do not substitute
the bundle's different `Redist` binaries without retesting. Vendor binaries
are not committed to this repository. This setup does not flash firmware.

In `Samples/Bin/OpenNI2/Drivers/orbbec.ini`, retain `[Device] UsbInterface=2`
and `[Depth] Resolution=1`, `InputFormat=3`. The failed ISO/compressed-depth
experiments used temporary SDK copies and are not part of this setup.

Ensure the ROS wrapper's library name resolves:

```bash
cd ~/OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin
ln -sfn libOpenNI2.so libOpenNI2.so.0
```

The launch scopes `LD_LIBRARY_PATH` and `OPENNI2_DRIVERS_PATH` to this SDK;
it does not replace system libraries. Use `openni2_sdk_dir:=/path/to/Samples/Bin`
to override the location. Use a sufficient Pi power supply and a secure USB
cable; the old 2–3 A buck supply was not validated for this full camera load.

## Build, launch, and view

On the Pi (zsh):

```zsh
cd ~/Roam-E
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select roam_e --symlink-install
source install/setup.zsh
ros2 launch roam_e astra_camera.launch.py
```

On the viewing computer, keep this tunnel running:

```bash
ssh -N -L 8765:127.0.0.1:8765 robot@robot-pi5.local
```

Connect Foxglove to `ws://localhost:8765`. Select
`/camera/color/image_raw/compressed` for RGB and
`/camera/depth_preview/compressed` for depth. The preview is not metric depth;
use `/camera/depth_raw/image` locally for depth processing. Camera-only launch
does not publish the navigation `map` frame. Stop with Ctrl+C before launching
another copy. Do not compete with this libuvc node using V4L2/OpenCV tools.

## Capture behavior and validation

Run `ros2 launch roam_e astra_camera.launch.py` after sourcing ROS Jazzy and
the workspace. This camera-only launch does not start navigation or motors.
Foxglove remains loopback-only on port 8765 and requires the SSH port forward.

Defaults: native MJPEG RGB 640x480 at requested 30 FPS (observed approximately
25 FPS), direct libuvc capture with an 800-byte USB payload reservation,
JPEG preview capped at 5 FPS, and decoded
RGB capped at 2 FPS only when `/camera/color/image_raw` has a subscriber.
Depth remains 640x480 at 30 FPS. Preview topic names are unchanged.

The worker continuously drains capture and retains only the latest frame.
USB transfer buffers are separate from this latest-frame slot. JPEG previews pass
through without decoding or re-encoding; reducing preview FPS alone does not
reduce USB capture traffic. Invalid/incomplete JPEGs are skipped. A failed
camera read is logged and terminates the node instead of replaying stale data.

Parameters exposed as launch arguments: `color_width`, `color_height`,
`color_preview_fps`, `color_decode_fps` (zero disables raw decoding),
`color_calibration_file`, and `rectify_color`. Node parameters are startup-only;
restart the camera launch to change them.
The supported direct-capture mode is 640x480 at requested 30 FPS. The camera
is selected by USB VID/PID 2bc5:0502, not by /dev/video number. `libuvc-dev`
is installed through rosdep; runtime uses libuvc.so.0. The bootstrap adds
a scoped 0502/video-group udev rule; reconnect the camera after installing it.
Only one launch may own the camera at a time.

V4L2 VGA RGB repeatedly stalled alongside VGA depth. Direct libuvc negotiation
reported a 1600-byte payload reservation; requesting 800 bytes allows the
camera's native JPEG stream to coexist with full-resolution depth. This is a
per-stream USB setting, not a firmware flash, depth reduction, or image resize.
The earlier 320x240 fallback is superseded. No modified experimental SDK is used.

RGB timestamps are host receipt timestamps, not hardware exposure times.
The measured RGB-only calibration described below is published; depth/color
registration and synchronization remain unverified. Changing resolution requires
matching calibration. This is not yet a calibrated RGB-D SLAM input.

On 2026-09-14, both VGA streams passed a 180-second simultaneous test with
816 JPEG preview messages, 337 decoded RGB messages, and 1536 depth previews.
JPEG decoding verified 640x480 RGB, and actual raw-depth messages reported
640x480. Native RGB capture recorded 4495 JPEG frames with zero invalid frames
at its 180-second report. This is a bounded bench test, not a long-duration
reliability or SLAM-calibration guarantee.

## Measured RGB calibration and rectification

`src/roam_e/config/astra_rgb_640x480.yaml` holds the RGB-only bench calibration
from 17 distinct chessboard views captured on 2026-09-18. The target has 7 x 7
internal corners with 35 mm spacing. The selected plumb_bob model fixes k3 to
zero: training RMS 0.508 px, pooled leave-one-view-out RMS 0.517 px, worst held-out
view 0.866 px. These are consistency checks using the same board, not independent
metric ground truth. See `scripts/camera/README.md` for provenance and limitations.
Private images and full analysis reports stay outside Git.

The launch loads the installed YAML through `color_calibration_file`. The RGB
node rejects invalid matrices, non-finite values, unsupported models, and
resolution mismatches before opening USB. It publishes
`/camera/color/camera_info` once for every frame used by either the JPEG preview
or decoded RGB stream, with exactly that frame's timestamp and optical frame.
K and D describe the raw RGB lens; identity R and P describe a monocular
rectified image, **not** the transformation between RGB and depth cameras.

`image_proc/rectify_node` subscribes to raw RGB plus matching CameraInfo on demand
and publishes `/camera/color/image_rect`. Its compressed transport is exposed to
Foxglove as **`/camera/color/image_rect/compressed`**. Select this topic in an Image
panel and leave **Rectify OFF** because these pixels are already rectified. The
original `/camera/color/image_raw/compressed` is still the native JPEG passthrough.
The rectified stream follows `color_decode_fps` (default 2 FPS); the native preview
is capped at 5 FPS. No viewer/processing consumer means no rectification workload.
Rectified JPEGs are decoded, remapped and recompressed; this does not reduce USB
capture traffic. Small black borders may appear where correction samples outside
the source image.

To disable only rectification, launch with `rectify_color:=false`. To explicitly
return to an uncalibrated RGB preview, use both `color_calibration_file:=''` and
`rectify_color:=false`; no placeholder RGB CameraInfo is published in that mode.
Keep the calibration for this physical RGB camera and resolution only. Do not
copy its values into the depth/IR CameraInfo. Depth intrinsics, RGB-depth
extrinsics, registration and exposure timing still need separate validation.

Implementation: `roam_e/rgb_calibration.py`, `roam_e/mjpeg_camera_node.py`, and
`launch/astra_camera.launch.py`, all under `src/roam_e/`.
Rectification uses the standard ROS
[image_proc implementation](https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/src/rectify.cpp).

### Runtime validation (2026-09-18)

Run the bounded check with the camera-only launch already running and ROS sourced:

```bash
python3 scripts/camera/check_rgb_rectification.py --seconds 30
```

It subscribes locally, checks 640x480 RGB/depth formats, matches raw/rectified/
CameraInfo headers exactly, and compares rectified pixels against an independent
OpenCV remap. It temporarily activates raw decoding/rectification; it does not
change parameters or issue robot commands. Optional `--output-dir` must name a
new private directory outside the repo and saves one raw/rectified frame pair
and a report. Without that option, no images/files are saved.

The first software restart produced heavy depth packet-corruption warnings and
about 2 FPS depth, so the combined check failed despite correct RGB output. A
full graceful camera-stack stop/restart recovered the stream after transient
startup warnings. The next 30-second check passed: 853 depth frames (28.6 Hz),
49 rectified JPEGs (1.68 Hz observed), and 36 exact-stamp raw/rectified/CameraInfo
triples. Pixel comparisons differed only by tiny interpolation rounding; median
Pi-local rectified delivery age was 62 ms (maximum 72 ms). This is not a measure
of end-to-end Foxglove latency. Final temperature was 68.6 C, throttled=0x0.

The check uses best-effort subscriptions and can drop frames; observed rates are
not a capture-FPS guarantee. The native RGB capture remained about 25 FPS with
zero invalid JPEGs in the observed counters. Software restart behavior remains
a reliability caveat; do not mistake this bounded pass for long-duration or
RGB-D registration validation.

## RGB-depth registration and timing: validation in progress

After RGB calibration, the 2026-09-18 read-only input check observed native RGB
JPEG previews at 4.53 Hz, decoded RGB at 1.82 Hz, and depth at 29.12 Hz. Both image
streams were 640x480. Stamps were monotonic; all 50 overlapping RGB frames had a
depth stamp within 20 ms. Median local receipt ages were 15.3 ms for decoded RGB
and 38.3 ms for depth. **These header statistics do not measure exposure timing.**

The active `/camera/driver` parameters were `depth_registration=true`,
`use_device_time=true`, `depth_time_offset=-0.033`,
`color_depth_synchronization=false`, and empty RGB/depth calibration URLs.
Vendor logs also reported depth registration enabled. Nevertheless,
`/camera/depth_raw/camera_info` still contained the fallback color model:
fx=fy=570.342, cx=319.5, cy=239.5, D=0. Our measured separate RGB model is
fx=580.354, fy=574.668, cx=320.607, cy=249.588, with nonzero distortion.

The driver publishes the color frame ID and color CameraInfo when its registration
flag is enabled. The `depth_raw` topic name refers to integer metric depth and
does **not** establish that the pixels remain in the native IR geometry. Conversely,
the RGB frame ID alone does **not** prove that those pixels align with the separate
UVC RGB camera. Do not copy the new RGB coefficients into depth CameraInfo as a
substitute for checking actual geometry. Do not apply a guessed constant pixel
shift: different viewpoints can produce depth-dependent parallax.

Read-only, bounded diagnostics (source ROS first; camera launch already running):

```bash
python3 scripts/camera/check_rgbd_inputs.py --seconds 30
python3 scripts/camera/inspect_rgbd_pair.py --seconds 15 \
  --output-dir /home/robot/roam-e-rgbd-alignment-UNIQUE_SESSION
```

The first command prints timing, image and CameraInfo summaries; it saves no
pixels. The second requires a new directory outside the repository and saves
one nearest-header pair with exact-stamp metadata, native millimetre depth,
raw/rectified RGB, and two **same-pixel diagnostic overlays**. It applies no
RGB-depth transformation and changes no camera parameters. Cyan marks >80 mm
jumps between valid neighboring depths; yellow marks invalid-depth boundaries,
which must not be interpreted as measured obstacle edges. Black in the depth
preview is invalid data, not free space.

The initial room pair was saved privately in
`/home/robot/roam-e-rgbd-alignment-20260918-scene-01`; its depth-minus-RGB header
delta was 12.95 ms and nonzero depth coverage was 56.1%. Visual overlap suggested
a possible residual offset, but invalid-depth boundaries and moving objects make
that scene inconclusive. No transform, timing offset, driver calibration URL,
depth mode, or firmware setting was changed as a result.

Controlled checks use the stationary, flat board upright near 1 m from the camera,
with the full board visible and the background at least 30 cm farther away. Compare
recognizable board boundaries and plane depths at multiple distances/positions;
do not fit calibration from one uncontrolled room screenshot. Depth metric scale,
RGB-depth extrinsics/registration, and actual exposure timing remain unverified.
RGB-D SLAM is not enabled yet. Motors/navigation stay off for these tests.

Relevant upstream behavior:
[OpenNI2 driver registration and CameraInfo](https://github.com/ros-drivers/openni2_camera/blob/jazzy/openni2_camera/src/openni2_driver.cpp),
[depth timestamp conversion](https://github.com/ros-drivers/openni2_camera/blob/jazzy/openni2_camera/src/openni2_frame_listener.cpp).

### First stationary-board pair (2026-09-18)

Saved privately as `/home/robot/roam-e-rgbd-alignment-20260918-board-01`.
RGB/depth headers differed by -13.33 ms (depth minus RGB); each CameraInfo matched
its image stamp exactly. All 49 internal corners were detected. With 35 mm
squares and the measured RGB intrinsics, solvePnP produced 0.336 px reprojection
RMS and estimated the board center at Z=0.916 m, tilted about 6.3 degrees.

Within an eroded internal-corner hull, all 16,116 pixels had valid depth. Median
depth was 913 mm, with median absolute deviation 6 mm and 5th–95th percentiles
901–926 mm. Under the **unverified same-pixel-registration hypothesis**, comparing
these readings with the RGB-estimated board plane gave median residual -2.39 mm
(5th–95th percentiles -4.45 to +0.94 mm). This supports rough distance consistency
at this pose, not millimetre calibration accuracy: RGB intrinsics, board geometry,
flatness, registration and sensor noise all affect the comparison. No independent
tape distance was supplied for this capture.

The depth surface is clearly detected, but its outline differs from RGB and has
invalid-depth bands along parts of the perimeter. Occlusion/shadows and actual
registration error must be distinguished before choosing a correction. Do not
fit a global pixel shift or change depth CameraInfo from this one view.
Next collect a stationary pair at about 1.3–1.5 m, in roughly the same image
region, with the camera unchanged and a more distant background.

`scripts/camera/analyze_board_pair.py PRIVATE_PAIR_DIRECTORY` reproduces the
offline corner/PnP/plane check and writes `board_analysis.json` plus a diagnostic
`board_roi.png` beside the private pair. It does not open USB, publish ROS data,
apply calibration, or touch robot controls. Runtime health at capture was
70.3 C, throttled=0x0. Original private images remain outside Git.

### Second stationary-board pair, farther range (2026-09-18)

Saved privately as `/home/robot/roam-e-rgbd-alignment-20260918-board-02`.
The cable had been bumped/reconnected between captures: the RGB driver's
five-second watchdog exited, and only RGB was restarted with the same calibration;
depth, rectifier and bridge were left running. Do not assume an unchanged camera
orientation between the two views; each is evaluated using its own RGB board pose.

All 49 internal corners were detected, with 0.269 px RGB pose reprojection RMS.
The RGB-estimated board center was Z=1.4539 m with about 21.9 degrees tilt.
The eroded internal-board ROI contained 5,681 valid depth pixels (99.65% valid),
median 1454 mm, and 5th–95th percentiles 1419–1490 mm. Much of this range reflects
the tilted plane, so it must not be reported as sensor noise. Under the same
unverified same-pixel hypothesis, the depth-minus-RGB-plane median residual was
+0.72 mm, with 5th–95th percentiles -5.76 to +7.16 mm. Header delta was -8.90 ms.

These two views show good RGB/depth distance consistency near 0.92 m and 1.45 m,
but do not independently establish millimetre accuracy, depth intrinsics, complete
registration, or exposure synchronization. A small outline mismatch and invalid
depth bands remain visible. Next inspect left/right off-centre views at roughly
the farther distance to assess whether the residual changes across the image.
No pixel shift, scale correction, CameraInfo replacement, timing adjustment or
motor command was applied. Capture health was 71.9 C, throttled=0x0.

### Left-side stationary-board pair (2026-09-18)

Saved privately as `/home/robot/roam-e-rgbd-alignment-20260918-board-left-01`.
All 49 corners were detected near the left side of RGB (internal-corner bounds
x=75.75–174.98, y=198.57–291.69 px). RGB pose reprojection RMS was 0.304 px;
board-center Z was 1.3321 m with about 10.6 degrees tilt. The eroded interior
contained 7,858 depth pixels, all valid, with median 1357 mm, MAD 4 mm and
5th–95th percentiles 1344–1368 mm. Depth-minus-RGB header delta was +1.82 ms.

Under the same **unverified same-pixel-registration hypothesis**, the median
depth-minus-RGB-plane residual was +25.01 mm (5th–95th percentiles +16.79 to
+32.58 mm), noticeably larger than the two earlier centered views. This is a
cross-modal consistency discrepancy, not proof of a 25 mm depth-sensor error:
RGB edge calibration, registration, depth geometry, board dimensions/flatness
and pose estimation can all affect it. A low RGB reprojection RMS does not
establish independent metric accuracy. No tape distance was supplied.

The board is fully visible, but its depth outline includes invalid-depth bands;
yellow overlay boundaries must not be treated as measured physical edges.
Next collect the right-side view at a comparable range with the camera/cable
fixed and the board stationary, fully visible and flat. No calibration, camera
parameters, transforms or timing offsets were changed; SLAM and motors remain
off. Capture health was 73.6 C, throttled=0x0. Images remain outside Git.

### Right-side stationary-board pair and comparison (2026-09-18)

Saved privately as `/home/robot/roam-e-rgbd-alignment-20260918-board-right-01`.
The board was fully visible with all 49 corners detected (internal-corner bounds
x=425.47–527.48, y=213.26–310.61 px). RGB pose reprojection RMS was 0.356 px;
board-center Z was 1.2770 m, tilted about 16.0 degrees. The eroded interior
contained 8,436 depth pixels, all valid, with median 1261 mm, MAD 15 mm and
5th–95th percentiles 1232–1292 mm. The depth spread includes the tilted plane
and must not be interpreted as pure sensor noise. Header delta was +1.39 ms,
which still does not establish exposure synchronization.

The depth-minus-RGB-plane median residual under the same unverified registration
hypothesis was -14.85 mm, with 5th–95th percentiles -21.00 to -9.69 mm.
Summary of the four single-pair observations:

| View | RGB board-center Z | Board tilt | Median depth-minus-RGB-plane residual |
| --- | --- | --- | --- |
| Center, near | 0.916 m | 6.3 degrees | -2.39 mm |
| Center, farther | 1.454 m | 21.9 degrees | +0.72 mm |
| Left | 1.332 m | 10.6 degrees | +25.01 mm |
| Right | 1.277 m | 16.0 degrees | -14.85 mm |

The off-centre residuals differ in sign and magnitude. These observations do not
support applying one constant additive depth correction, but they do not isolate
a cause: position, range and board orientation changed together, and each view
contains only one pair. The calculation already accounts for the RGB-estimated
plane tilt, but its accuracy still depends on RGB intrinsics, board geometry,
pose estimation and actual RGB-depth registration. Invalid-depth bands remain
around parts of the outline; they are not measured physical edges.

Next review the saved pairs and camera geometry/registration path offline before
choosing any additional controlled captures or calibration changes. Do not fit
a guessed pixel shift, overwrite depth CameraInfo, or start RGB-D SLAM from this
comparison alone. No camera settings, transforms or timing offsets were changed;
motors remain off. Capture health was 75.2 C, throttled=0x0. All original images
remain private, outside Git.
