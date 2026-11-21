# Astra Pro camera setup

Roam-E carries an Orbbec Astra Pro on a Raspberry Pi 5 running Ubuntu 24.04
(ARM64) and ROS 2 Jazzy. The camera appears on USB as two separate devices,
and each one is read by a different driver:

| Stream | USB ID | Driver | Output |
|---|---|---|---|
| Depth and IR | `2bc5:0403` | `openni2_camera` with Orbbec's legacy OpenNI2 SDK | 640×480 depth, 16-bit millimetres, ~30 Hz |
| Colour | `2bc5:0502` (UVC) | `mjpeg_camera_node`, direct libuvc capture | 640×480 MJPEG, ~25 Hz captured |

`ros2 launch roam_e astra_camera.launch.py` starts both drivers, the colour
rectifier, a depth preview, the depth obstacle cloud used by Nav2 and,
by default, a read-only Foxglove bridge. It does not start motors or
navigation.

Related pages: [RGB calibration](rgb_calibration.md),
[depth-to-colour registration](depth_registration.md) and the
[on-board registration test](onboard_registration_test.md). The tools in
[`scripts/camera/`](../../scripts/camera/README.md) save captured images and
reports to a new directory outside the repository (`<output-dir>` in the
commands); no camera images are committed.

## Install

With the ROS 2 apt repository configured:

```bash
sudo apt update
sudo apt install -y \
  v4l-utils libusb-1.0-0 libusb-1.0-0-dev libuvc-dev \
  python3-numpy python3-opencv python3-yaml \
  ros-jazzy-openni2-camera ros-jazzy-v4l2-camera ros-jazzy-image-proc \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-compressed-depth-image-transport \
  ros-jazzy-foxglove-bridge
sudo usermod -aG video "$USER"
```

Log out and back in; `id -nG` must list `video`. `v4l-utils` and `v4l2_camera`
are only needed for diagnostics, because the colour node calls libuvc
directly. [`scripts/setup/bootstrap_pi5.sh`](../../scripts/setup/bootstrap_pi5.sh)
installs the declared dependencies through rosdep, adds the `video` group and
installs the two USB rules below.

Tested versions: libuvc 0.0.7, libusb 1.0.27, OpenCV 4.6.0, NumPy 1.26.4,
openni2_camera 2.2.2, v4l2_camera 0.7.1, image_proc 5.0.12, image transports 4.0.7.

### USB access rules

One rule per USB device lets members of `video` open the camera without root.
Depth, in `/etc/udev/rules.d/56-orbbec.rules`:

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0403", MODE:="0660", GROUP:="video"
```

Colour, in `/etc/udev/rules.d/57-roam-astra-rgb.rules`:

```text
SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0502", MODE:="0660", GROUP:="video"
```

Run `sudo udevadm control --reload-rules`, then unplug and reconnect the
camera: reloading the rules does not change the permissions of a device that
is already connected. Do not run the camera stack as root.

### Legacy OpenNI2 SDK

Ubuntu's OpenNI2 package does not detect the Astra Pro's depth device. Depth
needs Orbbec's legacy ARM64 SDK, unpacked at
`~/OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin` (its core library reports OpenNI
2.3.0 build 63). The vendor binaries are not part of this repository. Use the
libraries in `Samples/Bin`; the bundle's `Redist` binaries differ and were not
tested. Nothing is flashed to the camera.

In `Samples/Bin/OpenNI2/Drivers/orbbec.ini`, keep `[Device] UsbInterface=2`
and `[Depth] Resolution=1`, `InputFormat=3`. The ROS wrapper links against
`libOpenNI2.so.0`, so add that name:

```bash
cd ~/OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin
ln -sfn libOpenNI2.so libOpenNI2.so.0
```

The launch file prepends this directory to `LD_LIBRARY_PATH` and sets
`OPENNI2_DRIVERS_PATH` for the camera processes only; system libraries are not
replaced. To use another location, pass `openni2_sdk_dir:=/path/to/Samples/Bin`
or set `ROAM_E_OPENNI2_SDK_DIR`.

### Power and cabling

The camera adds a noticeable load on the Pi's 5 V supply. During bring-up a
marginal supply made the Pi freeze under camera load, and a disturbed USB
cable made the camera disconnect and re-enumerate. Use a supply rated for the
full load and secure the cable.

## Build, launch and view

On the Pi:

```bash
cd ~/Roam-E
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roam_e --symlink-install
source install/setup.bash
ros2 launch roam_e astra_camera.launch.py
```

The Foxglove bridge listens on the Pi's loopback interface only. On the
computer running Foxglove, keep a port forward open and connect to
`ws://localhost:8765`:

```bash
ssh -N -L 8765:127.0.0.1:8765 <user>@<pi-host>
```

When another launch already runs its own Foxglove bridge (robot bringup,
navigation), start the camera with `use_foxglove:=false`.

| Topic | Content |
|---|---|
| `/camera/color/image_raw/compressed` | Camera JPEG passed through without re-encoding (≤5 Hz) |
| `/camera/color/image_raw` | Decoded `rgb8`, produced only while subscribed (≤2 Hz by default) |
| `/camera/color/camera_info` | Measured colour calibration; one message per published frame, same stamp |
| `/camera/color/image_rect` (and `/compressed`) | Rectified colour from `image_proc`, only while subscribed. Leave Foxglove's **Rectify** option off for it |
| `/camera/depth_raw/image` | Depth in millimetres (`16UC1`), registered to the colour camera on board |
| `/camera/depth_raw/camera_info` | The driver's generic colour model, not a measured calibration |
| `/camera/depth_preview/compressed` | Colourised JPEG for viewing only (≤10 Hz). Black is invalid depth, not free space |
| `/camera/obstacle_points` | Decimated XYZ cloud (0.45–3.0 m, ≤5 Hz) for Nav2's voxel layer, only while subscribed |

Use `/camera/depth_raw/image` for any depth processing; the preview is not
metric. The camera-only launch publishes no `map` frame. Only one process can
own the camera: stop a launch with Ctrl+C before starting another, and do not
open the colour device with V4L2 or OpenCV tools while the node runs.

### Launch arguments

Node parameters are read at startup; restart the launch to change them.

| Argument | Default | Effect |
|---|---|---|
| `openni2_sdk_dir` | `$ROAM_E_OPENNI2_SDK_DIR`, else `~/OpenNI-Linux-Arm64-2.3.0.65/Samples/Bin` | Legacy SDK location |
| `start_depth`, `start_color` | `true` | Start each stream |
| `use_foxglove`, `foxglove_port` | `true`, `8765` | Loopback-only, read-only Foxglove bridge |
| `color_width`, `color_height` | `640`, `480` | Must match the calibration file; 640×480 is the only tested mode |
| `color_preview_fps` | `5.0` | Rate cap for the JPEG passthrough |
| `color_decode_fps` | `2.0` | Rate cap for decoded and rectified colour; `0` disables decoding. RTAB-Map mapping uses `10.0` |
| `color_calibration_file` | `config/astra_rgb_640x480.yaml` | Colour intrinsics; an empty string publishes no colour CameraInfo |
| `rectify_color` | `true` | Start the on-demand rectifier |

## How capture works

**Colour.** Through V4L2, VGA colour repeatedly stalled while VGA depth was
streaming. The colour node therefore opens the UVC device by its USB ID
through libuvc and requests an 800-byte isochronous payload instead of the
1600 bytes the camera negotiates by default. With that reservation the
camera's native MJPEG stream and full-resolution depth run together. This is a
per-stream USB setting: no firmware change, depth reduction or image resize.

A worker thread drains the stream continuously and keeps only the latest
frame. JPEGs are published without decoding; incomplete JPEGs are skipped. If
no frame arrives for 5 seconds the node exits instead of republishing stale
data. The capture runs at a requested 30 FPS (about 25 FPS observed); lowering
the preview rate does not reduce USB traffic. Timestamps are host receipt
times, not exposure times.

**Colour calibration.** The node loads `color_calibration_file` and rejects
malformed matrices, non-finite values, models other than `plumb_bob` and
resolution mismatches before it opens the USB device. It publishes one
CameraInfo for every frame it publishes on either colour topic, with that
frame's stamp and `camera_rgb_optical_frame`. K and D describe the raw lens;
the identity R and P describe the monocular rectified image, not the
transform between the colour and depth cameras. `image_proc/rectify_node`
produces `/camera/color/image_rect` only while something subscribes; it
decodes, remaps and re-encodes, and may leave thin black borders where the
correction samples outside the source image. Disable it with
`rectify_color:=false`; for uncalibrated colour also pass
`color_calibration_file:=''`. The calibration belongs to this colour camera at
640×480 only and must not be copied into the depth CameraInfo.

**Depth.** The driver runs with `depth_registration=true`, so the camera
warps depth into the colour camera's view on board. The `depth_raw` topic name
means integer millimetres, not unregistered geometry. Depth stamps use device
time with the driver's -33 ms offset. How well the registered depth lines up
with the colour image, and the correction now used, are covered in
[depth_registration.md](depth_registration.md).

Implementation: `src/roam_e/roam_e/mjpeg_camera_node.py`,
`src/roam_e/roam_e/uvc_capture.py`, `src/roam_e/roam_e/rgb_calibration.py` and
`src/roam_e/launch/astra_camera.launch.py`.

## Bench results

Bounded runs on the Pi with the camera-only launch, September 2026:

| Check | Result |
|---|---|
| Simultaneous VGA colour and depth, 180 s | 4495 native JPEG frames captured, 0 invalid; 816 JPEG previews, 337 decoded colour frames and 1536 depth previews published; decoded colour and raw depth both 640×480 |
| Input timing, [`check_rgbd_inputs.py`](../../scripts/camera/check_rgbd_inputs.py) | JPEG preview 4.53 Hz, decoded colour 1.82 Hz, depth 29.12 Hz; monotonic stamps; all 50 colour frames had a depth stamp within 20 ms; median receipt age 15.3 ms (colour) and 38.3 ms (depth) |
| Rectification, [`check_rgb_rectification.py`](../../scripts/camera/check_rgb_rectification.py), 30 s | Depth 28.6 Hz; 49 rectified JPEGs (1.68 Hz); 36 raw/rectified/CameraInfo triples with identical headers; rectified pixels match an independent OpenCV remap to within interpolation rounding; median age of rectified frames on the Pi 62 ms (max 72 ms) |

Receipt ages are measured on the Pi and do not include Foxglove's network
latency. Rates are what a best-effort subscriber received, not capture rates.

## Known limitations

- Colour timestamps are host receipt times and depth stamps are shifted by a
  fixed offset; header pairing is not exposure synchronisation.
- After a software-only restart of the camera nodes, depth once showed heavy
  packet corruption (about 2 FPS); a full stop and restart of the launch
  cleared it.
- The legacy OpenNI component can crash on shutdown (exit code -11). Check that
  every camera process has exited before relaunching.
- The bench runs above last minutes, not hours; they are not a long-duration
  reliability test.
