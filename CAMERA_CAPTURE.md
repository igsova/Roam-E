# Astra camera capture

## Linux setup (Ubuntu 24.04 ARM64 / ROS 2 Jazzy)

With the official ROS apt repository already configured, install the packages
used during bring-up:

```bash
sudo apt update
sudo apt install -y \
  v4l-utils libusb-1.0-0 libusb-1.0-0-dev libuvc-dev \
  python3-numpy python3-opencv \
  ros-jazzy-openni2-camera ros-jazzy-v4l2-camera \
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
openni2_camera 2.2.2, v4l2_camera 0.7.1, image transports 4.0.7.

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
`color_preview_fps`, `color_decode_fps` (zero disables raw decoding).
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
No fabricated RGB calibration is published. RGB calibration, depth/color
registration and synchronization must be verified before visual SLAM.
Changing resolution requires matching calibration.

On 2026-09-14, both VGA streams passed a 180-second simultaneous test with
816 JPEG preview messages, 337 decoded RGB messages, and 1536 depth previews.
JPEG decoding verified 640x480 RGB, and actual raw-depth messages reported
640x480. Native RGB capture recorded 4495 JPEG frames with zero invalid frames
at its 180-second report. This is a bounded bench test, not a long-duration
reliability or SLAM-calibration guarantee.
