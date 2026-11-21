# Camera calibration and registration tools

Tools for the Astra Pro's colour and depth cameras. Methods and results are in
[`docs/camera/`](../../docs/camera/): [setup](../../docs/camera/setup.md),
[RGB calibration](../../docs/camera/rgb_calibration.md),
[depth-to-colour registration](../../docs/camera/depth_registration.md) and the
[on-board registration test](../../docs/camera/onboard_registration_test.md).

Live tools run on the Pi with the workspace sourced. The Python scripts never
open the USB device, change camera parameters, publish TF or command the
motors; the two diagnostic launch files start their own camera drivers, so
stop `astra_camera.launch.py` first. Tools that save data take a new output
directory outside the repository.

## RGB calibration

| File | What it does |
|---|---|
| `check_calibration_target.py` | Reports whether the 7×7 chessboard is detected in the live colour stream; saves nothing. Holds the shared corner detector |
| `capture_rgb_calibration.py` | Records distinct, sharp, still chessboard views and their corners for intrinsic calibration |
| `check_rgb_rectification.py` | Checks raw/rectified/CameraInfo stamps and compares `image_rect` with an independent OpenCV remap |

## RGB-IR calibration

| File | What it does |
|---|---|
| `rgb_ir_capture.launch.py` | Camera session for colour + IR capture, with depth and its preview off |
| `capture_rgb_ir_calibration.py` | Records still colour + native IR chessboard pairs with matched corner ordering |
| `fit_rgb_ir_calibration.py` | Fits IR intrinsics and the colour-to-IR transform (separate or RGB-anchored), with leave-one-view-out validation. Offline |
| `validate_saved_rgb_ir.py` | Tests a fitted model on a pair that was not in the fit and writes the candidate file. Offline |

## Depth registration checks

| File | What it does |
|---|---|
| `check_rgbd_inputs.py` | Reports colour and depth rates, header ages, nearest-pair deltas and CameraInfo; saves nothing |
| `inspect_rgbd_pair.py` | Saves one colour + registered depth pair with depth-edge overlays |
| `analyze_board_pair.py` | Compares registered depth with the board plane estimated from colour, at the same pixel. Offline |
| `native_depth_check.launch.py` | Camera session with on-board registration off (native depth and IR) |
| `capture_native_depth.py` | Confirms registration is off, then saves three colour + IR + native depth samples |
| `analyze_native_depth.py` | Compares native depth with the board plane predicted by the RGB-IR model, under two pixel-offset hypotheses. Offline |
| [`measure_depth_color_offset.py`](../../tools/calibration/measure_depth_color_offset.py) | In `tools/calibration/`: measures the depth-to-colour pixel shift used on the robot |

## On-board registration test

Here `firmware` means the camera's on-board registration, not [`firmware/`](../../firmware/).

| File | What it does |
|---|---|
| `firmware_rgbd_math.py` | Joint rectification, depth decoding, header pairing and point-cloud packing |
| `firmware_rgbd_test.py` | Adapter node publishing a rectified RGB-D pair and coloured cloud under `/rgbd_test` |
| `firmware_rgbd_test.launch.py` | Starts the adapter, a loopback Foxglove bridge and optional RTAB-Map odometry |
| `check_firmware_rgbd.py` | Records rates, metadata consistency and odometry statistics for a bounded run |

## Tests

```bash
cd scripts/camera && python3 -m pytest
```

They need NumPy and OpenCV, no camera; `test_check_rgb_rectification.py` and
`test_inspect_rgbd_pair.py` also need ROS 2 sourced (`rclpy`, `sensor_msgs`).
