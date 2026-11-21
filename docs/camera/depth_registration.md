# Depth-to-colour registration

RTAB-Map gives each colour feature the depth of the pixel under it, so the
depth image has to line up with the colour image pixel for pixel. On the Astra
Pro the two images come from different sensors, about 36 mm apart according
to the stereo fit below, through different drivers. This report covers three approaches evaluated on bench data
in September 2026 and the correction deployed in October 2026.

**Result.** The robot uses the camera's on-board depth registration plus a
fixed shift of the depth image by 7 px left and 1 px up, measured by aligning
depth edges with colour edges. The residual offset was roughly constant from
0.7 to 2.4 m, which shows the on-board registration already removes the
depth-dependent parallax. A full RGB–IR stereo calibration was fitted and
validated offline but is not used at runtime.

## Background: what the driver publishes

The camera launch runs `openni2_camera` 2.2.2 with `depth_registration=true`.
In this mode the camera warps depth into its own model of the colour camera,
and the driver:

- publishes `/camera/depth_raw/image` (16-bit millimetres) in
  `camera_rgb_optical_frame`. The topic name refers to integer depth, not to
  unregistered geometry;
- publishes, with no calibration URL set, a generic colour model as the depth
  CameraInfo: fx = fy = 570.342, cx = 319.5, cy = 239.5, no distortion. The
  measured colour model is fx = 580.354, fy = 574.668, cx = 320.607,
  cy = 249.588 with nonzero distortion ([rgb_calibration.md](rgb_calibration.md)).
  Rewriting the CameraInfo would not change the on-board pixel warp;
- stamps depth with device time minus 33 ms. Colour stamps are host receipt
  times, so header pairing is not exposure synchronisation.

With registration off, the driver builds depth CameraInfo from the IR
intrinsics and subtracts a generic principal-point offset of 5 px horizontally
and 4 px vertically at VGA. Registration cannot be switched at runtime in this
driver version, so native-depth tests use a separate launch
([`native_depth_check.launch.py`](../../scripts/camera/native_depth_check.launch.py)).
Sources: the driver's
[callbacks and CameraInfo construction](https://github.com/ros-drivers/openni2_camera/blob/2.2.2/openni2_camera/src/openni2_driver.cpp)
and [registration and mirroring calls](https://github.com/ros-drivers/openni2_camera/blob/2.2.2/openni2_camera/src/openni2_device.cpp).

All board experiments use the 7×7-inner-corner, 35 mm chessboard from the RGB
calibration, held still, with the measured colour intrinsics.

## Method 1: same-pixel consistency on a chessboard

**Method.** [`inspect_rgbd_pair.py`](../../scripts/camera/inspect_rgbd_pair.py)
saves one colour frame and the registered depth frame nearest in header time
(within 20 ms), each with its exact-stamp CameraInfo.
[`analyze_board_pair.py`](../../scripts/camera/analyze_board_pair.py) detects
the board in colour, solves its pose with `solvePnP`, intersects each pixel's
undistorted colour ray with the board plane to predict depth, and compares the
prediction with the measured depth **at the same pixel**. Only pixels inside
the inner-corner hull, eroded with a 7×7 kernel, are used, to stay clear of the
board's edges. The test asks whether on-board registration alone puts depth at the
right colour pixel.

**Results.** All 49 corners were found in every view; colour pose
reprojection RMS was 0.27–0.36 px.

| View | Board centre Z (from colour) | Tilt | Valid depth pixels in ROI | Median depth minus plane (5th–95th pct.) |
|---|---|---|---|---|
| Centre, near | 0.916 m | 6.3° | 16,116 (100 %) | -2.39 mm (-4.45 to +0.94) |
| Centre, far | 1.454 m | 21.9° | 5,681 (99.65 %) | +0.72 mm (-5.76 to +7.16) |
| Left | 1.332 m | 10.6° | 7,858 (100 %) | +25.01 mm (+16.79 to +32.58) |
| Right | 1.277 m | 16.0° | 8,436 (100 %) | -14.85 mm (-21.00 to -9.69) |

**Findings.** Near the image centre, depth and the colour-derived plane agree
within a few millimetres. Off-centre they disagree by +25 mm and -15 mm, with
opposite signs, so one constant depth correction cannot explain both. Position,
range and tilt changed together and each view is a single pair, so the cause
(registration error, colour calibration, board pose) is not isolated. The
depth outline of the board also differs from the colour outline, with
invalid-depth bands along parts of the edge.

## Method 2: RGB–IR stereo calibration

The IR camera sees the chessboard, so the colour–IR geometry can be calibrated
directly, and native (unregistered) depth, which lives in the IR image, can
then be mapped to colour in software.

**Data.** [`capture_rgb_ir_calibration.py`](../../scripts/camera/capture_rgb_ir_calibration.py),
run under [`rgb_ir_capture.launch.py`](../../scripts/camera/rgb_ir_capture.launch.py),
recorded nine colour + native 16-bit IR pairs. A pair is saved only if:

- both images show the full grid and have exact-stamp CameraInfo, with
  headers no more than 25 ms apart;
- both grids move less than 1.5 px RMS between consecutive checks;
- at least 4 s have passed since the last save, and the colour grid differs
  by at least 12 px RMS from every saved view;
- the IR edge width is at most 7 px and at most 35 % of the corner spacing.
  The IR projector's dot pattern widens edges (5–6 px against 1.5–1.8 px in
  colour), so the 3 px colour sharpness limit is relaxed for IR.

The IR grid's ordering is matched to the colour grid by comparing centred
corner shapes under all eight square symmetries; a pair is rejected if the
best match is above 12 px RMS or less than 8 px better than the runner-up.
Board poses spanned 0.73–0.90 m and 3.4–18.7° of tilt, mostly face-on.

**Fitting.** [`fit_rgb_ir_calibration.py`](../../scripts/camera/fit_rgb_ir_calibration.py)
holds the colour intrinsics fixed and compares two methods, each with three IR
distortion models (k3 = 0 throughout):

1. **Separate:** calibrate the IR camera from its nine planar views
   (`calibrateCameraExtended`), then fit the colour-to-IR transform with
   `stereoCalibrate` and both intrinsics fixed.
2. **RGB-anchored:** solve each board pose from the colour image, express all
   441 corners in the colour camera frame, and fit IR intrinsics plus one rigid
   transform to those 3D–2D correspondences in a single non-planar calibration.
   This conditions on the colour poses and does not propagate their
   uncertainty; it is not a full bundle adjustment.

Both start from the driver's IR matrix and use every view; none is dropped for
a high residual. Validation is leave-one-view-out: refit without a pair, solve
the held-out board pose from its colour corners only, and predict its IR
corners.

**Results.**

| Method | IR distortion | Held-out colour→IR RMS | Worst held-out view | Max leave-one-out translation change |
|---|---|---:|---:|---:|
| Separate | k1 | 1.390 px | 1.940 px | 44.66 mm |
| Separate | k1, k2 | 1.128 px | 1.564 px | 42.68 mm |
| Separate | k1, k2, p1, p2 | 1.104 px | 1.578 px | 26.36 mm |
| RGB-anchored | k1 | 0.691 px | 0.924 px | 8.34 mm |
| RGB-anchored | k1, k2 | 0.679 px | 0.933 px | 7.95 mm |
| RGB-anchored | k1, k2, p1, p2 | **0.587 px** | **0.729 px** | **4.71 mm** |

For the selected model (last row): training IR RMS 0.541 px, reverse held-out
IR→colour RMS 0.520 px, maximum leave-one-out rotation change 0.346°, focal
length change 0.61 %. In every fold the radial distortion stays monotonic out
to 5 % beyond the sensor-corner radius.
[`validate_saved_rgb_ir.py`](../../scripts/camera/validate_saved_rgb_ir.py)
then tested it on an earlier pair that was in none of the fits (board at
0.821 m, 20.4° tilt): colour→IR RMS 0.579 px (95th percentile 1.002 px,
maximum 1.179 px), IR→colour 0.564 px.

Fitted model, for column vectors in metres:

```text
X_ir = R * X_rgb + t

IR K = [[579.110626,   0,        320.121840],
        [  0,       573.905240, 240.937022],
        [  0,         0,          1      ]]
IR D = [-0.10703688, 0.12795724, -0.00369170, 0.00084440, 0]

R = [[ 0.999959354,  0.000649767, -0.008992672],
     [-0.000636042,  0.999998629,  0.001528986],
     [ 0.008993653, -0.001523204,  0.999958396]]
t = [0.035632937, -0.000224038, -0.003887569] m
```

The estimated optical-centre separation is 35.85 mm. The IR optical centre in
colour coordinates is `-R.T @ t`, not `t`, so this transform cannot be pasted
into a ROS parent→child TF as is.

**Findings.** Separate IR calibration is poorly constrained by mostly frontal
views: dropping a single view moves the translation by up to 45 mm. Anchoring
on the colour calibration cuts that to under 5 mm, but inherits any error in
the colour intrinsics and board poses. The leave-one-out spread is a
sensitivity measure, not a confidence interval.

## Method 3: native depth against the stereo model

**Method.** With registration off,
[`capture_native_depth.py`](../../scripts/camera/capture_native_depth.py)
first confirms through the driver's parameter service that
`depth_registration` is false, then saves three colour + IR + native-depth
samples of a still board within 25 s, headers within 25 ms of each other.
[`analyze_native_depth.py`](../../scripts/camera/analyze_native_depth.py)
holds the stereo model fixed (nothing is refitted): it maps the colour board
pose into the IR camera, predicts the IR corners, predicts the board-plane
depth for each IR pixel inside the board (hull eroded with a 9×9 kernel), and compares
it with native depth under two hypotheses for how depth pixels relate to IR
pixels: no offset, or the driver's generic (-5, -4) px offset.

The standard detector failed on these images; `findChessboardCornersSB` with
`NORMALIZE_IMAGE | EXHAUSTIVE | ACCURACY` on the full frame found all 49
corners in every colour and IR image, without cropping or manual placement. IR
is only contrast-stretched between its 1st and 99th percentiles. That fallback
is now part of the analysis script.

**Results.** The board was tilted 29.5–29.8°, steeper than any training view.
Colour pose RMS was 0.34–0.38 px and the stereo model predicted the IR corners
within **0.686–0.718 px RMS**, an out-of-sample check at a new angle. Native
depth was 99.6–99.8 % valid in the board region.

| Sample | Median depth minus predicted plane, no offset | With (-5, -4) px offset |
|---|---:|---:|
| 1 | -8.51 mm | -11.30 mm |
| 2 | -8.92 mm | -11.82 mm |
| 3 | -10.20 mm | -13.06 mm |

Replacing the IR distortion with zero changed these medians by less than
0.01 mm in this central region. A registered pair at a similar pose (31.8°)
gave -4.57 mm under the same-pixel hypothesis of Method 1.

**Findings.** A single plane at one pose cannot separate a pixel offset from a
constant depth bias: both hypotheses leave a residual of about -9 to -13 mm,
and three repetitions of one pose are not independent tests. The analysis also
includes a software registration (back-project native depth through the IR
model, transform with `R.T @ (X_ir - t)`, project into colour with a nearest-Z
buffer, no hole filling), used only for diagnostic overlays.

## Deployed correction: on-board registration plus a measured shift

The [on-board registration test](onboard_registration_test.md) showed that
registered depth, described by the colour calibration, already supports
RTAB-Map RGB-D odometry. What remained was a small 2D misalignment between the
registered depth and the rectified colour image, which hurt RTAB-Map's visual
matching.

**Measurement.**
[`tools/calibration/measure_depth_color_offset.py`](../../tools/calibration/measure_depth_color_offset.py)
subscribes to `/camera/color/image_rect` and `/camera/depth_raw/image` for a
few seconds while an object with clear outlines stands in front of a farther
background:

- colour edges: Canny on the blurred greyscale image, blurred again and
  averaged over all frames;
- depth edges: the near side of every jump of more than 8 cm between
  neighbouring pixels, dilated by one pixel, kept where present in more than
  half the frames;
- an exhaustive search over integer shifts within ±20 px in x and y for the
  one that maximises the overlap between depth edges and colour edges,
  ignoring a 40 px border. The ratio of the best score to the median score is
  reported as a contrast measure;
- the same search repeated per depth band (0.4–0.9, 0.9–1.4, 1.4–2.2,
  2.2–4.0 m) wherever a band has at least 300 edge pixels.

**Result.** The best shift was 7 px left and 1 px up, roughly constant from
0.7 to 2.4 m. Uncorrected parallax would not behave like that: with the
fitted 35.9 mm baseline and f ≈ 579 px, colour–IR disparity is about 30 px at
0.7 m and 9 px at 2.4 m. A constant residual means the on-board registration
already compensates parallax and leaves a fixed image-space offset, which a
constant shift can correct.

**Correction.**
[`depth_align_node`](../../src/roam_e/roam_e/depth_align_node.py) shifts each
registered depth image by `shift_x_px = -7`, `shift_y_px = -1` and publishes
`/camera/depth_aligned/image` with the original header. Pixels shifted in from
outside the frame become 0 (invalid), and the node does no work without a
subscriber. [`rtabmap_mapping.launch.py`](../../src/roam_e/launch/rtabmap_mapping.launch.py)
pairs the aligned depth with `/camera/color/image_rect` and
`/camera/color/camera_info`, so the colour calibration describes both images.

**Metric depth.** Separately, a flat-wall comparison against the lidar
([`measure_camera_wall.py`](../../tools/calibration/measure_camera_wall.py), at
0.70, 1.00 and 1.50 m) found registered depth = 0.991 × lidar distance + 3 mm,
within about 1 cm over that range.

## Conclusion

- The on-board registration handles parallax; a fixed 2D shift fixes the
  remaining offset between 0.7 and 2.4 m. The earlier caution against a
  constant shift was about uncorrected parallax, which the per-band
  measurement rules out for this setup.
- The shift costs one array copy per frame on the Pi, where software
  registration through the stereo model needs a per-pixel reprojection.
- The RGB–IR model (0.59 px held-out, 0.58 px on an unused pair) remains the
  most complete geometric description of the camera, but Method 3 could not
  pin down how native depth pixels map onto the IR image, so it is kept
  offline.

## Limitations

- One integer offset for the whole image. Registered depth is not undistorted
  with the colour model, so error that varies across the frame is not
  corrected. For scale, the colour lens distortion moves pixels by under 1 px
  near the centre and up to about 4 px at the image corners.
- The offset was checked from 0.7 to 2.4 m only.
- Colour and depth are paired by header time, not synchronised exposures;
  fast motion can still expose timing error.
- The board experiments used one board pose per condition and no independent
  range reference; the lidar comparison is the only metric depth check.
- All values apply to this camera unit at 640×480.

## Reproduce

Run on the Pi with the workspace sourced. Every capture script requires a new
output directory outside the repository.

```bash
# Method 1 (normal camera launch running)
python3 scripts/camera/check_rgbd_inputs.py --seconds 30
python3 scripts/camera/inspect_rgbd_pair.py --seconds 15 --output-dir <pair-dir>
python3 scripts/camera/analyze_board_pair.py <pair-dir>

# Method 2 (stop the normal camera launch first)
ros2 launch scripts/camera/rgb_ir_capture.launch.py
OPENBLAS_NUM_THREADS=1 python3 scripts/camera/capture_rgb_ir_calibration.py \
  --seconds 300 --max-pairs 16 --output-dir <rgb-ir-dir>
python3 scripts/camera/fit_rgb_ir_calibration.py <rgb-ir-dir>/session.json \
  --method rgb-anchored --output-dir <fit-dir>
python3 scripts/camera/validate_saved_rgb_ir.py <fit-dir>/report.json \
  <unused-sample-dir> --output-dir <candidate-dir>

# Method 3 (stop the normal camera launch first)
ros2 launch scripts/camera/native_depth_check.launch.py
python3 scripts/camera/capture_native_depth.py --output-dir <native-dir>
python3 scripts/camera/analyze_native_depth.py <native-dir> \
  <candidate-dir>/candidate.json --output-dir <analysis-dir>

# Deployed offset (normal camera launch running)
python3 tools/calibration/measure_depth_color_offset.py --seconds 6 --label box_1m
```

The diagnostic launch files start their own camera drivers; stop them and
restart `astra_camera.launch.py` afterwards. None of these steps command the
motors.
