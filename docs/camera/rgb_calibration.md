# RGB intrinsic calibration

The Astra Pro's colour stream is calibrated on its own, at 640×480, with a
chessboard. The result is
[`src/roam_e/config/astra_rgb_640x480.yaml`](../../src/roam_e/config/astra_rgb_640x480.yaml),
which the camera launch loads by default (see [setup](setup.md)).

| Parameter | Value |
|---|---|
| Model | `plumb_bob`, k3 fixed at 0 |
| fx, fy | 580.354, 574.668 px |
| cx, cy | 320.607, 249.588 px |
| k1, k2, p1, p2 | 0.08959, -0.23125, -0.00167, -0.00034 |
| Views | 17 distinct chessboard poses |
| Training reprojection RMS | 0.508 px |
| Leave-one-view-out RMS, pooled / worst view | 0.517 px / 0.866 px |

At the image corners the distortion correction moves pixels by about 4 px;
within the central third of the image it is under 1 px.

## Target

A flat 8×8-square chessboard on rigid backing, squares measured at
35 × 35 mm with a caliper, giving 7×7 inner corners (`square_m = 0.035`).
Flatness was not measured.

The 7×7 grid is symmetric, so a detector can return its corners in any of the
eight orderings of the square's symmetry group. The view-selection code
compares grids under all eight orderings; the RGB–IR work in
[depth_registration.md](depth_registration.md) resolves the ordering
explicitly before pairing corners across cameras.

## Corner detection

[`check_calibration_target.py`](../../scripts/camera/check_calibration_target.py)
holds the detector shared by every script:

1. `cv2.findChessboardCornersSB` with `CALIB_CB_NORMALIZE_IMAGE`.
2. If that misses, the classic `findChessboardCorners` (adaptive threshold,
   normalisation) followed by `cornerSubPix` in a 5×5 window.

A result is accepted only with all 49 corners and finite coordinates. The
fallback was added after the SB detector missed tilted views that the classic
detector found completely.

## View selection

[`capture_rgb_calibration.py`](../../scripts/camera/capture_rgb_calibration.py)
subscribes to `/camera/color/image_raw`, checks at most one frame per second
and saves a greyscale PNG and the detected corners only when every gate passes:

| Gate | Threshold | Purpose |
|---|---|---|
| Full grid | 49 corners | No partial boards |
| Corner spacing | ≥ 10 px between neighbours | Rejects boards too small or too oblique |
| Edge margin | Corners ≥ 8 px from the border | Keeps the subpixel window inside the image |
| Sharpness | `estimateChessboardSharpness` ≤ 3 px in both directions | Rejects motion blur and defocus |
| Stillness | Two consecutive detections within 2 px RMS, < 3 s apart | Rejects a moving board |
| Novelty | ≥ 15 px RMS from every saved view, under all 8 orderings | Avoids near-duplicate poses |

Sessions are bounded to 10–300 s and at most 60 views, and stop if no colour
frame arrives for 8 s. The output directory must not exist yet. The script
never solves or applies a calibration.

Capture procedure: keep the camera fixed and move the board. Hold each pose
for 5–6 s, and vary the board's image position (centre, edges, corners), size
(near and far) and tilt. Front-facing views alone leave the focal length and
distortion poorly constrained.

```bash
python3 scripts/camera/check_calibration_target.py --seconds 15   # detection only
python3 scripts/camera/capture_rgb_calibration.py \
  --output-dir <output-dir> --seconds 180 --max-views 40
```

## Fitting and model selection

The intrinsics were fitted offline with OpenCV's `calibrateCamera`; that
fitting step is not one of the scripts in this repository. Validation used
leave-one-view-out refits: for each view, refit without it, solve the held-out
board pose with `solvePnP` from its corners, and measure the reprojection RMS
of its 49 corners. The pooled figure is the square root of the mean squared
per-view RMS. A model was also rejected if the radial mapping
r_d = r(1 + k1 r² + k2 r⁴ + k3 r⁶) stopped increasing before the image-corner
radius, because the undistortion map then folds over near the edges.

The dataset was built in two rounds:

| Dataset | Corner coverage (convex hull of all corners) | Five-coefficient fit | Outcome |
|---|---|---|---|
| 13 views | ~45 % of the image | 0.502 px training RMS; three views held out of the fit reproject at 0.398, 0.895 and 0.651 px, and cx moves 10.4 px when they are left out | Rejected: radial mapping non-monotonic toward the image corners (minimum slope -1.47) |
| 17 views (one view added near each image corner) | ~55 % | 0.505 px; 1 of 17 leave-one-out refits non-monotonic near the corners | Replaced by k3 = 0 |
| 17 views, k3 = 0 | ~55 % | 0.508 px; 0.517 px pooled held-out, 0.866 px worst view; no non-monotonic refit | **Selected** |

Fixing k3 costs 0.003 px of training error and removes the unstable
extrapolation. For the selected coefficients, dr_d/dr stays above 0.80 out to
5 % beyond the image-corner radius.

## Runtime validation

The colour node publishes the calibration as CameraInfo with each frame's
stamp, and `image_proc/rectify_node` produces `/camera/color/image_rect` (see
[setup](setup.md#how-capture-works)).
[`check_rgb_rectification.py`](../../scripts/camera/check_rgb_rectification.py)
checks that pipeline end to end. It subscribes to the raw, rectified and
CameraInfo topics plus depth, and passes only if:

- at least 8 raw/rectified/CameraInfo triples have identical headers, covering
  ≥ 90 % of received rectified frames;
- the rectified pixels differ from an independent `cv2.remap` with the same
  calibration by a mean below 0.1 grey level, and the rectified image is
  measurably different from the raw one;
- at least 8 rectified JPEGs decode at 640×480, depth arrives above 15 Hz,
  and the calibration does not change during the check.

A 30 s run passed with 36 matched triples, 49 rectified JPEGs and depth at
28.6 Hz; pixel differences were interpolation rounding only.

```bash
python3 scripts/camera/check_rgb_rectification.py --seconds 30 [--output-dir <output-dir>]
```

## Limitations

- Fitting and leave-one-view-out testing use the same board and session. They
  measure internal consistency, not metric accuracy against an independent
  reference.
- The extreme image corners have no corner observations (hull coverage ~55 %),
  so distortion there is extrapolated.
- The calibration covers the colour camera at 640×480 only. It says nothing
  about the depth/IR camera or the colour–depth transform; those are covered
  in [depth_registration.md](depth_registration.md).
- Colour timestamps are host receipt times, not exposure times.
