# Offline RGB–IR calibration checkpoint — 2026-09-18

**Outcome:** an experimental RGB-anchored candidate was fitted and checked on
held-out images. It is **not applied or approved for deployment**. This work
does not establish native-depth geometry, RGB/depth registration or SLAM readiness.
The robot's active RGB calibration, camera settings, TF and firmware are unchanged.
No motors or navigation nodes were started. RGB and depth preview remained running.

## Data and methods

- Nine paired 640x480 observations; 49 corresponding inner corners per image.
- Flat 8x8-square chessboard; measured square pitch 35 mm.
- RGB board poses from the existing RGB candidate span roughly 0.73–0.90 m
  board-origin depth and 3.4–18.7 degrees tilt. These are estimates, not tape measurements.
- Existing RGB intrinsics, previously fitted from 17 RGB views, were held fixed.
- All nine views were used. No residual-based outlier or view deletion was performed.
- Headers within 25 ms and a stillness gate are not exposure synchronization.
- IR projection patterns/reflections affect corner detection. The recorded native
  IR images and the documented relaxed IR sharpness guard remain part of provenance.

Two methods were tested with three distortion models each (`k1`, `k1+k2`,
`k1+k2+p1+p2`; `k3=0` throughout):

1. **Separate intrinsics:** estimate IR intrinsics from the nine planar board
   views, then estimate the stereo transform with both cameras' intrinsics fixed.
2. **RGB-anchored:** solve the board pose separately in each RGB image, express
   all board corners in the RGB optical coordinate system, then jointly estimate
   IR intrinsics and one rigid RGB-to-IR transform from those 3D/2D correspondences.
   This is a conditional fit, **not full stereo bundle adjustment**: it treats
   RGB-derived board coordinates as exact and does not propagate their uncertainty.

The implementation uses OpenCV `calibrateCameraExtended`, `stereoCalibrate`
with `CALIB_FIX_INTRINSIC`, `solvePnP` and `projectPoints`. The non-planar
RGB-anchored fit supplies an initial intrinsic matrix, as required by
[OpenCV's calibration API](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html).
The driver-published IR matrix was used as an initialization/baseline only;
it is not known to be a factory calibration.

## Validation and sensitivity

For each leave-one-view-out fold, IR fitting and transform fitting exclude the
omitted pair. The held-out board pose is obtained from its **RGB corners only**;
the fitted transform and IR model then predict its IR corners. Reverse projection
uses an IR-only board pose and predicts RGB corners. These are pixel Euclidean
RMS errors, not millimetre depth errors. All views contain the same 49 points,
so pooled RMS is the square root of the mean of squared per-view RMS values.

| Method | IR model | Held-out RGB→IR RMS | Worst held-out view RMS | Max leave-one-out translation change |
|---|---|---:|---:|---:|
| Separate | k1 | 1.390 px | 1.940 px | 44.66 mm |
| Separate | k1+k2 | 1.128 px | 1.564 px | 42.68 mm |
| Separate | k1+k2+p1+p2 | 1.104 px | 1.578 px | 26.36 mm |
| RGB-anchored | k1 | 0.691 px | 0.924 px | 8.34 mm |
| RGB-anchored | k1+k2 | 0.679 px | 0.933 px | 7.95 mm |
| RGB-anchored | k1+k2+p1+p2 | **0.587 px** | **0.729 px** | **4.71 mm** |

The last model is retained for further validation, not deployment. Its training
IR projection RMS is 0.541 px; reverse held-out IR→RGB RMS is 0.520 px. Maximum
leave-one-out rotation change is 0.346 degrees and focal-length relative change
is 0.61%. Every fold passes a sampled positive radial-scale/radial-derivative
check out to the sensor-corner radius plus 5%. That check does not prove full
tangential distortion-map bijectivity or real-world accuracy.

The differing separate and anchored solutions demonstrate that IR-only geometry
is not firmly constrained by these mostly frontal views. Leave-one-out parameter
variation is a sensitivity diagnostic, **not a confidence interval**. The better
anchored result depends on the existing RGB calibration and its board-pose estimates.

### Earlier unused image pair

The retained candidate was also evaluated on the saved visibility capture from
earlier that day, which was **not included in any of the nine-view fits**:

- RGB→IR RMS **0.579 px**, 95th percentile 1.002 px, maximum 1.179 px.
- IR→RGB RMS **0.564 px**.
- Estimated board-origin depth 0.821 m, tilt 20.4 degrees.
- Full 49-corner correspondence verified; overlay inspected visually.

This is one separate capture at one range, not broad validation across distances,
image edges, motion or temperature. No parameters were tuned using this image.

## Retained candidate and frame convention

For column vectors in metres:

```text
X_ir = R_rgb_to_ir * X_rgb + t_rgb_to_ir

IR K = [[579.110626,   0,        320.121840],
        [  0,       573.905240, 240.937022],
        [  0,         0,          1      ]]
IR D = [-0.10703688, 0.12795724, -0.00369170, 0.00084440, 0]

R = [[ 0.999959354,  0.000649767, -0.008992672],
     [-0.000636042,  0.999998629,  0.001528986],
     [ 0.008993653, -0.001523204,  0.999958396]]
t = [0.035632937, -0.000224038, -0.003887569] m
```

Estimated optical-centre separation is 35.85 mm. This is **not** a physical
measurement. The IR optical centre expressed in RGB coordinates is `-R.T @ t`,
not `t`; do not paste this transform directly into a ROS parent/child TF command.
The existing driver's frame names and registration flag do not establish that its
depth pixels already use this model. No IR CameraInfo or static TF was changed.

## Artifacts (private on the Pi, outside Git)

Under `/home/robot/`:

- `roam-e-rgb-ir-calibration-20260918-session-01/`: original images and manifest.
- `roam-e-rgb-ir-calibration-20260918-offline-01/report.json`: separate-method fits.
- `roam-e-rgb-ir-calibration-20260918-offline-anchored-01/report.json`: anchored fits and all folds.
- `roam-e-rgb-ir-calibration-20260918-candidate-01/candidate.json`: full-precision candidate, marked unapplied/unapproved.
- Same candidate folder: `validation.json`, `validation_overlay.png`.
- `roam-e-rgb-ir-visibility-20260918-01/`: earlier unused validation sample.

The fit report records the training-manifest SHA-256. External validation records
the saved-image/metadata hashes. Private images and manifests are not committed.

## Reproduce without touching live cameras

Use **new** output-directory names. No additional Linux packages were installed;
these helpers use the Pi's existing Python, NumPy and OpenCV 4.6.0.

```bash
cd /home/robot/Roam-E
OPENBLAS_NUM_THREADS=1 nice -n 10 python3 scripts/camera/fit_rgb_ir_calibration.py \
  /home/robot/roam-e-rgb-ir-calibration-20260918-session-01/session.json \
  --method separate --output-dir /home/robot/NEW_SEPARATE_FIT
OPENBLAS_NUM_THREADS=1 nice -n 10 python3 scripts/camera/fit_rgb_ir_calibration.py \
  /home/robot/roam-e-rgb-ir-calibration-20260918-session-01/session.json \
  --method rgb-anchored --output-dir /home/robot/NEW_ANCHORED_FIT
OPENBLAS_NUM_THREADS=1 python3 scripts/camera/validate_saved_rgb_ir.py \
  /home/robot/NEW_ANCHORED_FIT/report.json \
  /home/robot/roam-e-rgb-ir-visibility-20260918-01 \
  --output-dir /home/robot/NEW_CANDIDATE_VALIDATION
```

Eight new synthetic/validation tests verify known-geometry recovery, transform
direction, held-out-data exclusion for both methods, metadata checks, output
safety and the radial guard. All **40 camera-helper tests** pass with ROS sourced.

## Next gate

Keep the candidate offline. Determine and validate how native depth pixels map
to the IR optical geometry, and how the current firmware registration transforms
them, before changing depth CameraInfo or attempting RGB-D SLAM. Do not apply
these IR parameters to already registered depth by assumption. New multi-range,
tilted validation views and RGB calibration uncertainty remain important if the
candidate is promoted. No further board holding was needed for this offline step.
