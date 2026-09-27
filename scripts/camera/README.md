# Camera calibration target check

Preparation for RGB-D SLAM. A measured RGB-only calibration is now available for
bench rectification; **depth calibration, RGB-depth registration and timing are
not yet validated**. Historical checkpoints below describe what was true at each
capture stage; the latest integration status is at the end.

The physical board has 8 x 8 squares, each measured as **35 x 35 mm**.
Use **7 x 7 internal corners** and `square_m = 0.035`. The earlier photograph's
36.47 mm caliper reading was incorrectly positioned and must not be used.
The opened board is flat and the pattern is continuous.
Keep it supported flat; do not assume the nominal board dimensions prove accuracy.

## Detection only

With the existing camera-only `astra_camera.launch.py` already running:

```bash
cd ~/Roam-E
source /opt/ros/jazzy/setup.bash  # use setup.zsh from zsh
python3 scripts/camera/check_calibration_target.py --seconds 15
```

Remove the pieces. Show the complete board, sharp and without strong reflections,
in `/camera/color/image_raw/compressed` in Foxglove. For this first detection check,
hold the board/camera stationary. The script subscribes locally to the raw RGB
topic, activating the existing on-demand decoder, and checks at most 1 frame/sec
by default with one OpenCV thread. It does not open USB devices, change capture
settings, publish ROS messages, save images, or command motors. Exit 0 means at
least one full grid was detected; exit 2 means none was detected (including no
incoming images). This is not proof that calibration is accurate.

Test without a camera:

```bash
python3 -m unittest discover -s scripts/camera -p 'test_*.py' -v
```

Dependencies: ROS Jazzy `rclpy`/`sensor_msgs`, `python3-opencv`, `python3-numpy`.
These are already present on the current Pi. No new Linux packages are required.

## RGB-D work still remaining

Independently validate the measured RGB intrinsics, then validate depth intrinsics
and RGB/depth registration. The existing depth preview
is a visualization, not metric data. A detected RGB board does not establish
infrared contrast or exposure synchronization. The symmetric 7 x 7 grid also
requires explicit corner-correspondence/orientation handling for RGB/IR stereo
calibration; do not blindly match detector indices from different cameras.

The capture tools below do not apply calibration or modify the camera launch.

## Bounded RGB capture

`capture_rgb_calibration.py` uses the same 7 x 7 internal corners and 0.035 m
spacing at 640 x 480. It saves grayscale PNGs and a corner/quality manifest to a
**new** user-selected directory outside the repo. It never solves or applies
calibration, changes camera settings, or publishes commands. Keep this private
camera dataset out of Git; only validated calibration coefficients belong there.

```bash
python3 scripts/camera/capture_rgb_calibration.py \
  --output-dir /home/robot/roam-e-rgb-calibration-SESSION_NAME \
  --seconds 180 --max-views 40
```

Replace `SESSION_NAME` with a unique name; existing directories are refused.
The session is bounded to 5 minutes and 60 views maximum. It processes at most
one image per second with one OpenCV thread and aborts after 8 seconds without
RGB messages. A view requires a full grid,
at least 10 px adjacent-corner spacing, sharpness <=3 px edge transition in
both directions, two detections within 2 px RMS of each other, and at least
15 px RMS difference from previously saved views. The motion/novelty comparison
handles the square board's rotation/reflection ambiguity. These are practical
selection heuristics, not a guarantee of calibration quality.

Once recording starts, keep the camera/cable fixed and move the **flat, supported** board.
Hold each view still for 5-6 seconds: vary image position (center/left/right/top/
bottom), size (near/far), and perspective (look from multiple tilted directions).
Keep the full board sharp and in view; do not collect only front-facing shots.
Do not move the robot or strain the camera cable. Wait for the recording-start
message; detection-only tests do not save images.

After capture, inspect view coverage and quality before fitting or applying any
intrinsics. Saved counts and low reprojection errors alone do not validate RGB-D
registration. A printed/chessboard pattern is visible in IR intensity, if material
contrast permits, not as alternating squares in metric depth.

### Hardware checkpoint (2026-09-17)

The full measured board was detected in 13/13 stationary RGB checks. This is
detection evidence, not a completed calibration. The first diverse-view session
saved zero views: candidates failed sharpness/stillness filters, then both camera
USB interfaces disconnected/re-enumerated and the RGB driver exited. No cause
(cable, connector, or power) has been proven and no calibration was applied.
Prefer a fixed camera and move the board on rigid backing for the next attempt;
verify a sharp, steady baseline first. Seven synthetic/unit tests pass, and the
missing-stream watchdog was verified with the camera launch stopped.

### Recovery and capture checkpoint (2026-09-18)

After reboot, RGB and depth both streamed at 640 x 480 during a 40-second check,
with no new USB disconnects. The stationary board was detected in 15/15 checks
and passed both sharpness directions. One baseline image was saved privately at
`/home/robot/roam-e-rgb-calibration-20260918-baseline-01`.

The following 180-second capture saved **8** distinct views at
`/home/robot/roam-e-rgb-calibration-20260918-views-01`; the USB connection remained
up, throttled status was 0x0, and the final temperature was 69.2 C. Rejections:
45 stillness, 108 duplicate-view, 9 missing-grid, and 2 sharpness checks. These
counts describe checked frames, not failed camera frames. More out-of-plane
tilted views are needed before fitting and validating intrinsics. No calibration
has been fitted or applied, and no images have been added to the repository.

References: [OpenCV corner detector](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html),
[ROS calibration tutorial](https://github.com/ros-perception/image_pipeline/blob/jazzy/camera_calibration/doc/tutorial_mono.rst).

### Tilt-pass detector checkpoint (2026-09-18)

The 120-second `roam-e-rgb-calibration-20260918-tilts-01` session saved no accepted
views: all 116 checks missed the board with the SB detector. RGB messages kept
arriving, no new USB kernel errors were observed, and throttled status was 0x0.
The private `diagnostic.png` is only a troubleshooting frame, not an accepted
calibration view, and must not be added to the calibration manifest or Git.

On that same diagnostic frame, both normal and exhaustive SB detection failed,
while the classic detector found all 49 corners. `detect()` now tries the classic
adaptive-threshold detector when SB misses, with subpixel refinement, full-grid
and finite-coordinate checks. The capture sharpness/stillness/novelty filters
are unchanged. Earlier saved images remain intact; no calibration is applied.

Verification: all 10 unit/synthetic tests passed on the Pi. The combined detector
found 49 corners on the previously missed diagnostic frame and passed sharpness
(2.29/1.72 px). A later live frame detected the grid but failed the unchanged
sharpness limit in one direction (3.31 px versus 3.0 px); improve steadiness or
lighting before the next coordinated capture. No new accepted views were saved
by these diagnostic checks.

The next 90-second capture, `roam-e-rgb-calibration-20260918-tilts-02`, saved one
additional accepted view (sharpness 1.86/1.93 px). Of 86 checked frames, 44 failed
sharpness, 5 required stillness, and 36 repeated the accepted pose. There were no
new USB kernel events during the check; final temperature was 76.8 C and
throttled status 0x0. The earlier eight-view set remains intact. The diagnostic
image shows a lateral shift of the board on the chair; more varied out-of-plane
poses are still needed. Continue with single-pose, coordinated captures instead
of repeatedly collecting the same perspective. No calibration has been fitted
or applied; diagnostic PNGs remain excluded from accepted-view manifests.

### Single-pose review checkpoint (2026-09-18)

Accepted one view each in `right-edge-01`, `left-edge-02`, `top-edge-02`, and
`bottom-edge-01` (all under the `roam-e-rgb-calibration-20260918-` prefix).
`left-edge-01` missed detection; `top-edge-01` failed sharpness. Keep failed
sessions and diagnostic frames out of calibration input. Fourteen accepted
images exist across the successful sessions including the baseline; cross-session
15 px RMS deduplication leaves **13** distinct views, excluding only the baseline
duplicate without deleting any files.

A private offline review is saved at
`/home/robot/roam-e-rgb-calibration-20260918-review-02.json`.
The accepted corner convex hull covers about 45% of the 640 x 480 image; extreme
image corners lack observations. A tentative standard five-coefficient RGB fit
has 0.502 px training RMS. Three views held out from the intrinsics fit give
0.398, 0.895 and 0.651 px RMS after solvePnP; this is a consistency check, not
independent metric validation. The fitted principal point shifts 10.36 px in x
between the all-view and holdout fits.

**Do not apply this candidate:** radial distortion becomes non-monotonic when
extrapolated toward the image corners (minimum radial slope -1.47). Low training
error does not establish usable edge correction. Next capture observations near
all four image corners, with the entire target still visible and sharp, then
refit and compare model complexity/stability. No ROS calibration file or camera
parameters were changed, and depth/IR calibration remains unverified.

The last hardware check reported 70.8 C and `get_throttled=0x80000`: a historical
soft-temperature-limit event, without an active limit or undervoltage bit at that
instant. Plan cooling before sustained camera/SLAM load. The offline fit used
one OpenCV thread and completed in under one second; no USB or ROS access was
needed for the review.

### Corner coverage and candidate checkpoint (2026-09-18)

After a later Pi reboot, the camera initially disconnected from USB during
startup. That `upper-left-01` attempt saved no views. Stopped the camera-only
launch, and the camera was reconnected. A subsequent 30-second check
confirmed simultaneous 640 x 480 RGB and depth streams, with no new USB kernel
events. No motor/navigation launch was started.

Accepted one view each in `upper-left-02`, `upper-right-01`, `lower-right-01`, and
`lower-left-01`. This gives 18 accepted images including the duplicate baseline,
or **17 distinct views** after deduplication. Corner convex-hull coverage rose
from about 45% to 55% of the image. These figures describe detected inner corners,
not a requirement that every image pixel contain a calibration point.

Private reports:

- `/home/robot/roam-e-rgb-calibration-20260918-review-03.json`
- `/home/robot/roam-e-rgb-calibration-20260918-model-comparison-01.json`

The default five-coefficient fit has 0.505 px RMS; its all-view radial check now
passes, but one of 17 leave-one-view-out refits still becomes non-monotonic near
the image corners. Prefer the simpler candidate with `CALIB_FIX_K3` (k3 = 0):
0.508 px all-view RMS, 0.517 px pooled leave-one-view-out RMS, 0.866 px worst
held-out view, and no non-monotonic radial models in those 17 refits. Each held-out
pose is solved with solvePnP; this is internal consistency testing, not independent
metric ground truth or proof that the physical board is exactly planar.

Candidate RGB intrinsics: fx 580.354, fy 574.668, cx 320.607, cy 249.588 pixels.
The candidate YAML is private at
`/home/robot/roam-e-rgb-calibration-20260918-candidate.yaml`, **not** in
`~/.ros/camera_info` and **not** wired into the camera launch. No calibration has
been applied. This calibration belongs to the separate 640 x 480 RGB stream;
do not reuse it as the depth/IR camera calibration or claim registered RGB-D.

Next: test RGB rectification and CameraInfo publication with correct image
timestamps, then independently verify depth intrinsics, RGB-depth extrinsics,
registration and timing before enabling RGB-D SLAM. Keep private images out of
Git. The final health check was 69.7 C, throttled status 0x0, with no new USB
events in the observed four-minute window.

### RGB CameraInfo and rectification integration (2026-09-18)

The selected 17-view, k3-fixed candidate is copied without changing coefficients
to `src/roam_e/config/astra_rgb_640x480.yaml`. The camera-only launch now loads it
by default and starts the standard `image_proc/rectify_node`. No depth settings,
USB modes, firmware, navigation or motor controls are changed.

The native JPEG preview and decoded raw RGB share one calibrated CameraInfo
publisher. Metadata uses each published source frame's exact timestamp and
`camera_rgb_optical_frame`; different preview/decode schedules are handled without
duplicating metadata for a frame. These are still host receive timestamps, not
hardware exposure synchronization. Invalid or wrong-resolution calibration files
fail startup before opening USB; an empty path explicitly disables RGB metadata.

In Foxglove select `/camera/color/image_rect/compressed` with **Rectify OFF** for
already-corrected RGB. `/camera/color/image_raw/compressed` remains the original
native JPEG. Rectification is lazy and follows the decoded-RGB rate (default
2 FPS), not the native capture rate. `python3-yaml` and `ros-jazzy-image-proc` are
declared runtime dependencies and included in the manual package list in
`CAMERA_CAPTURE.md`.

Identity R in this file means monocular rectification, not co-located RGB/depth
sensors. Pixel-for-pixel agreement between RGB and depth has not been established.
See `CAMERA_CAPTURE.md` for configuration, rollback, and runtime topic details.

The live `check_rgb_rectification.py --seconds 30` check passed after a clean
camera restart: depth 28.6 Hz, 36 exact-stamp raw/rectified/CameraInfo triples,
and independent pixel-remap agreement. The first restart had depth packet
corruption and failed the combined check; this intermittent restart issue is
documented in `CAMERA_CAPTURE.md`, not hidden by the later pass. Private outputs
are under `/home/robot/roam-e-rgb-rectification-20260918-check-01` and `check-02`
(the latter abbreviated name uses the same full prefix). No dataset images are
added to Git. These runtime checks prove the RGB processing path, not metric
accuracy of the physical calibration or RGB-depth registration.

### RGB-depth baseline check (2026-09-18)

Added `check_rgbd_inputs.py` (bounded timing/CameraInfo report, no pixels saved)
and `inspect_rgbd_pair.py` (one private pair and diagnostic edge overlays, no
registration applied). See `CAMERA_CAPTURE.md` for commands and findings.
The latter rejects output paths inside the repository and never overwrites an
existing directory. Helper tests cover timing comparisons, depth endianness,
row padding, invalid-depth boundaries and integer-difference overflow.

Current depth registration is enabled, but the published depth CameraInfo uses
the driver's fallback color model rather than the newly measured RGB intrinsics.
Both messages label the same optical frame; this is not geometric validation.
Timestamp-neighbor matching passed for all 50 checked RGB frames, but an explicit
-33 ms depth offset and separate RGB timestamp path prevent us from calling the
exposures synchronized. The uncontrolled room overlay is inconclusive; next use
the stationary board upright, with a more distant background, to inspect physical
alignment. No depth/timing parameters, firmware, transforms or motors were changed.

### First controlled board check (2026-09-18)

`roam-e-rgbd-alignment-20260918-board-01` (private home-directory dataset) contains
a stationary RGB/depth pair with all 49 RGB corners detected. RGB pose estimates
board-center Z=916 mm, while the eroded board-interior depth median is 913 mm.
The -2.39 mm median depth-minus-RGB-plane residual assumes same-pixel alignment;
it is only a consistency check, not independent depth calibration. Perimeter
alignment still needs multiple ranges and consideration of invalid-depth bands.

`analyze_board_pair.py PRIVATE_PAIR_DIRECTORY` performs this offline analysis
using the existing 35 mm/7x7 target definition and the pair's RGB CameraInfo.
Its annotated ROI and JSON report remain beside the private input images, not
in Git. Plane-intersection unit tests cover front-parallel and tilted planes,
parallel rays and behind-camera intersections. See `CAMERA_CAPTURE.md` for
the detailed results and next-range capture instructions.

### Farther-range board check (2026-09-18)

The private `roam-e-rgbd-alignment-20260918-board-02` pair detected all 49 corners.
RGB estimated centre Z=1.4539 m and depth median was 1.454 m. The board was tilted
about 21.9 degrees; the plane-aware median residual was +0.72 mm, subject to the
same unverified registration hypothesis and RGB/board calibration limitations.
Do not treat the raw ROI depth spread as noise or the small residual as independent
metric accuracy. Outline mismatch/invalid bands remain; next inspect off-centre
left/right views before deciding how to handle registration. No camera parameters
or transforms were changed. Full figures and cable-recovery context are recorded
in `CAMERA_CAPTURE.md`; original images remain private and outside Git.

### Left-side board check (2026-09-18)

The private `roam-e-rgbd-alignment-20260918-board-left-01` pair detected all
49 corners, with RGB pose RMS 0.304 px, centre Z=1.3321 m and 10.6 degrees tilt.
The 7,858-pixel interior depth ROI was fully valid, with median 1357 mm.
The same-pixel plane comparison produced a +25.01 mm median residual, larger
than the centered views. This does not isolate a depth error from RGB edge
calibration, registration or board/pose effects; there is no independent tape
measurement. Invalid-depth perimeter bands also limit edge interpretation.
Next collect the right-side counterpart before choosing further geometric
checks. No settings or transforms were changed, and no SLAM/motor operation
was enabled. See `CAMERA_CAPTURE.md` for detailed metrics and caveats.

### Right-side board check and comparison (2026-09-18)

The private `roam-e-rgbd-alignment-20260918-board-right-01` pair detected all
49 corners, with RGB pose RMS 0.356 px, centre Z=1.2770 m and 16.0 degrees tilt.
All 8,436 interior depth pixels were valid; median depth was 1261 mm. The
same-pixel plane residual was -14.85 mm (5th–95th percentiles -21.00 to -9.69 mm),
versus +25.01 mm on the left and -2.39/+0.72 mm in the two centered views.
Position, range and orientation were not independently controlled, so the sign
change does not identify the source of the discrepancy or justify a correction.
Next review saved data and camera geometry/registration offline; no further
capture or calibration change is implied by this checkpoint. No settings or
transforms were changed, and SLAM/motor operation remains disabled. Full results
are in `CAMERA_CAPTURE.md`; original images remain outside Git.

## Automated paired RGB–IR recorder (2026-09-18)

`capture_rgb_ir_calibration.py` records native, stationary 640x480 pairs using
the existing 7x7-inner-corner, 35 mm-square board. It does not estimate/apply
calibration, open USB itself, publish images/TF, or command robot motion.
Output must be a new directory outside the repository. Gray RGB, native 16-bit
IR, normalized IR previews, exact-stamp CameraInfo, raw/corresponded corners,
quality measurements and a session manifest remain private. The existing RGB
calibration is unchanged.

Use a camera-only session with no other camera launch already running:

```bash
cd /home/robot/Roam-E
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch scripts/camera/rgb_ir_capture.launch.py
```

This uses the existing legacy SDK path and RGB driver. The loopback-only,
read-only Foxglove bridge exposes RGB previews, RGB CameraInfo and TF only.
Depth preview is deliberately absent. Excluding depth CameraInfo matters: a
Foxglove subscription to that metadata was observed to keep depth streaming
even without a depth-image subscriber. Do not start navigation/motor nodes.

In another sourced shell, after a successful dual-stream preflight and when
you are ready:

```bash
OPENBLAS_NUM_THREADS=1 python3 scripts/camera/capture_rgb_ir_calibration.py \
  --seconds 300 --max-pairs 16 \
  --output-dir /home/robot/roam-e-rgb-ir-calibration-UNIQUE_SESSION
```

The recorder requires both grids and exact-stamp metadata, header separation
at most 25 ms, two stable checked views, at least four seconds between saved
pairs and at least 12 px symmetry-invariant RGB grid RMS from previous views.
Near-header matching is **not** exposure synchronization. Keep board and camera
still during each capture; move/tilt the board between captures, not the camera.

The two square grids' orderings are matched across the eight D4 variants using
centered shape agreement. Large disagreement or ambiguous ordering is rejected.
This assumes roughly parallel images, as seen in the visibility test; retain
and inspect raw detections before fitting any stereo calibration. Projected IR
dots make the visible-light sharpness estimator report wider edges (about
5–6 px versus 1.5–1.8 px for RGB in the earlier sample). The IR-only guard allows
up to 7 px and 35% of minimum corner spacing, explicitly flags this exception,
and retains native images. This is not certification of calibration accuracy.

For the legacy driver, CameraInfo discovery must settle before Image connects;
on exit, CameraInfo must disconnect before Image. The recorder performs those
ordered operations and keeps ROS alive for cleanup on Ctrl-C/SIGTERM. Confirm
the driver logs IR.State=0 afterward; a force-kill cannot guarantee cleanup.

### Reboot-session preflight status

The recorder/helper suite passes with ROS sourced. A full camera launch after
the reboot had `libuvc start stream failed (-99)`. An RGB-only retry briefly
worked, then RGB stalled and depth packets became corrupt during an IR test.
The isolated RGB–IR launch recovered RGB. One IR startup attempt was prevented
by subscriber-discovery order; after that fix, some IR frames arrived, but USB
packet corruption persisted even with depth disabled. All four preflight
directories (`roam-e-rgb-ir-calibration-20260918-preflight-01` through `-04` in
the Pi home directory) contain **zero accepted pairs**. No calibration session
or fitted calibration should be claimed from these runs.

The camera launch was stopped for a physical USB reconnect, with the Pi left
running; the current power source is being confirmed. Resume only after
both streams are healthy. Do not loosen matching/quality checks to force a
dataset through transport failures. No calibration or firmware was changed.

### Successful reconnect and first paired dataset (2026-09-18)

After the physical camera USB reconnect, preflight `-05` delivered both streams
without packet-corruption messages during capture. The subsequent bounded
300-second run saved **nine** accepted RGB–IR pairs (the limit was 16). Originals,
exact-stamp metadata and detected corners are private on the Pi at
`/home/robot/roam-e-rgb-ir-calibration-20260918-session-01`. Do not add them to Git.

Each pair has 49 corresponding corners in 640x480 RGB and native 16-bit IR.
The offline review checked all image formats and exact-stamp CameraInfo and
visually inspected numbered corner correspondences. RGB board-pose estimates
using the existing candidate intrinsics give approximately 0.73–0.90 m board
origin depth, 3.4–18.7 degrees tilt and 0.34–0.53 px reprojection RMS. Header
separation is within 17 ms. These are dataset checks, **not** independently
measured range, a stereo-fit residual, or proof of exposure synchronization.

Position coverage includes left/centre/right and upper/lower regions, but most
accepted poses are nearly face-on. Treat this as an initial fitting dataset,
not a validated stereo calibration; assess conditioning and held-out views
before deciding whether additional tilted views are necessary. Rejected frames
were primarily missing grids or insufficient stillness. No detector thresholds
were loosened during this session and no new calibration was fitted/applied.

Native IR remained about 30 FPS and native RGB about 25 FPS during capture;
RGB decoding remained capped at 2 FPS. No new kernel USB errors were reported.
The Pi reached 78.5 C at the sampled end-of-capture check with `throttled=0x0`;
this is not a sustained thermal qualification. IR.State returned to 0 after
ordered subscriber cleanup. Packet warnings appeared only while the SDK was
tearing down the IR read thread. Stopping the camera launch then reproduced the
legacy OpenNI component's exit-code -11 shutdown crash; all device-owning
processes exited before restarting the normal camera-only RGB/depth launch.

### Offline RGB–IR candidate fit (2026-09-18)

The nine pairs now have reproducible offline fitting and validation results in
[RGB_IR_CALIBRATION_20260918.md](RGB_IR_CALIBRATION_20260918.md).
`fit_rgb_ir_calibration.py` compares separate IR-intrinsics/stereo fitting with
an explicitly conditional RGB-anchored fit, including leave-one-view-out checks.
`validate_saved_rgb_ir.py` checks a previously unused saved visibility pair.
Neither helper opens camera devices or imports/communicates through ROS.

The retained experimental RGB-anchored candidate achieves 0.587 px pooled
held-out RGB-to-IR RMS and 0.579 px on the earlier unused pair. Omitting a view
changes the estimated translation by up to 4.71 mm and rotation by 0.346 degrees;
these are sensitivity measurements, not physical accuracy bounds. The separate
IR calibration was substantially less stable. The candidate remains private,
unapplied and unapproved for deployment. Active RGB calibration, depth settings,
TF and firmware are unchanged. Native-depth/firmware-registration validation is
still required before claiming RGB-D alignment or SLAM readiness.

### Native-depth registration check (2026-09-18)

An initial camera-only native-depth comparison is now recorded in
[DEPTH_REGISTRATION_20260918.md](DEPTH_REGISTRATION_20260918.md). RGB, IR and
native depth were captured simultaneously and the normal RGB/depth launch
restored afterward. The RGB–IR candidate predicts the new, steeper board view
within 0.69–0.72 px, but native-depth plane residuals of roughly -9 to -13 mm
do not uniquely determine the depth/IR pixel offset or a depth bias. No new
calibration or correction was deployed.

`native_depth_check.launch.py` and `capture_native_depth.py` support bounded,
camera-only capture. `analyze_native_depth.py` evaluates saved data only and
includes an exhaustive full-frame chessboard detector fallback. That fallback
found all 49 corners in all six saved RGB/IR images without cropping or manual
placement after the standard detector failed. Private captures stay outside Git.

### Firmware-registered RGB-D baseline (2026-09-18)

Calibration investigation is paused in favor of a practical, isolated test of
the existing firmware-registered pipeline. See
[RGBD_FIRMWARE_TEST.md](RGBD_FIRMWARE_TEST.md) for code, assumptions, packages,
launch commands, Foxglove setup, and measured results. A colored cloud and
RTAB-Map visual odometry were produced with motors/navigation off and odometry
TF publication disabled. The observed 20-second tracking window had zero lost
messages; controlled motion accuracy and full SLAM are not yet validated.
This does not approve or deploy the private RGB–IR calibration candidate.
