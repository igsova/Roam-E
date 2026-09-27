# Native-depth registration check — 2026-09-18

Status: capture and initial offline consistency check completed; **no new
calibration, depth correction or TF deployed. Not a SLAM-readiness certificate.**

## What was established

- Installed `openni2_camera` is 2.2.2. Its standard launch sets
  `depth_registration=true`. Thus `/camera/depth_raw/image` means integer
  millimetre depth, **not necessarily native/unregistered geometry**.
- The registered-depth callback assigns the RGB frame and uses the OpenNI color
  CameraInfo. With no calibration URL this is a generic model, not the measured
  calibration of our separate UVC RGB camera. Changing CameraInfo alone would
  not change the firmware's pixel warp.
- When registration is off, the driver constructs depth CameraInfo from IR
  intrinsics with a default principal-point subtraction of 5 px horizontally
  and 4 px vertically at VGA. This generic assumption is not camera-specific
  verification. The driver also labels IR CameraInfo with the depth frame even
  though the IR Image has its own configured frame.
- The registration parameter is not dynamically settable in this version.
  The temporary diagnostic launch therefore started a separate camera-only
  session after gracefully stopping the normal one. Its log confirms firmware
  registration 0 and depth/IR mirroring 0. There was no firmware flash.

Primary source: exact installed upstream version's
[driver callbacks and CameraInfo construction](https://github.com/ros-drivers/openni2_camera/blob/2.2.2/openni2_camera/src/openni2_driver.cpp)
and [SDK registration and mirroring calls](https://github.com/ros-drivers/openni2_camera/blob/2.2.2/openni2_camera/src/openni2_device.cpp).

## Capture and detection

We presented the 7x7-inner-corner, 35 mm-square board. We saved one
firmware-registered RGB/depth pair, then three approximately stationary
RGB/IR/native-depth triples. All images were 640x480. IR and depth streamed
together; 229 IR, 227 depth and 15 decoded RGB messages were received during the
bounded native capture. Exact-stamp CameraInfo accompanies every saved image.
Header differences are within 15 ms; this does **not** establish exposure sync.

The initial standard detector failed on these images. The image was usable:
OpenCV `findChessboardCornersSB` with `NORMALIZE_IMAGE | EXHAUSTIVE | ACCURACY`
found all **49 corners in every RGB and IR sample**, at full frame, with no crop,
manual corner placement or smoothing. IR is only contrast-normalized using its
1st/99th percentiles. Broader cropped/smoothed exploratory variants also found
grids, but were not used for the reported geometry. Detection was visually
reviewed. The improved search is retained in `analyze_native_depth.py` and tested.

The failed analysis directory `analysis-01` was retained; successful results are
in `analysis-02`. No additional capture was needed to fix corner detection.

## Results and limits

The RGB–IR candidate was held fixed; nothing was refitted to these samples.
The new board angle was about 29.5–29.8 degrees, compared with mostly less than
19 degrees in the original fitting dataset. RGB pose reprojection RMS was
0.34–0.38 px. Candidate RGB-to-IR corner prediction RMS was **0.686–0.718 px**.
This is useful extra validation, not proof of absolute depth accuracy.

For an eroded interior board region, native depth validity was 99.6–99.8%.
The following are median **observed depth minus predicted board-plane Z**:

| Saved sample | IR-pixel model, no offset | Driver-style (-5, -4) pixel offset |
| --- | ---: | ---: |
| 1 | -8.51 mm | -11.30 mm |
| 2 | -8.92 mm | -11.82 mm |
| 3 | -10.20 mm | -13.06 mm |

These columns use the candidate IR distortion; testing zero distortion changed
the medians by less than 0.01 mm for this central board region. That insensitivity
does not validate either distortion model across the image.

The earlier firmware-registered pair had -4.57 mm median residual under a
same-pixel RGB-plane hypothesis, but was at a slightly different pose (31.8
degrees). It is not a controlled accuracy ranking against the native samples.

Neither the 5-by-4-pixel offset nor a constant depth bias is identifiable from
this single approximate pose. The board moved slightly between repetitions,
and board dimensions/flatness, RGB pose and prior calibration uncertainty all
affect residuals. Do **not** apply a +10 mm depth correction or zero the offset
just because one column is closer to zero. Three repetitions are not three
independent pose/range validation cases.

Offline software registration overlays were generated for both offset
hypotheses. They use `X_ir = R * X_rgb + t`, inverted as
`X_rgb = R.T * (X_ir - t)`, optical-axis-Z depth, nearest-pixel projection and a
nearest-Z buffer. Holes are not filled. These are explicitly labelled diagnostic
hypotheses, not live ROS outputs or an approved registration solution.

## Artifacts and reproduction

Private data remains outside Git, under `/home/robot/`:

- `roam-e-depth-registration-20260918-firmware-01`: original registered pair,
  `board_analysis.json` and board ROI review.
- `roam-e-depth-registration-20260918-native-01`: `capture.json`, three native
  RGB/IR/depth image sets and per-image metadata.
- `roam-e-depth-registration-20260918-detection-01`: offline detector review.
- `roam-e-depth-registration-20260918-analysis-02`: hypotheses, overlays and report.

Candidate input:
`/home/robot/roam-e-rgb-ir-calibration-20260918-candidate-01/candidate.json`.
No candidate parameters were installed into the camera launch or TF tree.

To analyze saved triples (use a new private output directory each time):

```bash
cd /home/robot/Roam-E
source /opt/ros/jazzy/setup.bash
OPENBLAS_NUM_THREADS=1 nice -n 10 python3 scripts/camera/analyze_native_depth.py \
  /home/robot/roam-e-depth-registration-20260918-native-01 \
  /home/robot/roam-e-rgb-ir-calibration-20260918-candidate-01/candidate.json \
  --output-dir /home/robot/NEW-native-depth-analysis
```

Use `setup.zsh` instead when running in zsh. ROS Python dependencies are needed
by reused helpers, but the analyzer does not create a ROS node, subscribe,
publish, open camera hardware or communicate through ROS.

For a future capture: stop the existing camera-only launch first, ensure no
other process owns the camera, source the workspace, and run
`ros2 launch scripts/camera/native_depth_check.launch.py`. Run
`python3 scripts/camera/capture_native_depth.py --output-dir /home/robot/NEW-capture`
in another sourced shell. The recorder checks registration is false and saves
three samples within 25 seconds; it does not require successful board detection
to save images. Analyze them before treating them as valid calibration data.
Stop the diagnostic launch and restore `ros2 launch roam_e astra_camera.launch.py`.
Neither launch should be combined with an active motor/navigation test.

## Restored operating state and next decision

The diagnostic launch was stopped and the normal camera-only launch restored.
A 12-second check received approximately 29.3 Hz depth, 4.55 Hz compressed RGB
preview and 1.57 Hz on-demand decoded RGB (configured cap 2 Hz). Both streams
were 640x480 with monotonically increasing timestamps. Median header ages were
about 17 ms RGB and 38 ms depth; the depth driver includes a -33 ms timestamp
offset. Motors/navigation were not started.

Next: test native-depth pixel geometry against independently identifiable
surface boundaries at more than one range/tilt before promoting software
registration. The current residuals alone do not establish a defect, a required
depth bias correction, or sufficient RGB-D accuracy for SLAM. The improved
offline corner detector should avoid repeating the initial detection failure.
