# R15 sensor measurement record

The new photos and a confirming measurement resolve the LiDAR mounting height missing from the initial review. R15 is derived from the preserved R14 master. Digital QA results are in `validation.json`; this is not a physical fit certificate.

## Measured inputs

| Dimension | Value | Basis |
|---|---:|---|
| Brass post body length | 44.78 mm | User caliper photograph |
| Fixed-base underside to rotating-cylinder bottom | 5.57 mm | User caliper photograph; endpoints explicitly confirmed |
| Plate top to rotating-cylinder bottom | 50.35 mm | 44.78 + 5.57; no extra printed spacer |
| Rotating cylinder diameter | 70.26 mm | User caliper photograph |
| Camera width | 161 mm | User measurement |
| Camera height | 22.80 mm | Rounded 22.78 mm measurement |
| Camera depth | 24.92 mm | User caliper photograph |

Photographs: `photo-225cbf39-3a6e-4753-af3a-afabdd298f82.png` (post); `photo-e8c05eef-5d76-4f36-99d0-c69c7435cabc.png` (base datum); `photo-2c804848-f76b-4b9b-8f2f-ec072dd3a6a7.png` (rotor diameter); `photo-2fbe04e7-9043-4494-9f9d-c68135930daf.png` and `photo-21e3907b-6e1e-484a-8a0f-bccbe2537684.png` (camera). These are reference images, not scale drawings.

## Model datums and fit allowances

Scene scale is 100 mm per Blender unit. The unchanged P10 support-plate top is Z 163.75 mm. The measured rotor bottom is therefore Z 214.10 mm. R15's flat roof is Z 217.90 mm: **27.10 mm higher than R14**, and 3.80 mm above the rotating-cylinder bottom. The central roof is 2.80 mm thick. Its 74.26 mm opening gives **2.00 mm radial clearance** to a 70.26 mm cylinder. The local roof seam passes through the rotor centre so the halves can withdraw horizontally.

The camera measurements are treated as the maximum body/optical envelope, excluding its removable rear mounting bracket and cable. This interpretation is an assumption, not an extra measurement. The camera reference remains anchored at its existing rear and bottom supports: rear Y −79.225 mm, front Y −104.145 mm, bottom Z 150.25 mm. The surround mouth is nominally 162.20 × 24.00 mm, or 0.60 mm clearance per straight side, with a flared interior. Corner shape is approximate; test the actual camera with Q15A.

The inherited upper camera clamps remain at the same position. Reducing the camera height by 0.70 mm changes the upper compliant pad from 0.50 to **1.20 mm nominal**. Actual compressed foam thickness needs a physical check; do not force the camera against rigid shims.

## What remains approximate

- LiDAR mounting-hole XY positions come from the prior model. Four 44.78 mm posts rest directly on P10, matching the photos. R15 excludes the old four 4 mm P11 printed spacers; do not add their height again.
- The rotor shell's 19.5 mm height and optical-window shapes are inherited reference details. The lower window edge is approximately 5.55 mm above its bottom in that reference, above the proposed roof. This does not establish the real optical scan plane or full field of view.
- LiDAR base-outline details, the division of the 5.57 mm stack into plate/rim/screw heights, motor, PCB, post AF dimension, threads and screw heads are approximate clearance/visual references. The former complete imported LiDAR is preserved hidden in the master.
- Camera optical elements were rescaled with the reference envelope, not individually measured. Their visual locations are not camera calibration data. The existing removable mounting bracket, clamps and cable reference are not scaled.
- No sensor relocation or URDF/calibration update is implied. Measure the installed scan plane and optical datums before updating software transforms.
- Actual cables, print shrinkage, PETG creep, vibration, thermal behavior, strength and full optical fields of view are not simulated.

Print the small fit samples before the covers. If a mounting datum differs on the physical robot, revise the model rather than forcing or uniformly scaling a cover.
