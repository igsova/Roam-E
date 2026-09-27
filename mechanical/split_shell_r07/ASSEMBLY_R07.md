# R07 split-cover assembly and printing

## Status and scope

R07 replaces **only P15 and P16** with P15A/P15B and P16A/P16B. Do not also print the old one-piece covers. Keep every previously printed R06 part other than those covers. The complete R07 pack contains 45 unique STLs / 120 pieces including optional items; two of the new items are joint test coupons, not robot components.

The robot is not scaled. The split lies at assembled X=95 mm, with opposing cut faces at X=94.85 and X=95.15 mm. “Right” here means the model's +X side, not an instruction to reverse ROS frames. Joint tongues extend across this plane inside the shell; printed panel dimensions include them. New local screw recesses and internal reinforcing pads are intentional.

**Mechanical prototype only.** Mesh checks do not establish strength, long-term creep, heat resistance, sensor accuracy, waterproofing or physical fit. The prior Geekworm/MUX assumptions remain provisional. Disconnect all robot power before working on the covers; do not drive the robot with loose panels or dangling sensor wiring.

## Additional hardware

Buy **five M3 × 12 mm socket-head screws, five standard M3 hex nuts (5.5 mm across flats, approximately 2.4 mm thick), and five M3 washers (approximately 6 mm outside diameter, 0.5 mm thick)** in addition to the existing hardware. The joint schedule is in `geometry_checks.json`, `manifest.json` and `R07_JOINTS.csv`. Screw lengths are measured under the head. Do not substitute thicker locknuts without checking the geometry.

- H1 / H2 / H3: the three roof joints, at X=89 mm and Y=-35 / 0 / +35 mm.
- S1 / S2: front/rear lower skirt joints, at X=89 mm and Z=19 mm.
- The exact generated screw lengths are listed in `R07_JOINTS.csv` (also in the package). Existing K hood screws and the existing skirt screws are reused.

Each screw bore is nominally 3.4 mm. Captive nut recesses are 5.9 mm across flats and 2.8 mm deep, open from the cavity side. The printed nut-bearing shoulder is 2.2 mm. Roof receiver pads are reinforced rather than relying on a thin roof skin; the geometry report gives each main screw-bearing thickness. Hardware meshes are schematic purchased-part references, not STLs to print.

## 1. Print the joint samples first

1. Import **Q07A** and **Q07B** separately at 100% scale in millimetres. They are cropped from the actual H1 roof joint.
2. Keep their supplied orientations. Inspect local support, the 3.4 mm bore, washer seat and nut pocket; remove support cleanly.
3. Insert an M3 nut from the cavity side. Bring the tongue into the matching open-sided receiver. They should locate without forcing or separating print layers.
4. Fit the washer and the screw length specified for H1. Tighten gently by hand. Confirm a flat seat, full nut engagement, no bottoming and no cracking or distortion.
5. The nominal receiver clearance is **0.30 mm per mating face**. A snug nut fit depends on material and printer calibration. If the coupons bind, adjust local fit tolerances in the CAD; do not shrink the whole robot. These samples check the connector principle, not the whole cover or every curved wall.

Q07 parts are optional and are not installed on the robot. The inherited P19 coupon is still the correct separate test for your real ToF board/pads/retainer.

## 2. Slice the four cover pieces

| STL | Orientation already applied | Support areas to inspect |
|---|---|---|
| P15A | Roof down, cavity up | Low supports under curved roof; camera-aperture bridges; original bosses and local receivers |
| P15B | Outer side toward bed | Curved exterior, root of the lap tongues, original hood bosses |
| P16A | Original bottom down | Original inward mounting flanges, wheel relief, ToF pocket/nut-entry details |
| P16B | Outer side toward bed | Curved exterior, original M4-station foot and right ToF pocket |

- Import one full cover piece per plate; no scaling and no automatic lay-flat/orientation changes.
- Start with the calibrated **0.4 mm nozzle / 0.20 mm layer** profile you use for your chosen rigid material, 3 walls and approximately 15% infill; inspect the thin walls and small locating features. These are starting settings, not a tested material prescription.
- Use around **4 mm brim for P15A; 3 mm for P16A**. The smaller panels have ample room for a larger brim if your material needs it. Account for brim gap and actual bed exclusion regions.
- Use local/snug supports where possible. Unrestricted tree branches can leave the bed even when the model fits. Check every support/brim path against the actual printable region before sending a job.
- Do not indiscriminately fill the entire cavity with support. Also do not turn support off everywhere: the geometry still has unsupported local surfaces. Check layer preview around the roof, camera opening, flanges and joint details.
- Protect the captive-nut recesses, sensor apertures and thin locator slots from difficult-to-remove supports. Confirm the slicer retains those features, rather than smoothing them away.
- The curved roof is **not perfectly flat**. Roof-down reduces long support towers; it does not make the whole initial footprint touch the bed.

All supplied STLs are grounded at Z=0 and contain one connected, closed solid each. The complete print pack preserves the other R06 print orientations. The Blender assembly uses 100 mm per Blender unit; do not apply that scale again to millimetre STLs.

The automated Creality Print comparison failed at CLI startup, before slicing, on an unchanged R06 model too. Actual support grams and time savings are therefore **unmeasured**. Review and slice in the GUI using your own calibrated profile. No executable G-code is included.

## 3. Assemble the hood

1. Clean support from P15A and P15B; inspect the three roof lap tongues and two shallow lower locating tabs.
2. Insert the three M3 nuts into P15B from underneath the roof tongues. Keep them seated while bringing the parts together.
3. Slide P15B inward in the -X direction. The roof tongues enter P15A's receivers and the lower tabs align the front/rear seam edges. Keep the designed 0.30 mm seam clearance; do not clamp it out by distorting the shell.
4. Fit the three roof screws and washers from above. Tighten evenly and gently; they hold the roof overlap together, not the camera or LiDAR.
5. The original camera, LiDAR, four service ports, smile and left/right hood fasteners retain their positions. P15B keeps the original two +X-side K mounting bosses. Refit the assembled hood to the original sensor frame and reuse its original screws/nuts/sleeves.
6. Check the hood is seated, the camera opening is unobstructed, the LiDAR rotates freely, and no new joint hardware touches the frame or wiring. Confirm fan intake and cable bends on the real build.

The smile is fused into the P15A STL. Blender retains separate display materials; STL does not encode colors or illumination. Paint the raised accent after printing if desired.

## 4. Assemble the skirt

1. Fit the S1/S2 nuts into P16B's front/rear lower tongues. Slide the panel into the main skirt's receivers and install its two lower screws and washers from outside.
2. The complete original **M4 mounting station** belongs to P16B. M4 is a joint identifier: it still uses the original **M3 × 12 mm** hardware. The hole remains at X=90, Y=42 mm. Do not omit this base attachment; it supports the removable panel in addition to the two seam screws.
3. Install the eight original ToF assemblies using their unchanged P17 retainers, P18 compliant pads and T-series screws. Seven stations are on P16A; the +X-side station is on P16B. Verify board fit with P19 before installing all sensors.
4. Route the right sensor's harness with strain relief and enough service slack to detach the panel. No new connector or harness design is supplied here. Do not trap wires in either lower tongue or near a wheel.
5. Bolt the skirt to the original base mounts. Check both wheel envelopes and battery drawer movement manually with power disconnected. Confirm the rear battery opening is clear.

For service, unplug the relevant sensor harness first and release the panel's original robot attachment(s) as well as its new seam screws. Do not pull a bolted panel off the frame. The exploded illustration identifies parts, not a collision-free removal animation.

## Validation and prior references

`geometry_checks.json` covers manifold solids, modeled joint/frame/hardware clearances, unchanged R06 objects, source hash, fit dimensions and screw shoulders. `validation.json` independently checks protected source surfaces, unchanged part hashes, new STL imports and sampled cap separation. These are digital checks, not a physical or material test.

R07 retains the current Geekworm C02/C08/F05 and MUX C09/P20/F06 parts. The unused MCU/C04/S06 remain absent. Historical R04/R05/R06 instructions are supplied in the full pack for unchanged parts; **R07 overrides their one-piece P15/P16 cover and cover-orientation instructions**, and the current manifest/CSV overrides old quantities or part IDs.
