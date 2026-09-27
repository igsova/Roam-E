# R15 — raised LiDAR roof and measured camera surround

Replace **all three matching hood sections P15B, P15C and P15D together**. Do not mix an R15 raised section with an older hood. The lower skirt, ToF fit, electronics carriers, chassis, rigid camera mounts and support plate are unchanged.

## What changes

- The flat roof rises **27.10 mm**. The four measured 44.78 mm brass posts and stationary LiDAR base are enclosed; the rotating round body projects through a **74.26 mm** opening with nominal **2 mm radial clearance**.
- The roof has **no projecting LiDAR ring**. Its local front/rear seam passes through the rotor centre, allowing each half to slide horizontally away from the close-fitting opening.
- A built-in camera surround encloses the body, leaving the measured **161 × 22.80 mm** front visible. Reference depth is **24.92 mm**. The smiling face remains attached to the front hood.
- All eight bolted joints retain their R14 PETG press-in nut profiles. Upper roof joints move upward as complete features, not scaled holes. The side-panel end laps remain bolted, not thin clips.
- Two upper camera pads become **1.20 mm nominal** instead of 0.50 mm, because the corrected camera is 0.70 mm shorter. Prefer compliant cut foam; the F03 STL is an optional flexible-material reference, not a rigid spacer.
- The four old **P11 printed LiDAR spacers are omitted**. Your measured brass posts sit directly on the black P10 plate. Do not double-count spacer height.

## Test before reprinting the covers

Disconnect the battery and all external power. Never put a fit sample on a powered, spinning LiDAR. Avoid touching optical windows or using the sensor as a load-bearing support.

1. Print **Q15A camera-face fit ring** with the intended PETG profile. Check it passes over the actual front without force and does not cover any optical window. This tests the opening, not the complete rear bracket or cable clearance.
2. Print **Q15B + Q15C LiDAR aperture halves**. Support them at the proposed roof level, **54.15 mm above the black plate**, independently of the rotor. Bring them horizontally around the sensor. Check a clear gap all around, including runout; stationary base/fasteners must remain below the roof underside. Confirm real optical windows and scan plane are above the roof.
3. If your PETG nut fit has not been proven, use the retained **Q13 standard** nut-fit tests and **Q14A/B + Q14C/D** end-joint samples in the complete pack. Nut size is standard M3, nominal 5.5 mm AF × 2.4 mm thickness. The ribbed pocket is 5.9 mm AF × 2.8 mm deep, with a 5.30 mm opposing-rib-tip distance. Do not use taller locknuts.
4. If a nut drops out, spins, cannot seat squarely or cracks the plastic, stop. Change the local rib fit after testing; do not scale a full cover. Retention is for handling without glue, not a locking-nut or vibration guarantee.

Camera dimensions are treated as its maximum envelope. Corners, hidden LiDAR details and cable routing are not fully measured. See `MEASUREMENT_REVIEW.md` for assumptions. Check samples before another long print.

## Printing

Use supplied STL orientation, millimetres, **100% scale**. P15C and P15D print with their broad, flat exterior roof on the bed. P15B remains outer-side-down with its inherited curved exterior. Keep the smiling face attached as requested.

**Local supports can still be required** at the camera bridge/surround, joint ears, nut pockets, curved side panel, shoulders and vents. This is not claimed support-free. Inspect sliced layers and support contact locations. Keep nut ribs and bolt bores clean without cutting away retention material.

New exports are reimport-checked for closed single-solid meshes, bed Z=0, correct millimetre dimensions and nominal K1C bounds with a 4 mm brim allowance. Check actual slicer exclusion zones and support extents separately. `PRINT_PARTS.csv` contains exported dimensions. No G-code or verified time/material estimate is supplied.

## Hardware

Reuse the R14 hardware:

- **H1–H6: M3×12** socket-head screws, existing M3 nuts and washers.
- **H7/H8: M3×10** socket-head screws, standard M3 nuts and nominal 6 mm OD × 0.5 mm washers. Do not use M3×12 here; the longer front screw interferes with the frame in R14.
- Existing K frame fasteners, P12 sleeves and P14 plugs.
- Actual four **44.78 mm brass LiDAR posts**, with the sensor's compatible fasteners. Reference screw meshes are not a thread or engagement specification.
- Two upper compliant camera pads, **1.20 mm nominal**; verify grip and compression with the real camera before tightening.

Metal hardware and sensors in Blender are references, not printable parts. Historical hardware tables apply only to unchanged items and must not override the measured LiDAR stack above.

## Assembly

Keep power disconnected and cables away from sliding seams and bolt tips. Allow approximately 250 mm free working space in front, behind and to the right.

1. Fit the camera on its existing lower/rear supports and check the new upper pad thickness. Rigid clamps stay unchanged; tighten gently without deforming the body.
2. Check LiDAR posts stand directly on P10, with the cylinder bottom **50.35 mm above the plate**. Keep wires below the proposed roof and clear of the rotor.
3. Preload H4–H6 nuts into P15D, and H1–H3 plus H7/H8 into P15B. Verify nuts stay seated during handling. Retain appropriate P12 sleeves and P14 plugs.
4. Bring **P15D horizontally from the rear**. Its half-aperture surrounds the rear of the rotor. Secure its rear-left K fastener loosely.
5. Bring **P15C horizontally from the front**. The camera passes through the opening and the front roof half closes around the rotor. Secure its front-left K fastener loosely.
6. Bring **P15B horizontally inward from the right**. Engage roof tongues and reinforced end laps without bending. Start all H and K fasteners by hand, then snug evenly. H7 enters from the front exterior and H8 from the rear exterior.
7. Before power-up, check the camera is not pinched, the rotor is untouched with clearance around its circumference, optical windows are unobstructed, and no wire/nut/bolt tip can contact the moving assembly. Compare camera images and LiDAR scan coverage after cautious reassembly; CAD checks do not certify optical performance.

For removal, undo H1–H8 screws/washers and appropriate K fasteners. Withdraw P15B rightward first, P15C forward, then P15D rearward, removing each temporary holding K fastener before moving that piece. Do not lower or lift the closed hood past the camera. Nuts should remain in their own pieces.

## Digital verification and limits

`tools/cad/create_sensor_enclosure_r15.py` builds from preserved R14. `validate_sensor_enclosure_r15.py` independently checks geometry, unchanged-object identity, eight nut-seat profiles, flat roof areas, stationary intersections, rotating-cylinder clearance and three horizontal removal paths sampled every **0.5 mm for 250 mm**. The exporter requires a passing validation hash for the exact master and geometry report before making STLs.

New STLs are reimported and checked; retained files are hash-checked against R14. ZIP CRCs and STL hashes are verified. `validation.json` records exact results and limitations. Physical fitting, strength, nut grip, cables, optical calibration/FOV and sliced supports still need checking. This is not continuous-motion proof or a physically tested print.
