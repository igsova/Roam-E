# ROAM mechanical models

**Current integrated mechanical prototype: [R19 — serviceable three-piece lower skirt](service_skirt_r19/README.md).**

Replace the three P16 skirt panels and add four small P191–P194 chassis cleats. The feet now stay on the chassis, with four new exterior M3×12 service screws and PETG-retained nuts. Remove right → front → rear horizontally; the installed wheels, frame, upper shell and R18 drawer stay put in the sampled path checks. All eight ToF stations include four provisional female-jumper references and conservative local bend keepouts. Full flexible harness routing still needs physical checking; unplug harness tails before service. Seven print parts, two fit samples, a complete Blender assembly and a separate removal/reassembly animation are in the R19 folder. Mesh, 0.5 mm sampled motion, drawer withdrawal, exterior driver access and PETG slicing checks pass. **Keep the corrected R18 drawer and the R17 guides unchanged.**

[R18 — corrected curved battery drawer](drawer_cleanup_r18/README.md) remains the source of the unchanged P05 drawer.

Replace **only P05** relative to R17. R18 closes the unintended front-floor gaps, rebuilds the coarse washer-pad transitions, and replaces the bulky finger-pull backing with a rounded closed cup. Inside-only guide-tip relief lets you retain the **R17 P06L/P06R guides**, lower panels, switch and M3 lock hardware. The complete updated Blender assembly and a one-STL replacement ZIP are in the R18 folder. Master/STL functional sections, static interference and sampled withdrawal checks pass; the corrected STL also completes an isolated K1C/PETG slicing test. Physical print fit and supports still require inspection. **Use R18's P05 instead of the superseded drawer in the R17 STL package.**

[R17 — three-piece lower shell, curved drawer and rocker mounts](lower_controls_r17/README.md) supplies the unchanged lower parts and fit specimens.

R17 replaces the lower skirt with **P16C/P16D/P16B**, joined by M3 bolts and PETG retained nuts. The curved **P05** drawer carries the battery rocker; the right lower panel carries the separate wheel-power rocker. Reprint **both P06 guides** with the drawer. The upper hood and measured ToF positions remain unchanged, and 57 retained STLs are byte-identical to R16. Geometry, exported-solid and sampled drawer/seam checks pass. **Print Q17 fit specimens first:** the proposed Ø20.5 mm switch hole and 2.2 mm snap seat need checking with the real switches. Their provided ratings are AC-only; no direct DC battery/motor wiring is approved. See [power notes and Pi 5 auto-boot](lower_controls_r17/POWER_NOTES.md).

[R16 — closed camera surround](camera_closure_r16/README.md) remains the source of the unchanged upper robot.

R16 closes the unintended gaps between the camera surround and curved hood wall. **Replace only P15C relative to R15**; keep the existing side/rear panels, roof, sensor positions and all hardware. The other 61 STL files are byte-identical to R15. Dedicated end-coverage tests pass on both the Blender model and reimported STL, along with sampled assembly-clearance checks. Physical fit and local supports still need checking.

[R15 — raised measured-sensor enclosure](sensor_enclosure_r15/README.md) is preserved below; its front hood P15C is superseded by R16.

R15 raises the flat roof **27.10 mm** to hide the measured LiDAR posts/base, leaves a **2 mm radial clearance** around the rotating cylinder, and adds a close-fitting surround for the **161 × 22.80 × 24.92 mm** camera. Replace **P15B, P15C and P15D together**; retain all eight bolted PETG nut-retaining joints and the horizontal assembly paths. Use nominal **1.20 mm upper compliant camera pads**, and omit the old P11 printed spacers because the 44.78 mm brass posts sit directly on P10. Print Q15A and Q15B/Q15C fit samples first. Model/STL checks pass; physical fit, optical coverage and local slicer supports remain to verify. The lower robot parts and rigid sensor mounts are unchanged.

[R14 — all hood side-panel connectors bolted](panel_bolts_r14/README.md) is preserved as the previous revision.

R14 replaces both remaining thin P15B end tabs with reinforced 5 mm bolted overlaps and glue-free PETG nut retention. Replace **all three matching hood parts: P15B, P15C and P15D**. Add two M3×10 screws, two standard M3 nuts and two washers; the longer M3×12 screw interferes with the sensor frame at the new front joint. Print the Q14A/B and Q14C/D joint pairs first. The flat roof, attached smiling face, earlier nut retention and horizontal assembly paths remain; physical fit and local print supports still need checking.

[R13 — PETG press-in hood nut retention](nut_retention_r13/README.md) is preserved as the previous revision.

R13 adds tapered retention ribs inside all six H1–H6 hood nut pockets, intended to hold standard M3 nuts during assembly without glue. Replace only P15B and P15D relative to R12; keep the existing front smiling hood and all hardware. Print the three Q13 standard fit samples first: actual PETG grip must be checked before committing to the covers. Light and firm sample alternatives are included. The flat roof, reinforced bolted sides and horizontal assembly arrangement remain. See the R13 guide for physical-test requirements and digital-check tolerances.

[R12 — bolted hood side connectors](bolted_sides_r12/README.md) is preserved as the previous revision.

R12 replaces the two 1 mm front/rear seam tabs with 5 mm reinforced overlaps, captive M3 nuts and exterior-accessible bolts. Replace only P15C/P15D; add two M3×12 screws, two standard M3 nuts and two washers. Test Q12C/Q12D first. The R11 flat roof, attached smile, retained P15B and horizontal assembly paths remain. The other 46 STLs are unchanged; physical fit and local support still need checking.

[R11 — flat-roof service hood](flat_roof_r11/README.md) remains available as the previous revision.

R11 removes the raised LiDAR rim from P15C/P15D and makes their broad roof faces planar for roof-down printing. The smiling face stays attached; use local support where needed at the camera bridge, rounded perimeter, rear vents and joint details. Replace only those two hood sections, retaining P15B, all lower parts and all R10 hardware. The 44 retained STLs are byte-identical. Closed-solid checks and the three sampled horizontal assembly paths pass; physical fit and sliced support paths still need checking.

[R10 — horizontally assembled service hood](service_hood_r10/README.md) is preserved as the previous revision.

R10 fixes the camera-blocked hood installation path. Replace **only P15A** with **P15C (front) and P15D (rear)**, approaching horizontally around the installed camera/LiDAR. Reuse the printed P15B side panel and all other parts. It adds one M3 × 12 screw/nut/washer, a roof lap joint and two side locators. The three sequential assembly/removal paths are checked against the installed robot at 0.5 mm intervals. Print its small H4 coupons first, follow the assembly order, and dry-fit without power; physical fit and unmodeled wiring remain to check.

[R09 — measured ToF board and pin-row fit](tof_measured_fit_r09/README.md) remains the source of the unchanged lower skirt and ToF parts in R10/R11.

R09 corrects all eight ToF pockets and their Blender board/pin references using the measured **10.66 mm PCB short dimension** and **4.80 mm sensor-edge-to-pin-row-centre distance**. It retains the 2 mm solder clearance and R07 split covers. Only P16A, P16B and P19 STLs change from R08. **Print R09 P19 with the real soldered sensor before either skirt half.** Remaining board datums, physical fit and slicer preview still need verification.

[R08 — solder clearance](tof_solder_relief_r08/README.md) remains available for history. It provided enough solder depth but retained the old, wider board reference and the wrong pin-row position; use R09 for the measured boards.

[R07 — split hood and skirt](split_shell_r07/README.md) remains available. It adds full-size split covers and removable locating/screw joints for K1C printing. Its hood, joints, and other unchanged parts are retained in R09; its old ToF pockets lack the corrected solder clearance. **R10 supersedes the old P15A and one-piece hood fitting instructions** after the printed assembly revealed the camera-blocked insertion path.

[R06 — Geekworm + TCA9548A, unused MCU removed](geekworm_mux_r06/README.md) remains available, including its electronics inspection scene and the previous one-piece covers.

[R05 Geekworm](geekworm_r05/README.md) is also preserved. It still contains the historical MCU placeholder and does not contain the multiplexer; use the integrated R16 pack for new prints after the relevant fit tests. The older external `roam_tof_mux.blend` is a donor for multiplexer parts only, not the current complete robot.
