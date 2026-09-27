# R09 — measured ToF board and pin-row fit

R09 corrects the lower skirt using the measured **10.66 mm PCB short dimension** and confirmed **4.80 mm from the nearest black sensor-package edge to the pin-row centre**. Earlier R08 allowed solder depth, but retained the incorrect board width and row position.

| Dimension | R08 | R09 |
|---|---:|---:|
| PCB short dimension | 12.60 mm | 10.66 mm |
| Sensor centre to pin-row centre | 7.90 mm | 6.00 mm |
| Sensor edge to pin-row centre | 6.70 mm | 4.80 mm |
| PCB-pocket short dimension | 13.20 mm | 11.26 mm |
| Clearance above PCB for solder | 2.00 mm | 2.00 mm |

The sensor package's modeled short dimension remains 2.4 mm, so its half-width plus the measured edge-to-row distance gives **1.2 + 4.8 = 6.0 mm**. The channel and external closed bump move **1.9 mm toward the optical opening**. The board's pin-side edge and pocket wall move inward by **1.94 mm**, without scaling the robot or moving its optical centres.

- [Assembled Blender model](roam_tof_measured_fit_r09.blend)
- [Lower skirt inspection](roam_tof_measured_fit_r09_skirt_inspection.blend)
- [R08/R09 pocket comparison](roam_tof_measured_fit_r09_fit_comparison.blend)
- [Changed STLs: P16A, P16B, P19](ROAM_R09_changed_parts.zip)
- [Complete R09 STL pack](ROAM_STL_R09_Measured_ToF_K1C.zip)
- [Print and fit-test instructions](ASSEMBLY_R09.md)
- [Geometry checks](geometry_checks.json) · [Independent verification](validation.json)

![Actual R08 and R09 pocket sections](measured_fit_comparison.png)

## Scope and assumptions

All eight sensor stations are revised. The Blender PCB references are narrowed to 10.66 mm and all six pad/pin positions per board move with the new row. The sensor package, optical windows, circular mounting ears, mounting hardware, P17 retainers and P18 pads stay at their prior datums. **Only P16A, P16B and P19 change in the STL pack; the other 42 files are byte-identical to R08.** Previous revisions are untouched.

The measured 10.66 mm and 4.80 mm values do not establish every board datum. PCB length (25 mm), thickness (1.6 mm), optical-side PCB edge (3.1 mm from optical centre) and mounting-ear centres remain inherited assumptions. The model shortens the pin side only; it does not distort the circular ears or change the optical package to make the measurement fit. **Print R09 P19 with the real soldered board first.**

The PCB cavity keeps 0.3 mm nominal clearance at each short edge. Its actual seating plane stays unchanged. The solder recess is 16.2 × 4.2 mm with 2 mm clearance above that plane, with a 1.2 mm closed outside skin. The optical aperture remains 5.6 × 3.6 mm. Its edge is now 2.1 mm from the nearest solder-channel edge; this is intentionally smaller than the measured sensor-edge-to-pin-centre distance because the aperture and channel each include clearance.
