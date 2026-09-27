# R15 — measured sensor enclosure

**P15C correction:** use the [R16 front hood](../camera_closure_r16/README.md) for new prints. R15 left an unintended camera-surround opening at the curved wall. R16 closes the ends and retains all other R15 components; only P15C needs replacing. These R15 CAD/STL files remain preserved for history.

Raised flat roof around the measured LiDAR mounting stack, a close-fitting camera surround, and the existing three-piece horizontal assembly with eight retained-nut bolted joints. **Reprint P15B, P15C and P15D together.** Older revisions remain preserved.

## Files

- [Assembled Blender master](roam_sensor_enclosure_r15.blend)
- [Exploded inspection scene](roam_sensor_enclosure_r15_exploded.blend)
- [Replacement hood, optional pads and fit samples — ZIP](ROAM_R15_hood_replacement.zip)
- [Complete integrated STL pack — ZIP](ROAM_STL_R15_Sensor_Enclosure_K1C.zip)
- [Assembly, hardware and printing guide](ASSEMBLY_R15.md)
- [Measured inputs and assumptions](MEASUREMENT_REVIEW.md)
- [Digital verification report](validation.json)

The roof rises **27.10 mm**, keeping a **2.00 mm nominal radial gap** around the 70.26 mm rotating LiDAR body. There is no raised roof ring. The camera reference is corrected to **161 × 22.80 × 24.92 mm**; its rear and bottom mounting datums stay fixed. Rigid mounts are unchanged. Two upper compliant pads change to **1.20 mm nominal**; verify actual foam compression.

Print **Q15A camera opening** and **Q15B/Q15C LiDAR aperture halves** before the full hood. Check unpowered with the actual sensors. Do not rest the samples on the LiDAR. The complete pack retains previous Q13/Q14 nut/joint tests.

The measured 44.78 mm brass posts sit directly on P10. The old four 4 mm P11 printed spacers are excluded: adding them would invalidate the height. Reuse H1–H6 M3×12 and H7/H8 M3×10 hardware. All hood nut-retention profiles remain the standard PETG design.

The roof is flat for roof-down printing; the attached smiling face and camera bridge can still need **local supports**. P15B retains its curved side surface. Slicer support extents, actual fit, full optical FOV, wiring, print strength and nut grip are not physically validated. Camera corner shape and hidden LiDAR details are approximate; read the measurement record before printing.

![R15 assembled native CAD preview](hood_assembled.png)

![R15 exploded native CAD preview](hood_exploded.png)
