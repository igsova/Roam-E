# R08 — ToF solder-pin clearance

R08 fixes the shallow pin-row relief in **all eight ToF pockets**, building on the R07 split hood/skirt and R06 electronics. The sensor-facing solder/pin tips were estimated at about **1 mm above the PCB**. The revised pocket provides **2.0 mm nominal clearance from that PCB face**, with a closed outside skin. Print the small **R08 P19** pocket first; this is not a physically validated fit yet.

- [Assembled Blender model](roam_tof_solder_relief_r08.blend)
- [Lower-skirt inspection model](roam_tof_solder_relief_r08_skirt_inspection.blend)
- [Before/after pocket comparison](roam_tof_solder_relief_r08_fit_comparison.blend)
- [Changed STLs only: P16A, P16B, P19](ROAM_R08_changed_parts.zip)
- [Complete R08 STL pack](ROAM_STL_R08_ToF_Solder_Relief_K1C.zip)
- [Print and fit-test instructions](ASSEMBLY_R08.md)
- [Geometry checks](geometry_checks.json) · [Independent verification](validation.json)

![The original shallow pad relief and the new deeper solder channel](solder_relief_comparison.png)

## What changed

Each pin row now has a **16.2 × 4.2 mm** internal clearance channel, centred at the original header-pad row. The channel is 2.0 mm deep measured from the sensor-facing PCB seating plane; it is not a 2 mm cut into the old 0.85 mm wall. A tapered exterior bulge keeps **1.2 mm of closed plastic outside the channel**. Its outermost face is **2.35 mm beyond the original local flat face**.

The old row had about **0.2 mm** clearance above the PCB, suitable only for its modeled copper pads. The revised channel does not rely on trimming or flattening the existing solder joints.

The board seating plane, optical aperture, sensor position/orientation, retaining ears, P17 retainer, P18 pads, mounting holes, and R07 split joints remain unchanged. No global scaling was applied. Only **P16A, P16B and the P19 trial pocket** have different STL geometry; the other **42 files** are byte-identical to R07. All earlier revisions remain untouched.

This intentionally changes the outside outline locally. Check the new footprint and supports in the slicer; use this revision's print notes instead of the old P16A brim recommendation. No additional hardware is needed.
