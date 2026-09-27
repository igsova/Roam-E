# R11 — flat-roof service hood

R11 removes the raised LiDAR ring and makes the broad P15C/P15D roof faces planar, so the supplied roof-down prints sit directly on the bed. The smiling face stays attached, as requested. **Local support remains necessary where indicated by the slicer**: camera-opening bridge, rounded perimeter transitions, rear vents and joint details.

## Files

- [Complete robot Blender model](roam_flat_roof_r11.blend)
- [Isolated hood inspection](roam_flat_roof_r11_hood_inspection.blend)
- [Roof-down print illustration](roam_flat_roof_r11_print_inspection.blend) — geometry only, not sliced supports
- [Replacement STLs ZIP](ROAM_R11_hood_replacement.zip) — P15C, P15D, optional Q11C/Q11D joint coupons
- [Complete integrated STL pack ZIP](ROAM_STL_R11_Flat_Roof_K1C.zip)
- [Print and assembly instructions](ASSEMBLY_R11.md)
- [Geometry report](geometry_checks.json) and [independent validation](validation.json)

![Flat roof without raised ring](flat_roof_detail.png)

## Reuse your other prints

Only the two main hood sections change from R10. Reuse **P15B**, all camera/LiDAR mounts, electronics carriers, both lower skirt pieces, measured ToF parts and every existing fastener. There is no additional hardware beyond R10. R11 retains the horizontal assembly order: **rear P15D → front P15C → side P15B → remaining screws**. Do not lower the assembled hood over the camera.

The roof plane is assembled Z=190.80 mm; both STLs are 88.80 mm high in print orientation. The old curved shoulder and retained side panel remain below/adjacent to the new flat central surface. No whole-model scaling was applied. The H4 washer seat and sliding fit are unchanged.

Both replacement hood meshes and the coupons pass closed-solid/reimport checks. The broad roof has >2.3 mm sampled thickness. The three assembly/removal paths are checked at 0.5 mm intervals over 240 mm against the installed robot, including stationary holding screws. These checks do not prove continuous clearance, real cable clearance, physical fit, strength or slicer support consumption. Test coupons and dry-fit with power disconnected. Inspect the sliced layers before printing; no G-code is supplied.

![Supplied roof-down orientation, local supports not shown](roof_down_print_layout.png)

## Reproduce

From the repository root, run Blender with `--background --disable-autoexec --python tools/cad/create_flat_roof_r11.py -- --render`, then `--background --disable-autoexec --python tools/cad/validate_flat_roof_r11.py`. The generator reads R10 without modifying it; the validator assembles and verifies the ZIP packages after QA.

Earlier revisions remain intact under their own folders.
