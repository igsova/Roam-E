# R07 — split hood and lower skirt

Full-size, unscaled covers for the Creality K1C. **Prototype: geometry-checked; slicer preview and physical fit tests still required.** R06 is preserved, including its original one-piece covers.

![R07 split covers](split_covers_exploded.png)

## Open or print

- [Complete assembled robot](roam_split_shell_r07.blend)
- [Exploded cover inspection](roam_split_shell_r07_exploded.blend)
- [Print-orientation Blender scene](roam_split_shell_r07_print_preparation.blend)
- [Changed parts only — ZIP](ROAM_R07_changed_parts.zip): four cover pieces plus two optional joint coupons
- [Complete R07 STL pack — ZIP](ROAM_STL_R07_Split_Shell_K1C.zip)
- [Assembly and print guide](ASSEMBLY_R07.md)
- [Geometry checks](geometry_checks.json) · [Independent validation](validation.json)

## What changed

P15 becomes **P15A main hood + P15B removable side panel**. P16 becomes **P16A main skirt + P16B removable side panel**. Three roof screws join the hood; two lower screws join the skirt. Integral lap tongues locate the parts, with 0.30 mm nominal mating clearance. The hood also has two shallow lower locating tabs. Standard M3 nuts are removable; no glue or heat-set inserts are required.

The cap-side mounting foot on the skirt is retained whole, not accidentally cut into a detached island. Its original M4 joint ID means the existing **M3 × 12 screw at that mounting station**, not an M4-diameter screw.

Camera, LiDAR, smile, service ports, eight ToF locations, base mounts, battery opening, Geekworm, multiplexer and Arduino Nano are retained. The unused MCU stays removed. The other **39 STL files are byte-identical to R06**. No firmware, ROS or electrical changes.

## Printing envelope

| Part | Supplied orientation | Approximate X × Y × Z, mm |
|---|---|---:|
| P15A main hood | Roof down; cavity up | 207.85 × 208.40 × 90.00 |
| P15B hood panel | Outer side toward bed | 88.04 × 171.29 × 30.00 |
| P16A main skirt | Original bottom down | 211.33 × 209.47 × 87.00 |
| P16B skirt panel | Outer side toward bed | 87.00 × 178.89 × 33.48 |

All four fit the nominal 220 × 220 mm bed, including a geometrically modeled 4 mm brim allowance. **The main skirt has little spare margin: start with a 3 mm brim and inspect generated support/brim paths.** Brim gap, excluded bed regions and support expansion also consume space. Do not scale or auto-orient these STLs.

The new orientations avoid printing a 226/233 mm-high cover on its side, but they are **not support-free**. Curved exterior surfaces, existing internal flanges, apertures and local joint details still require inspection. The installed Creality Print 7.1 command-line process crashed before slicing, including on the unmodified R06 reference; therefore no print-time, support-weight or G-code validation is claimed. Logs and attempted comparison profiles are in `slicer_check/`; no G-code is supplied.

**First step: print Q07A + Q07B and test an actual M3 screw, washer and nut.** Then slice the four covers individually. See the assembly guide for the hardware schedule and checks before installing sensors.

Reproducible source: [`create_split_shell_r07.py`](../../tools/cad/create_split_shell_r07.py), [`validate_split_shell_r07.py`](../../tools/cad/validate_split_shell_r07.py). Geometry is generated from the integrated R06 model, not an older donor robot.
