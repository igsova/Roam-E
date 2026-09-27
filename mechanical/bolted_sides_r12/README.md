# R12 — bolted hood side connectors

The two thin side tabs at the front/rear hood seam are replaced by **5 mm bolted overlaps**, reinforced front pads and captive M3 nuts. Two exterior-accessible M3×12 bolts clamp the joint instead of relying on the old 1 mm locating tabs. R11's flat roof, attached smiling face and horizontal installation paths are retained.

- [Complete robot Blender model](roam_bolted_sides_r12.blend)
- [Isolated hood inspection](roam_bolted_sides_r12_hood_inspection.blend)
- [Exploded side-joint inspection](roam_bolted_sides_r12_joint_inspection.blend)
- [Print-orientation inspection](roam_bolted_sides_r12_print_inspection.blend)
- [Replacement STL ZIP](ROAM_R12_hood_replacement.zip): P15C, P15D and optional Q12C/Q12D fit-test pair
- [Complete integrated STL ZIP](ROAM_STL_R12_Bolted_Sides_K1C.zip)
- [Print / hardware / assembly guide](ASSEMBLY_R12.md)
- [Geometry checks](geometry_checks.json) and [validation](validation.json)

![Outside-accessible side bolts](bolted_side_detail.png)

Reprint only **P15C + P15D** after testing **Q12C + Q12D** with your actual hardware. Keep P15B and all other robot parts. Add **two M3×12 screws, two standard M3 nuts and two M3 washers**. Load the nuts into the rear cover before fitting it; remove the side screws before taking the covers apart. Do not drill the old thin tabs or attempt to snap the new overlaps into place.

The new pieces are closed solids; the three horizontal assembly paths, screwdriver access and unchanged flat roof are checked digitally. The other 46 STLs are byte-identical to R11. Physical fit, cables and strength remain to be checked. Keep local support at the camera bridge, rounded transitions, vents and the projecting side ears; the design is not entirely support-free.

![Inside of separated side joint](side_joint_exploded.png)

Earlier revisions are untouched. To regenerate from the repository root, run Blender with `--background --disable-autoexec --python tools/cad/create_bolted_sides_r12.py -- --render`, then `--background --disable-autoexec --python tools/cad/validate_bolted_sides_r12.py`. The validator packages the checked output after the required assertions pass.
