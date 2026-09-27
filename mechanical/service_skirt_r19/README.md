# R19 — serviceable three-piece lower skirt

R19 replaces the three lower skirt panels and adds four small chassis-mounted cleats. The inward mounting feet **stay on the chassis**; the covers slide horizontally off them. The wheels, chassis uprights, electronics deck and upper hood do not need to be removed for the tested panel-removal paths.

**Prototype: print the two small PETG joint samples first. Actual jumper fit, wire routing, print tolerances and mechanical strength still require a physical check.**

## Files

- `roam_service_skirt_r19.blend` — complete assembled robot, including the unchanged R18 drawer, measured ToFs, and four female-jumper references per sensor.
- `roam_service_skirt_r19_assembly_demo.blend` — separate inspection file. Play frames 1–241 to see removal and reassembly. Upper parts are hidden for visibility, not omitted from the collision tests. Fasteners that must be removed are hidden in this demonstration.
- `ROAM_R19_skirt_replacement.zip` — seven replacement/additional print parts plus two fit samples, instructions, previews and validation reports. Not a full-robot STL pack.
- `R19_lower_exploded.png` — cyan removable panels; orange fixed cleats.
- `R19_four_female_jumpers.png` — local connector and wire-bend reference.

## What to print and retain

| Part | Quantity | Purpose |
| --- | ---: | --- |
| P16B R19 | 1 | Right wheel-switch panel; removes to the robot's right |
| P16C R19 | 1 | Front three-ToF skirt; removes forward |
| P16D R19 | 1 | Rear/left four-ToF skirt; removes backward |
| P191 | 1 | Fixed front-left cleat, old M1 anchor |
| P192 | 1 | Fixed front-right cleat, old M2 anchor |
| P193 | 1 | Fixed left cleat, old M3 anchor |
| P194 | 1 | Fixed right cleat, old M4 anchor |
| Q19A + Q19B | 1 each, test first | Actual panel/cleat joint samples |

Replace all three skirt panels together. Keep the **R18 P05 drawer**, R17 P06 guides, chassis, wheels, pylons, upper shell, all eight sensors, P17 retainers and P18 pads. Their original model geometry/poses are unchanged. Existing printed panels are not direct drop-in partners for the new cleats.

Add **four M3×12 socket-head screws, four standard M3 nuts and four M3 washers** for the new outside-access service joints. Nominal nut size is 5.5 mm across flats × 2.4 mm thick; the new pockets have tapered PETG retention ribs. No adhesive is intended. Test the actual nuts in Q19B: hold snugly without splitting the plastic; do not force an oversized nut.

Reuse the four old chassis mounting screw/washer/nut sets to attach the cleats to the existing holes. **M1–M4 are location identifiers, not M4-size fasteners; all four use the existing M3 hardware.** Reuse the S1–S4 panel seam hardware and its existing retained nuts.

## First installation

1. Disconnect the battery and any USB/external power. Remove the old panels. Withdraw the battery drawer if needed for access to the existing chassis fasteners; do not disturb its guides.
2. Print Q19A/Q19B in the intended PETG profile and test the new bolt/nut fit. Check a real female jumper against the provisional connector dimensions below before printing the large parts.
3. Fit the four retained nuts into P191–P194. Bolt these cleats onto the four old chassis mounting holes. They remain fixed during subsequent cover removal. Check the original screw heads/washers seat fully and the cleats are stable.
4. Transfer the ToFs, P17/P18 parts, seam nuts and wheel switch to their matching R19 panels. Plug on the four female jumpers per ToF, avoiding any load on the solder joints.
5. Assemble **rear → front → right** horizontally, following the reverse of the removal sequence. Keep wiring clear of the chassis edges, wheels, cleats and seams. Refit service/seam screws only after every panel is seated. Tighten gently; do not pull a misaligned PETG panel into place with the screws.

## Routine removal — no upward sliding

Disconnect all power and unplug the relevant sensor harness tails and wheel-switch wiring **before moving a panel**. Keep a service disconnect accessible; the whole flexible harness is not modeled. Do not pull on wires or leave loose wire loops near a wheel.

1. Remove the **right panel P16B** first: remove the old S1/S2 seam screws/washers and the new external M4-location screw/washer. Pull straight right (+X). Leave the original vertical chassis anchor screw in P194 installed.
2. Remove the **front panel P16C**: remove S3/S4 seam screws/washers and the two new external M1/M2-location screws/washers. Pull straight forward (−Y). P191/P192 stay on the chassis.
3. Remove the **rear panel P16D**: remove the new external M3-location screw/washer. Pull straight backward (+Y). P193 stays on the chassis.

The model tests each path out to 240 mm. This is a full-clearance demonstration, not a requirement to pull that far before supporting or lifting an already-clear panel. **Do not lift while it is still wrapped around the installed frame.** Reassembly is rear, front, then right. The drawer can remain installed during these tested paths.

## ToF jumper space — explicit assumptions

Female jumpers confirmed. Each of the eight stations now includes **four 2.5 × 2.5 × 14 mm female housing references** and **1.6 mm-diameter wire references with a 6 mm centreline bend radius**. A 16.2 mm-wide keepout spans the entire six-pin row, so the clearance check is not tied to the illustrated four pin positions. The pictured colors and pin positions are **not an electrical wiring diagram**.

The connector/bend envelopes fit below the chassis. The highest local bend reserve is Z44.2 mm; the chassis underside is about Z51.91 mm. **The remaining wire tails and their complete route to the controller are not modeled or validated.** Routing straight upward at the inboard end of these plugs would hit the chassis and, at some stations, the drawer/guides. Route and secure the tails in the under-chassis space, then find an existing clear route upward; do not trap them behind the drawer or run them across wheel travel. Verify this physically while the covers are loose and power is disconnected. No new wire passage through the chassis has been cut.

Original optical positions, the 10.66 mm board dimension, 4.8 mm sensor-edge-to-pin-row datum, and 2 mm solder reliefs are preserved. Longer plugs, thicker insulation or stiffer wires may need more room. Do not sharply kink wires to make an oversized connector fit.

## Printing

All supplied STLs are in millimetres, centered on the bed and grounded. Do not rescale. Start with their exported orientation: skirts upright/bottom-down, cleats on their chassis-contact bases. They are not advertised as support-free. Inspect supports near wheel cutouts, ToF details, screw holes and retained-nut pockets; avoid damaging the small retention ribs during support removal.

The largest panel is about **213.53 × 122.65 × 87 mm**. A 2 mm brim fits the nominal 220 mm K1C square; a 4 mm brim does not fit that panel's width. Check the actual printer exclusion zones and brim/toolpath preview.

An isolated Creality K1C/Generic PETG 0.20 mm smoke test completed for all nine STLs (four walls, 20% infill, automatic snug supports, 2 mm brim). This confirms slicer processing, **not tuned print quality, support removability or machine readiness**. Diagnostic projects/G-code under `slicer_check/` are not approved print jobs and are excluded from the replacement ZIP.

## Digital checks and limits

- 889 R18 objects remain geometrically unchanged; only the three skirt objects were replaced. The R18 source file is hash-checked and untouched.
- Seven installed printed solids and both reimported fit samples are single connected manifold meshes with positive volumes.
- No new static part/hardware volume interference, apart from the intentional nominal nut/rib press fit. Coincident exterior washer bearing faces are checked with a reported 0.002 mm numerical separation, not by shrinking the shell.
- Each panel, its sensors/retainers, captive seam nuts, switch where applicable, local wires and conservative connector reserves pass **481 positions at 0.5 mm intervals over 240 mm**. Installed wheels, frame, upper assembly, drawer and other remaining panels are included. Reverse motion represents assembly.
- The unchanged loaded drawer passes **301 withdrawal positions over 150 mm** with the new panels and cleats installed.
- All four new exterior bolts have a checked 6 mm-diameter driver-access envelope. New joint clearance is 0.4 mm; actual nut-grip surfaces and all eight original solder reliefs were measured in the resulting mesh.

These are sampled geometric checks, not continuous-motion proof, strength analysis, flexible cable simulation or a physical fit guarantee. Dry-fit the PETG samples and real harness before committing to long prints. No electrical circuit, switch rating or battery safety approval is implied.
