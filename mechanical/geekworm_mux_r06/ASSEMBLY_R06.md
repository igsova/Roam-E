# ROAM R06 — Geekworm + TCA9548A, unused MCU removed

This is the combined model built from the latest **Geekworm R05 robot**, importing only the multiplexer assembly from `roam_tof_mux.blend`. Do not switch back to that donor robot: it has the older buck installation.

The separate **`Component | MCU` placeholder is removed**, along with its carrier, spacers, screws, washers, nuts and label. **Arduino Nano, Raspberry Pi, BNO055, LiDAR interface, motor drivers, camera and eight ToF mounts remain.** No ROS code, firmware or electrical wiring is changed.

## Current print parts

| Part | R06 action | Quantity |
|---|---|---:|
| P04 electronics deck | Replace: old MCU holes filled; two multiplexer holes added; channel feet integrated in STL | 1 |
| C04 old MCU carrier | Removed — do not print | 0 |
| S06 old MCU PCB spacers | Removed — do not print | 0 |
| S01 common 2.5 mm lower spacers | Quantity reduced from 14 to 12; same geometry | 12 total |
| C09 TCA9548A carrier | Imported donor C08, renamed to avoid Geekworm C08 collision | 1 |
| P20 multiplexer end clips | Imported unchanged | 2 |
| F06 multiplexer compliant pads | Imported donor F05, renamed to avoid Geekworm F05 collision | 2 |
| C02 / C08 / F05 Geekworm parts | Retained unchanged from the current R05 robot | 1 / 2 / 2 |

The complete pack has **41 unique STL files / 116 pieces**, including optional parts. That total is coincidentally unchanged: five printed MCU-related pieces are removed and five multiplexer pieces are added. Each STL contains one part; duplicate it according to the filename/CSV. C04 and S06 are intentionally absent. Do not print C08 from the old multiplexer pack: the current mux carrier is **C09**.

The changed-only pack contains P04, C09, P20 and F06: four files, six pieces. For an already printed R05 robot, remove the old MCU mounting parts and keep only 12 S01 spacers in the complete assembly. No new S01 print is necessary.

## Multiplexer assembly

1. Disconnect all robot power. Remove the hood/sensor frame and electronics cassette as needed for safe underside access. Remove the unused MCU assembly; leave the Nano in place.
2. Trial-print **C09 and two P20 clips first** and check the real multiplexer, soldered headers, jumpers and screw fit before printing the replacement deck. Its footprint comes from the provided **21.4 × 31.5 mm** outline; other details remain provisional.
3. Fit the revised P04 deck. Its new multiplexer through-hole axes are **(-6, -2) and (24, -2) mm** in assembled robot XY coordinates, diameter **2.9 mm**. Former MCU holes at **(2, 24.25) and (18, 24.25) mm** are closed in the new model. All other board positions and the Geekworm mount are unchanged.
4. Bolt C09 to the deck with **two M2.5 × 10 mm screws, two M2.5 nuts and four washers**, one washer above and below each joint. Suitable nuts/washers from the removed MCU assembly can be reused; its two long M2.5 × 20 screws are not the specified mux screws. Check actual engagement and underside clearance.
5. Place the TCA9548A component-side up. Its 31.5 mm length follows the carrier's long dimension. The board rests on ledges at the short ends; the PCB mounting holes are not used. Keep solder tails clear of the base.
6. Put one **F06** soft pad under each clip tongue at the board edge. Fit the two **P20** clips, rotated 180 degrees relative to each other, with **two M2 × 6 mm screws**. These screws form threads in provisional **1.7 mm pilots**; test without forcing. Stop before the PCB bows or the clips touch components.
7. Check that the female jumper connectors can be fitted/removed and that their wires can turn without pressing against the frame, neighbouring boards or buck fan. The modeled housings are envelopes, not a complete wired harness. Provide strain relief and service slack; inspect clearance again with the hood and frame refitted.

The multiplexer carrier sits directly on the deck at Z=112.5 mm; PCB underside is Z=119 mm. The reference includes 24 jumper housings with two wire-clearance guides reaching Z=148.5 mm. These estimates must be checked against the actual connectors and wire bends.

## Print orientation and materials

- All STLs are already in **millimetres**, centred and grounded. Use **100% scale**.
- P04 is supplied flat base down; its cable-channel walls have integral connecting feet.
- C09 is supplied base down; P20 clips have their flat undersides down. No supports are expected for these new parts, but inspect the actual toolpaths, pilot holes and bridges.
- F06 pads are **7 × 0.9 × 0.4 mm**. Prefer thin compliant tape cut to size. Never substitute rigid plastic against the PCB. If printing in TPU, verify the slicer actually produces the intended thickness.
- The unchanged R05 Geekworm cradle/retainers retain their previous print orientation. Large inherited covers/frames still need the supports listed in `PRINT_PARTS.csv`; do not apply the small-part guidance to the entire robot.

Blender assembly coordinates retain **100 mm per Blender unit**. The STL numerical coordinates are millimetres, so no additional scaling is required. Geometry checks are not a substitute for slicer preview, calibrated material settings or real fit tests.

## Hardware count changes

The current `HARDWARE.csv` is authoritative for screw counts:

- M2.5 × 20: **14 → 12**, removing the two MCU screws.
- M2.5 × 10: **4 → 6**, adding the two mux deck screws to the four Geekworm retainer screws.
- M2 × 6: **add two**, for the mux end clips.
- The mux needs two nuts/four washers as above; all other hardware groups remain unchanged. Screw and nut models are schematic, not printable replacements.

## What remains provisional

Only the multiplexer **21.4 × 31.5 mm outline** is dimensioned in the supplied image. The donor model assumes **1.6 mm PCB thickness**, **2.54 mm header pitch**, **18.4 mm row spacing**, **3 mm solder-tail space** and **2.54 × 2.54 × 8 mm female housings**. Its illustrative 3.4 mm PCB holes on 26 mm centres are not used for mounting. The carrier offers nominal 0.3 mm side clearance. Check these dimensions and pad compression on the actual board.

The Geekworm base thickness, terminal overhang and cable dimensions remain provisional as documented in the previous buck update; this merge does not resolve those measurements. Strength, cooling, material temperature and physical fit are still untested.

## Checks and historical documents

`geometry_checks.json` records source hashes, exact removed objects, the imported collection, preserved unrelated geometry, static assembly/connector checks, sampled drawer travel, duplicate-part checks and ray checks of new/filled deck holes. Exported new STLs are re-imported for manifoldness, connected-solid count, positive volume, dimensions and Z=0 placement. All package files have SHA-256 hashes in `manifest.json`.

In the **complete** pack, `historical_R04_reference/` and `Geekworm_mount_R05_reference.md` are historical instructions for unchanged parts. **This R06 guide and the current manifest/CSVs override their MCU references, old part IDs, spacer quantities and deck instructions.** The changed-only pack does not duplicate the historical robot documentation.

This is a mechanical merge only. The multiplexer is not software-configured by these files, and its signal/power harness is not electrically validated here.
