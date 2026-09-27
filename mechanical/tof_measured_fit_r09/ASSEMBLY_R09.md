# R09 measured ToF fit — print and assembly

## Print this revision's P19 first

Use the **R09 P19** test pocket, not the previous R07/R08 P19. It is cut from the actual revised front-centre skirt pocket. It tests the combined narrower board cavity, relocated solder channel, optical opening, and unchanged P17/P18 retention system.

1. Import P19 at 100% millimetre scale. Keep its supplied outer-face-down orientation, with the cavity upward. Preview local support under the closed exterior bump and in the original nut-entry features; the outer face is not entirely coplanar with the bed.
2. Remove all trapped support, especially from the solder channel and optical opening. Do not enlarge features to force a fit.
3. With the sensor disconnected from power, place the real soldered module in the pocket. The optical package must enter the existing aperture; the PCB and mounting ears must sit flat on the plastic without rocking on solder. The measured pin row is 4.8 mm from the nearest sensor-package edge, not its centre.
4. Fit the existing P17 retainer, two P18 pads, two M2.5 × 8 screws, nuts and washers. Tighten gently; do not use screw force to pull a tilted PCB flat. Check that both mounting ears are captured and the board cannot slide enough to obscure either optical window.
5. Check the pin-side edge has clearance and the optic-side edge/ears also fit. The two new measurements do not independently locate the mounting ears or confirm board thickness. If the board binds or retention is ineffective, stop and measure those datums before printing either full skirt half.

The pin tips were previously estimated at about 1 mm high. The channel still provides 2 mm depth measured **above the sensor-facing PCB plane**. This is a nominal CAD clearance, not a confirmed printed measurement.

P19 is a standalone trial piece, not an insert for an existing skirt. Do not cut the whole new channel into an old skirt: the new exterior bump supplies wall material that the old shallow pocket lacks.

## Parts and orientation

| Part | Quantity | Action |
|---|---:|---|
| P16A | 1 | Reprint with seven corrected ToF pockets; original bottom down |
| P16B | 1 | Reprint with the remaining corrected pocket; outer +X side down |
| P19 | 1 optional | Print first; actual corrected pocket section |

All other 42 STLs are unchanged from R08, including the R07 hood, split joints, P17 retainers, P18 pads and Q07 joint samples. No new hardware is required. The complete package has 45 unique STLs / 120 physical pieces including optional trial pieces.

Use a **2 mm brim as a starting point for P16A**, if your material needs one. Check the actual printable bed/exclusion zones, brim gap and supports in the GUI slicer. `PRINT_PARTS.csv` records the exported bounding dimensions. Geometric fit within the nominal 220 mm K1C bed does not validate toolpaths. Do not scale or auto-orient the parts.

Inspect the moved solder recess, closed bump, 1.2 mm exterior skin, nut channels, mounting flanges and split-joint details in layer preview. No support-free, support-weight or print-time claim is made; no G-code is included. The previous Creality CLI failure is not resolved by this CAD revision.

## What changed and what stayed fixed

- PCB short dimension: **12.60 → 10.66 mm**. The pin-side pocket wall moves 1.94 mm inward, producing an **11.26 mm cavity** with 0.30 mm nominal edge clearance.
- Pin-row centre: **7.90 → 6.00 mm** from optical centre, derived from **4.80 mm edge-to-row + 1.20 mm sensor half-width**. The Blender pad/pin references, channel and closed external bump all move 1.90 mm toward the optics.
- Solder clearance remains **2.00 mm above the PCB face**, with a **1.20 mm exterior skin**. Channel width × height stays **16.2 × 4.2 mm**.
- The original optical aperture and sensor/world pose do not move. The PCB seat plane, circular mounting ears, P17/P18 parts, screw axes and R07 split joints stay unchanged. R09 restores the obsolete pocket area rather than leaving both the old and new large channels in the shell.
- Board length, thickness, ear geometry and the optical-side PCB edge remain inherited assumptions. The actual module must pass P19 before those assumptions are accepted for printing.

The P16A/P16B Blender names retain their R07 suffix for continuity; scene revision and custom properties identify R09. PCB reference names explicitly say **25x10.66 R09**. These boards are purchased-part references and are not printable STLs.

## Reassemble and inspect

Follow the [R07 split-cover assembly instructions](../split_shell_r07/ASSEMBLY_R07.md), provided as `Split_shell_R07_reference.md` inside the ZIP. R09 supersedes older ToF pocket dimensions and P16A brim guidance. Disconnect all power while fitting boards and wiring. Reuse the original skirt attachments as well as the split-joint screws; unplug harnesses before removing a panel.

Check wheels, cables and battery-drawer movement manually before driving. Digital geometry checks cover closed connected meshes, actual exported STLs, modeled board/pin clearance, unchanged seating patches, added-material/component clearance, optical rays and sample split-panel separation. They do not prove printed fit, strength, creep, solder shape, cable motion, optical performance, weather sealing or collision-free disassembly of the complete robot.
