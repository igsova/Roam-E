# R08 solder-clearance printing and fit test

## Start with P19 — not the full skirt

The sensor-facing pin tips were estimated at **about 1 mm**. R08 provides **2 mm from the PCB face to the channel floor**, giving about 1 mm margin around that estimate. Actual solder width, pin projection, board variant and print accuracy still need checking.

1. Print **this R08 P19** at 100% scale in millimetres. The R07 P19 does not include this change. The coupon is cut from the actual revised front-centre pocket, including the outside bulge and original nut channels.
2. Keep the supplied orientation: the outer face/bulge points toward the bed and the PCB cavity points up. The bulge means the whole outer face is no longer coplanar; inspect local supports. Remove all support from the solder recess and optical aperture without enlarging them.
3. With the sensor disconnected from power, seat the real soldered board. Its optical head should enter the original opening and its PCB/mounting ears should sit on the original plastic supports without rocking. The solder row must not act as the seating surface.
4. Check both optical windows are exposed, then test the unchanged P17 retainer, two P18 pads, and original two M2.5 × 8 screws, nuts and washers. Tighten gently; do not use the screws to force a tilted board flat.
5. If it does not seat freely, stop and measure the pin projection and row position/width. Adjust the local channel, not the whole robot's scale. Do not bend, grind or desolder the sensor just to match this provisional pocket.

**P19 is a trial piece, not a replacement insert for a printed skirt.** Do not try to carve the entire 2 mm clearance into the old skirt: the original wall is thinner than that at the PCB seat and the new model supplies extra material outside it.

## Replacement parts

| Part | Quantity | Change |
|---|---:|---|
| P16A main split skirt | 1 | Seven solder-row recesses and closed exterior bulges |
| P16B side panel | 1 | One solder-row recess and closed exterior bulge |
| P19 trial pocket | 1 optional | Actual updated front-centre pocket section |

Keep the R07 hood, split-joint hardware, P17 retainers, P18 pads, and all other already printed parts. The complete R08 pack has 45 unique STLs / 120 pieces including optional coupons; 42 STL files are unchanged from R07. The changed-only pack contains three STLs. No additional fasteners are required.

The R08 Blender object names for P16A/P16B retain their R07 suffix for continuity; the scene revision and object properties identify the R08 solder modifications. Do not confuse the unchanged name with the old geometry.

## Slice the skirt

- P16A: original bottom down, as exported. Start with a **2 mm brim**, if needed, and verify all toolpaths against the actual K1C printable area. The outward bulges reduce the room for the previous 3–4 mm brim. `PRINT_PARTS.csv` gives measured STL bounding dimensions; fitting the nominal 220 mm square does not validate support branches or excluded bed regions.
- P16B: exterior side toward the bed, as exported. Inspect support under the curved exterior and the new bulge, as well as the original mounting flange and joint details.
- Use your calibrated material/profile, and check the 1.2 mm closed skin is retained in layer preview. No support-volume or print-time claim is made, and no G-code is included. The previous Creality CLI crash has not been resolved or used to validate this revision.
- The bumps have tapered lower transitions to limit new overhangs when P16A is upright. This is not a support-free guarantee; particularly inspect the internal row channel and the existing nut pockets.

Disconnect robot power before fitting boards, wiring, or covers. Keep the same R07 split assembly procedure and original skirt attachment points. Check wheel clearance, cable routing and battery drawer travel manually on the assembled robot before driving it. These checks are still necessary after a printed fit test.

## Geometry and limits

The small optical opening remains 5.6 × 3.6 mm. Sensor positions/orientations and mounting axes are unchanged. Channel width/height are 16.2 × 4.2 mm; 2 mm depth is measured from the modeled PCB face. The closed outer skin is 1.2 mm, and its farthest face is 2.35 mm beyond the original flat optical panel, below the aperture. The board is **not moved inward** to make room.

Digital checks cover manifold connected meshes; STL re-import/scale; unchanged object and STL hashes; sampled PCB seating surfaces, solder clearances and wall thickness; added-material collisions with other visible modeled components; sampled optical rays; and split-panel pair separation. They do not validate solder measurements, strength, creep, print accuracy, waterproofing, real cable motion, actual optical performance or collision-free removal of the complete assembly.

R08 overrides the solder-clearance and P16A brim assumptions in earlier guides. [R07 split-cover instructions](../split_shell_r07/ASSEMBLY_R07.md) otherwise remain applicable. In the full ZIP they are included as `Split_shell_R07_reference.md`.
