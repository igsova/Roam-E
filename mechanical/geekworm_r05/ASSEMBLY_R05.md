# ROAM R05 — Geekworm mount / start here

**Prototype; hardware fit pending delivery.** This revision replaces only the old buck installation in the R04 ToF robot. The new mount retains the Geekworm's supplied plastic base. Do not fasten through the illustrative PCB holes or clamp the circuit board.

## Parts to change

| ID | Quantity | Action / supplied print orientation |
|---|---:|---|
| C02 Geekworm base cradle | 1 | Replaces R04 C02; broad top face on bed, integral feet upward; 62 × 62.4 × 7 mm including ears/feet |
| C08 Geekworm edge retainer | 2 | New; identical copies, flat upper face on bed, lip already oriented; 46 × 8 × 7.2 mm |
| F05 compliant edge strip | 2 | Optional STL template, 46 × 1.2 × 0.6 mm; prefer cut compliant tape, not rigid plastic |
| S01 lower spacer, 2.5 mm high | 2 reused | Remain between electronics deck and new integral feet; full-pack S01 quantity remains 14 |
| Old S05 buck PCB spacer | 0 | Omit the two obsolete 3.8 mm spacers |

STLs contain **one part each**; make the quantity in the filename. Use millimetres and 100% scale. C02 and C08 have already been flipped for printing, so do not match the upright assembled Blender orientation in the slicer. No supports are expected for these small parts in the supplied orientation, but inspect countersinks, small hole bridges and nut pockets layer-by-layer. Use a calibrated material/profile and verify bed adhesion. No G-code is supplied.

For a robot already printed as R04, use only the changed-part package. A fresh full R05 set contains 40 unique STLs / 116 pieces including optional parts. Its `PRINT_PARTS.csv` gives the quantities and support notes for every part. **The inherited large covers and frames still require their documented supports**; the no-support note above is only for the new small parts. All STL bounding boxes fit the K1C's 220 × 220 × 250 mm envelope individually; that does not validate supports, brim or toolpaths.

## New/reused metal hardware

- **2 × M2.5 × 16 mm countersunk screws** for cradle-to-deck mounting. Countersunk screw length includes the head. Reuse the two original underside M2.5 nuts and washers; omit the old top washers. Heads must be flush or recessed, never pushing into the OEM base.
- **4 × M2.5 × 10 mm screws plus 4 × M2.5 nuts** for the two retainers. The nuts go in underside hex pockets in C02. Confirm actual screw heads fit and sufficient thread engagement remains.
- Remove the two old M2.5 × 20 mm buck screws. The full-pack count of that screw changes from 16 to 14; other existing groups are unchanged. The full package contains an updated `HARDWARE.csv`.

Modeled fasteners are schematic, without threads. Nominal bores are 2.9 mm, cradle countersinks 5.7 mm diameter × 1.4 mm deep, and nut pockets 5.4 mm across flats. Printer tolerance and real fastener geometry need checking; do not force-fit nuts into a cracking part.

## Assembly sequence

1. Disconnect battery and all external power before modifying the robot. Remove the hood/sensor-frame assembly as needed for access; do not work beneath a powered fan.
2. Remove the old buck, C02 carrier, two S05 PCB spacers and its two long mounting screws. Retain the existing deck and two S01 lower spacers, plus underside nuts/washers.
3. Fit four new M2.5 nuts into the **underside** pockets in the new C02 cradle before fitting it to the deck. Remove support/bridging debris first. Temporarily retain loose nuts with removable tape if needed, without blocking threads or seating faces.
4. Place the cradle's two integral feet onto the original S01 spacers. Install the two countersunk M2.5 × 16 screws through the cradle, feet, spacers and original deck holes; fit the reused underside washers/nuts. Check screw clearance below the deck and that both upper heads sit flush. Do not drill new holes in P04.
5. Place the Geekworm **with its supplied base still fitted** on C02. Match the Blender inspection view: USB-C output towards the side cable channel, green terminal towards the centre/IMU side. The terminal must remain above and clear of the neighbouring assembly.
6. Put the two thin compliant strips on the outer long base rims. Fit one C08 retainer along each long edge with two M2.5 × 10 screws each. Tighten only enough to retain the plastic base; do not bow the PCB, crush the base, or let a lip touch components. Check resistance to sliding along the open ends by hand—retention is a friction clamp, not a positive end stop.
7. Trial-fit the intended USB-C lead and DC terminal/wires. Check plug insertion/removal, switch access, strain relief and actual cable bends. Secure wiring so it cannot reach the fan, rub the IMU or strain the connector. USB-A simultaneous-use clearance has not been established by a dedicated plug guide.
8. Trial-refit the sensor frame and hood. Check the complete fan inlet/exhaust path, avoid blocked vents, and ensure no wire is trapped. Only then plan the separately verified electrical installation and temperature/load tests.

## Confirm when the converter arrives

The provided drawing specifies a 59 × 44 mm base and 22 mm assembly height, but shows the barrel-input variant. The purchased terminal variant is represented using photographic estimates for its unmarked details.

- Measure base length, width, **thickness (assumed 5 mm)** and corner radius. Inspect the outer rim: the retainers need approximately 1.2 mm of sound plastic ledge beyond the PCB along the two long edges.
- Check the assembled height, underside protrusions, fan location and whether the supplied base is suitable for clamping. Do not remove its base merely to make this cradle fit.
- Measure green-terminal protrusion (**assumed 9 mm beyond base**) and unplugging/wire space. Check the real USB-C plug size and cable bend, not just the socket.
- Test C02/C08/pads on the loose module first. If it does not sit flat or the lips cannot grip without PCB contact, revise the model before installation. Do not compensate by overtightening screws.
- Recheck fan airflow and material temperature in the enclosed robot. A digital 12 mm inlet keep-out is not a cooling-performance test.

`dimensions.json` records estimates; `R05_buck_joints.json` lists assembled datums and mounting axes. `geometry_checks.json` documents the modeled checks, and the manifest records STL dimensions, topology checks and hashes. Actual strength, heat resistance, fit and slicing remain untested.

## Inherited R04 instructions

The full package includes historical R04 instructions/images in `R04_reference/` for unchanged chassis, covers, camera, drawer and ToF parts. **This R05 guide overrides their old buck instructions, C02/S05 quantities and buck fastener lengths.** The historical joint JSON is not the new buck joint specification. The changed-only package does not duplicate the historical robot instructions.

Purchased electronics, metal fasteners and the conservative clearance envelope are not printable replacements. The separate `reference_only` STL is intentionally absent from both print packages.
