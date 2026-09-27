# R17 — three-piece lower shell and curved switched drawer

**PETG mechanical prototype. Print the small joint/switch tests first.** The
switches' exact snap fit and DC switching rating have not been verified.
This package is not a wiring design or an approval to switch motor/battery
current with the photographed AC-rated rockers. Read `POWER_NOTES.md`.

## What to replace

| R17 part | Replaces / purpose |
|---|---|
| P16C | Front portion of old P16A, with the three front ToF stations |
| P16D | Rear portion of old P16A, with three rear stations and the left station |
| P16B | Right lower panel, right ToF station, WHEELS switch, retained S1/S2 nuts |
| P05 | Curved battery drawer with BATTERY switch and closed finger recess |
| P06L + P06R | Matching guide ends and nut traps for the curved drawer |

Use all three matching skirt sections together. **P16A is obsolete in this
revision. Reprint both P06 guides with P05:** the old guide ends project into
the curved fascia. The rail/channel cross-section and fastening axes are
retained; their rear ends are shortened to clear the fascia.

The upper hood, camera, LiDAR, electronics, battery position, base mounting
axes, all eight ToF modules, retainers and pads are unchanged from R16. The
57 carried-over STLs are copied byte-for-byte. Do not reprint them for R17.

The drawer front follows the lower skirt's superellipse contour instead of a
flat chord. Local flat lands intentionally remain beneath the switch bezel
and two lock washers. The new fascia retains its original width and height;
it is not a scale reduction. Two shallow 7 mm-wide longitudinal underside
channels clear the existing modeled caster cap heads during withdrawal.
They retain nominal 1.6 mm floor thickness below the unchanged battery liner.

## Fasteners and glue-free nut retention

| Joint | Hardware | Relationship |
|---|---|---|
| S1 | Reuse M3×12, standard nut, washer | Front skirt ↔ right panel |
| S2 | Reuse M3×12, standard nut, washer | Rear skirt ↔ right panel |
| S3 + S4 | Add 2 × M3×12, 2 nuts, 2 washers | Front skirt ↔ rear skirt, left upper seam |
| E-left + E-right | Reuse M3×12, standard nuts, washers | Drawer fascia ↔ revised fixed guides |
| M1–M4 | Retain existing mounting hardware | Skirt ↔ chassis; these IDs are not M4 thread sizes |

Standard nuts are modeled at **5.5 mm across flats × 2.4 mm thick**, with
6 mm OD × 0.5 mm washers. Do not substitute taller nyloc nuts without checking.
The S joints have 5 mm ears, 3.4 mm bores, 0.3 mm receiver clearance and
tapered PETG crush ribs, nominal 5.30 mm between opposing tips. S1/S2 keep
their previous bolt axes. S3/S4 are new; their seam is at Y = −4 mm with a
0.4 mm gap. There are no unbolted alignment tongues at the new seam.

Preload the S1/S2 nuts into the right panel and S3/S4 into the rear section
from the inside before fitting the covers. The revised E nuts load from the
top of each guide: orient the parallel nut flats against the slot sides,
press down to the stop, and check alignment with an unpowered loose screw.
The E slots use two local crush ribs. All retention remains a physical fit
test, not a guarantee against PETG creep. Do not hammer nuts into the plastic
or overtighten bolts to compensate for a bad fit.

## Switch dimensions and fit tests

The two purchased switches are **reference geometry only**, not STL parts.
The supplied drawing was interpreted as a 23 mm bezel, 20.3 mm upper body,
approximately 18 mm lower body, 26 mm depth to terminal tips, and 4.8 × 0.8 mm
blades. Clips and actuator shape are approximations. The listing does not
specify a certified cutout or allowable panel thickness.

The prototype uses a **20.5 mm circular hole and a 2.2 mm flat snap seat**,
with a larger clearance behind the seat so the clips can expand. Forty
millimetres of interior depth is reserved from the mounting plane, but the
actual solder joints, insulation and wire bend are not modeled or approved.

Print in the same PETG, orientation and slicer settings intended for the parts:

1. Q17A + Q17B: actual new seam receiver/ear. Check screw passage, nut retention,
   and full closure without forcing the parts.
2. Q17C + Q17D: actual drawer/panel switch seats, Ø20.5. Insert the real switch
   from outside. Check that the bezel sits flat, both clips engage, the body
   does not rock, and it can be serviced by squeezing the clips from inside.
3. Q17E / Q17F: optional Ø20.3 / Ø20.7 alternatives, both 2.2 mm thick. These
   compare diameter only; **the large parts still use Ø20.5**. If an alternative
   works better, revise/regenerate the large parts before printing them.
4. Print one P06 guide and check its top-loading E nut pocket. Check the real
   switch body depth and insulated terminal/wire envelope before large prints.

If the clip does not latch, do not assume a larger hole will fix a panel that
is too thick. Measure the actual switch and revise the seat. Do not glue or
force a poor switch fit. Add durable BATTERY and WHEELS labels to the real
robot so the two controls cannot be confused.

## Printing and assembly

STLs are already millimetres at 100%, with bed placement and orientation set.
All three skirt panels and P05 are bottom-down. Q17A–D are upright to match
their parent parts; Q17E/F are flat diameter-comparison discs, not full
orientation-matched clip tests. Use at most a 2 mm brim on P16D as a geometric starting point;
check the K1C's exclusion zones and all generated supports. Larger brim or
automatic support margins may exceed the bed even though the part fits.

Local supports may still be required at the wheel arch, sensor recesses,
switch counterbores, finger recess and nut features. This is not a
support-free claim or a tested slicer profile. Keep bores/clip pockets clear,
and check first-layer contact and layer preview before printing.

Disconnect the battery and all external power for mechanical work. Install
the matched guides on the original chassis axes. Fit/test the lower sections
on the bench, keeping the ToF harness and switches accessible. Retain the
original M mounting feet and screws. Joint separation is checked geometrically,
but a collision-free removal path around every installed pylon/deck/cable is
not certified; do not force a section past an obstruction. Remove obstructing
assemblies as needed. Upper-hood installation instructions remain R16's.

The switches are inserted from outside, with their clip releases accessible
inside. Do not leave uninsulated terminals facing the battery or frame. The
drawer switch must have a service disconnect and strain-relieved slack;
never pull the drawer by its wiring. After releasing the two E screws, draw
it out only far enough to disconnect the harness, then withdraw it fully.
Refit slowly and verify no wire crosses a rail, ToF pocket or wheel opening.

Before driving, check tire clearance, all sensors, drawer movement, switch
retention and the distinction between motor-power-off and battery-off.
The power circuit must be independently reviewed/tested first. Gracefully
shut down the Pi before cutting its supply; external USB power can keep it on.

## Verification limits

See `validation.json` and `export_checks.json` for the actual run status.
Checks include single closed solids, source-object identity, unchanged ToF
pin-row/solder datums, nut-rib and switch-seat measurements, reserved switch
space, static interference checks, sampled 150 mm drawer withdrawal and
200 mm shell-joint separation, STL reimport and nominal K1C bounds.

Contact-volume tests shrink the tested reference object by 0.1% to avoid
numerical artifacts at coincident bearing faces; retained legacy contacts
and intentional nut-rib interference are listed separately. Motion sampling
uses full-size surface meshes at 0.5 mm intervals, not a continuous collision
proof. Physical fit, grip, strength, PETG creep, unmodeled wiring, actual
switch tolerances and slicer toolpaths remain to be verified.
