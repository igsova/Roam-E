# R14 — P15B end connectors are bolted, not thin tabs

## Replace the three matching hood parts

The two small P15B locators shown at the panel's front and rear edges in R13 are superseded by **H7 front** and **H8 rear**. Their old projecting tabs are removed. Each new joint has:

- A **5.0 mm-thick lap ear** integrated into P15B, with a broad reinforced root following the curved shell.
- A matching **3.6 mm clamping pad** on P15C (front) or P15D (rear).
- Nominal **0.30 mm mating clearance**, **3.4 mm bolt bore**, and **2.2 mm nut-bearing shoulder**.
- A **5.9 mm AF × 2.8 mm deep** M3 nut recess, with six tapered retention ribs using the R13 **standard PETG** profile (5.30 mm between opposing ridge tips).

Print **P15B, P15C and P15D from this revision together**. A new P15B cannot use these bolts with old front/rear covers: those lack the matching holes and receivers. Do not drill the former thin tabs or force mixed revisions together. The lower skirt, ToF mounts, sensor frame, camera, LiDAR, electronics mounts, P12 sleeves and P14 plugs are unchanged. The flat roof and attached smiling face remain.

## Two additional fasteners

For **H7 and H8 only**:

- **2 × M3×10 mm socket-head screws**, length measured below the head.
- **2 × standard M3 hex nuts**, nominal 5.5 mm across flats × 2.4 mm thick.
- **2 × M3 washers**, nominal 6 mm outside diameter × 0.5 mm thick.

**Do not substitute M3×12 or longer at these new joints.** The 12 mm front screw collided with the modeled right sensor frame; the 10 mm screw passes through the complete nut with approximately 1 mm protrusion. Keep the modeled washer thickness. Taller locknuts are not compatible with the standard pocket.

Reuse all existing H1–H6 and K hardware unchanged. H1–H6 still use their existing M3×12 screws; the new shorter length applies only to H7/H8. Purchased bolts, nuts and washers appear in Blender for clearance reference and are not included in printable STL geometry.

## Print the small actual-joint tests first

| Pair | Joint | Roles |
|---|---|---|
| **Q14A + Q14B** | H7 front | Front-cover clamp / side-panel nut lap |
| **Q14C + Q14D** | H8 rear | Rear-cover clamp / side-panel nut lap |

These are crops of the actual revised joints, supplied in the same layer orientations as their parent covers. They are optional fit tests, not extra components installed on the robot. Use the intended PETG and slicer profile, import at **100% scale in millimetres**, and keep the supplied orientations. Inspect local supports at the ear, pocket, curved base and bore; a brim/support may be needed for these small crops.

1. Remove support carefully from the nut seat and bolt hole without shaving off the retention ribs. Let the samples cool before fitting hardware.
2. Align and press a standard M3 nut squarely into each side-panel sample, supporting the ear immediately behind the pocket. The ribs should grip the nut so it stays seated when inverted and gently handled, without glue. Do not hammer, heat or forcibly bend the ear.
3. Slide the pair together along its seam direction. Check the lap passes behind the clamp pad without bending, and both bolt bores align. The small nominal clearance is intentional.
4. Fit the washer and **M3×10** screw from the outside clamp face. Start the thread by hand; tighten only until snug. Do not use tightening force to align the covers, crush the pads or eliminate their designed seam.
5. Stop if a nut spins, drops out, will not seat, or cracks the plastic. The complete pack retains the R13 light/standard/firm nut-fit samples. If another grip level is needed, regenerate the affected pockets before printing the full covers; do not scale the whole cover.

Press-in retention is not a locking nut or proof against vibration loosening. Its grip, insertion force, repeated-removal life and PETG creep have not been physically tested. The digital model intentionally overlaps the nominal metal nut flats with small compressible ribs; it does not simulate deformation.

## Full-cover printing

| Part | Orientation | Approximate supplied STL dimensions |
|---|---|---|
| P15B | Outer side toward bed, same orientation as before | 88.04 × 174.90 × 30.00 mm |
| P15C | Broad flat roof on bed; smile remains attached | 207.85 × 116.20 × 88.80 mm |
| P15D | Broad flat roof on bed | 207.82 × 108.00 × 88.80 mm |

Use the supplied orientations, 100% scale, and a calibrated PETG profile. P15B retains a curved outer surface, not a new flat bed face. Use brim/local support where your sliced preview requires it. The front camera bridge, shoulders, vent and joint overhangs may also require local support. Nut pockets and screw holes must be accessible for cleanup. The added ears are not claimed support-free.

Nominal 220 × 220 mm bed fit is checked with 4 mm brim allowance, but support extents, printer exclusions and actual toolpaths must be checked in your slicer. No G-code, print-time estimate or material-strength/torque rating is provided.

## Assembly and removal

Disconnect the battery and all external power. Leave the camera, LiDAR and sensor frame mounted; keep cables clear of seams, nut pockets and screw tips. Allow roughly 250 mm working room in front, behind and on the right.

1. Off the robot, preload **H4–H6 nuts into P15D**, and **H1–H3 plus H7/H8 nuts into P15B**. Verify they stay seated during handling. Install P12 sleeves and keep P14 plugs in their original positions.
2. Fit P15D horizontally from the rear, then loosely secure its rear-left K frame fastener.
3. Fit P15C horizontally from the front, allowing the camera to pass through its opening. Loosely secure its front-left K fastener.
4. Slide **P15B horizontally inward from the right**. Engage the roof tongues and both new end laps together. They are rigid bolted overlaps, not snap clips; do not flex or force them.
5. Install **H7 from the front exterior** and **H8 from the rear exterior**, with their washers and M3×10 screws. Fit H1–H6 and remaining K fasteners. Start all threads by hand before tightening gently and evenly.
6. Verify camera clearance, unobstructed LiDAR rotation, wiring, full nut seating and screw-tip clearance before powering up. Retrieve any loose metal nut immediately while power is disconnected.

For service, remove **H7 and H8 screws/washers as well as H1–H6 and the appropriate K fasteners** before moving the covers. Withdraw P15B to the right, then P15C forward, then P15D rearward, supporting each section and removing its holding K fastener as needed. Nuts should remain with their sections. Never pull P15B with either new end bolt still installed.

## Verification and limits

The generator is `tools/cad/create_panel_bolts_r14.py`; the independent checker is `tools/cad/validate_panel_bolts_r14.py`. The source R13 master is hash-checked and never overwritten. New STL meshes are reimported and checked for a single closed manifold solid, scale, bed fit and orientation. Unchanged objects and the 53 retained STL files are checked for identity. The three new cover STLs and four Q14 samples are the seven new/replaced files.

Checks cover all eight nut-pocket profiles, modeled screw/washer/nut clearance, 6 mm-diameter screwdriver approach at H7/H8, and each of the three 240 mm removal paths sampled every 0.5 mm with the fixed camera, frame and LiDAR present. The new screw tips are checked against the fixed robot. Earlier roof and opposite-side joints remain intact.

Changes are confined to the two end-joint regions within the documented mesh tolerances. Original non-planar surface triangulation can differ by small fractions of a millimetre; the report records actual surface errors. Digital checks are not continuous collision proof, wiring simulation, physical retention/strength testing or sliced support validation. Test both coupon pairs and dry-fit the real covers unpowered.

This guide supersedes older instructions that describe P15B's end tabs as passive locators. Historical guides apply only to unchanged components.
