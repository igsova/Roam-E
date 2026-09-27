# R12 — bolted side connectors: print and assembly

## Replace only two hood sections

Replace **P15C (front smiling hood)** and **P15D (rear hood)** together with the R12 versions. The old thin side tabs and their slots are superseded by two reinforced, bolted overlaps: **H5 lower** and **H6 upper**.

- Rear overlap thickness: **5.0 mm**, replacing the old 1.0 mm locator tabs.
- Front clamping pad thickness: **3.6 mm**; reinforced rear roots join the ears to the shell.
- Screw clearance: **3.4 mm**. Standard M3 nut pockets: **5.9 mm across flats, 2.8 mm deep**, with a 2.2 mm shoulder.
- Nominal initial clearance between overlap faces: **0.30 mm**. Main front/rear seam: **0.40 mm**, unchanged.
- Both bolts insert from the **outside left side** of the robot. The nuts go into the inside of the rear section before assembly.

Keep **P15B**, all lower skirt/ToF parts, camera/LiDAR/electronics mounts, the four P12 sleeves, four P14 plugs and all R11 hardware. The flat roof and attached smiling face are unchanged. No roof ring is reintroduced. Do not try to retrofit the old thin tabs by drilling them; they do not have the new material around the nut and screw holes. Earlier revisions remain intact.

## Additional purchased hardware

For H5 and H6 together:

- **2 × M3 × 12 mm socket-head screws**, length measured under the head.
- **2 × standard M3 hex nuts**, 5.5 mm across flats, approximately 2.4 mm thick.
- **2 × M3 washers**, approximately 6 mm outside diameter × 0.5 mm thick.

The hardware is drawn in the Blender model for clearance checks; it is **not included as printable STL geometry**. Do not substitute taller locknuts or longer screws without checking clearance. Reuse H1–H4 roof hardware and the four K mounting screws/washers.

## Print the small fit test first

**Q12C + Q12D** are an exact crop pair around the lower side connector, with the same orientation relative to the print layers as the full hood. They are optional tests, not extra parts installed on the robot. The retained Q11C/Q11D samples test the unchanged H4 roof joint, not these new side joints.

1. Import Q12C and Q12D at **100%, millimetres**, one or both on a plate. Keep their supplied orientations. Use a brim and local support as needed for the projecting ear.
2. Clear support from the 3.4 mm bore and hex nut recess. Verify the standard M3 nut seats fully without forcing or cracking the pocket; use a small removable piece of tape to retain it if it is loose.
3. Slide the two samples together along the seam direction, rather than bending the ear into place. Verify the overlap clears the pad and both screw bores align.
4. Insert the washer and M3×12 screw from the flat outside face. Start the thread by hand. The nut should be held against rotation by the hex pocket.
5. Tighten only until snug. Do not crush the printed pads or forcibly close the 0.4 mm seam. If tightening visibly bends/cracks a part, or the nut spins, stop and adjust the local fit before printing the full hood.

A coupon does not prove full robot/cable clearance or structural strength. Use the same material and layer settings intended for the covers.

## Print the replacement covers

| Part | Supplied orientation | Approximate dimensions |
|---|---|---|
| P15C | Flat roof down, cavity up | 207.85 × 116.20 × 88.80 mm |
| P15D | Flat roof down, cavity up | 207.82 × 108.00 × 88.80 mm |

- One cover per plate, **100% scale**, no automatic reorientation.
- Both broad roof faces lie on Z=0 in the supplied STLs. Their geometric contact areas remain approximately 118 cm² front / 91 cm² rear.
- **Local support is still needed where indicated by the slicer**, including the attached camera-opening bridge, rounded shoulder transitions, rear vents, roof-joint details and the new projecting side ears. The two small side ears are not claimed support-free.
- Inspect the entire sliced preview, especially the first layers of the ears and nut-pocket roofs. Remove trapped support before test fitting. Do not fill every interior cavity indiscriminately.
- Use your calibrated profile. Consider enough walls/local solid material around the new joints so the nut shoulder is not mostly sparse infill. No material-specific strength or torque rating has been validated.
- A 4 mm brim fits the nominal 220 × 220 mm bounding box; check the actual printer exclusions and support/brim extents. No G-code, measured support consumption or print-time estimate is supplied.

## Assembly — preserve the horizontal paths

**Disconnect the battery and any external power first.** Leave the camera, LiDAR, their clamps and sensor frame in place. Secure real cables clear of the shell seams, nut pockets and screw tips. Keep approximately 250 mm of working space in front, behind and on the right. Coordinates here are the mechanical model's: front is -Y, rear +Y, retained side panel +X, new bolted side -X.

1. **Prepare rear P15D off the robot.** Insert all three captive nuts: H4 roof, H5 lower side, H6 upper side. Small pieces of removable tape can prevent them falling out. Fit the rear -X P12 sleeve; keep P14 plugs with their original positions. Leave all cover screws/washers out initially.
2. **Fit rear P15D first.** Hold it at its final height behind the robot and slide it horizontally toward the front (-Y). Loosely install its rear -X K mounting screw and washer to hold it on the frame.
3. **Fit front P15C second.** Fit its front -X P12 sleeve. Hold the front cover at its final height in front of the camera and slide it horizontally rearward (+Y). The camera passes through the opening; the new 5 mm ears pass behind the front clamping pads. Do not flex them like snap clips. Loosely install the front -X K holding screw.
4. **Fit P15B from the right last.** Keep its H1–H3 captive nuts and two P12 sleeves in place. Slide it horizontally inward (-X), engaging the unchanged roof tongues and lower locators.
5. **Install H5 and H6 from outside.** Place a washer on each M3×12 screw, pass it through the front clamping pad and engage the preloaded rear nut by hand. Install H1–H4 roof screws/washers and the two remaining +X K screws/washers. Start all fasteners before tightening gently and evenly. Keep the designed seam; do not use the bolts to force a misaligned cover into position.
6. Check the camera opening, LiDAR rotation, all wires, plugs, screw tips and nut seating. The covers must not press on optics or connectors. The exterior bolt heads protrude from the side; account for them in physical clearance around the robot.

If a nut falls out, withdraw the front and rear sections as appropriate and reseat it while the cover is off. Do not push loose hardware into the powered electronics cavity.

## Removal / service

Release any actual wiring attached to the covers. Remove **H5/H6 side screws and washers as well as H1–H4 roof screws/washers** and the two +X K screws/washers. Keep the -X holding screws until each section is supported by hand.

1. Pull P15B out toward +X.
2. Support P15C, remove its front -X K screw/washer, and pull it forward toward -Y.
3. Support P15D, remove its rear -X K screw/washer, and pull it rearward toward +Y.

Retain the loose nuts, sleeves and plugs. There is no downward slide over the camera and no forced snapping of the new side joints.

## Digital checks and limitations

See `geometry_checks.json` and `validation.json`. R12 changes only the two hood sections and adds six purchased-hardware reference objects. The other original robot objects and 46 retained STL files are unchanged from R11. The two hood STLs and two new coupon STLs each reimport as one closed manifold solid. The roof remains planar. All three removal paths are sampled at 0.5 mm increments over 240 mm with the modeled frame, sensors, relevant stationary covers/holding screws and moving captive nuts included. The bolts must be removed before moving their sections. Straight 6 mm-diameter screwdriver access is checked at both side bolts and at H4.

These are sampled rigid-mesh checks, not continuous collision proof, a wiring simulation, physical fit validation, strength testing or slicer-toolpath validation. Test the coupons first and dry-fit the actual covers unpowered. Stop if a sensor, cable or frame is contacted.

This guide supersedes the R10/R11 thin-side-locator instructions for P15C/P15D. Historical guides in the full pack apply only to unchanged components.
