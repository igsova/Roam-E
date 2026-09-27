# R11 flat-roof service hood — print and assembly guide

## What to reuse

**Do not reprint the whole robot.** Replace only R10 **P15C + P15D** with their R11 versions below. If you still have the earlier P15A, these two sections replace that single main hood:

- **P15C — front slide-on smiling hood**, one piece.
- **P15D — rear slide-on hood**, one piece.

Keep **P15B**, both lower skirt pieces, all electronics and sensor mounts, the camera clamps/pads, LiDAR adapter and all existing hardware. The corrected R09 ToF parts remain unchanged. Q11C/Q11D are small optional H4 joint-test prints, not parts fitted to the robot. Existing Q07 samples are still valid for the unchanged P15B joints.

Do not simply saw the printed P15A into two pieces: R10 includes a reinforced roof lap joint, nut pocket, screw seat, deliberate seam gap and side locating tabs/receivers that the original print lacks. Keep the old part as a spare/reference; no destructive conversion is required.

## What changed from R10

- The raised ring around the LiDAR opening is removed from P15C and P15D.
- Their broad exterior roof faces share a single **Z=190.80 mm** plane in the assembled model. In the supplied STLs that face sits at **Z=0** on the bed.
- The former 192 mm high rim is gone; printed height is now **88.80 mm**, not 90 mm. This is a local roof edit, not scaling of the robot.
- The camera opening and smiling lower face stay attached. No separate face part or extra hardware is introduced.
- Roof-side washer seats, bores, inner skin, nut pockets and sliding-joint datums are retained. The broad roof has more than **2.3 mm** thickness in a 2 mm-grid digital sample, rather than being shaved down to a thin skin.
- Rounded perimeter shoulders and the old P15B side panel are deliberately retained; they are not part of the planar central roof. The retained side panel can have a small height transition to the new plane.

**Local support is still allowed and needed where your slicer indicates it.** In particular, keeping the face attached leaves the long camera-opening bridge. Rear vents, perimeter shoulders and lap-joint details can also require local support. This revision removes the ring-induced roof stand-off, not every overhang in the hood.

## Hardware — unchanged from R10

For the existing **H4** roof joint (reuse your R10 hardware; if upgrading from R09, add):

- 1 × **M3 × 12 mm socket-head screw** (length under the head).
- 1 × standard **M3 hex nut**, 5.5 mm across flats, approximately 2.4 mm thick.
- 1 × **M3 washer**, approximately 6 mm OD × 0.5 mm thick.

H4 is at assembled X=-85, Y=4 mm, on the roof on the opposite side from the existing three H1–H3 screws. Its screw is fitted vertically from above. R11 leaves its washer seat and nut shoulder unchanged. Do not substitute a thick locknut or a longer screw without checking fit.

Reuse the existing three H1–H3 M3 × 12 roof screws/nuts/washers and four K M3 × 16 side mounting screws/washers. Their original frame captive nuts stay installed. Keep all four P12 locating sleeves and four P14 service-port plugs.

## First: test H4 with the two small prints

1. Print Q11C and Q11D at **100%, millimetres**, in their supplied orientations. They are crops of the actual joint.
2. Clear local support from the bores and tongue receiver. Do not enlarge the parts by globally scaling them.
3. Insert the M3 nut from the underside of Q11D. The nut pocket is 5.9 mm across flats / 2.8 mm deep.
4. Slide the parts together horizontally along the tongue. Nominal clearance is 0.30 mm per mating face; the main seam is 0.40 mm. Do not squeeze that gap closed.
5. Fit the washer and M3 × 12 screw from the roof side. Tighten gently by hand. Verify full nut engagement without bottoming, cracking or bending the parts.

If the joint binds, adjust local tolerances before printing the full covers. The coupons do not prove camera clearance or wire routing.

## Print the two replacement sections

| Part | Supplied print orientation | Approximate dimensions |
|---|---|---|
| P15C | Roof down, cavity up | 207.85 × 116.20 × 88.80 mm |
| P15D | Roof down, cavity up | 207.82 × 104.00 × 88.80 mm |

- One section per plate; **no scaling or automatic reorientation**.
- Use your calibrated material profile. A 0.4 mm nozzle / 0.2 mm layers / 3 walls is a starting point, not a tested material specification.
- A 4 mm brim fits the nominal 220 × 220 mm bed bounding box. Check the actual K1C exclusions and all support/brim paths in your slicer.
- Keep the supplied roof-down orientation. The broad flat face should be on the bed, not suspended on a ring or a raft.
- Inspect layer 1: the front has approximately **118 cm²** and rear **91 cm²** of geometric planar bed contact (holes excluded). These are mesh areas, not measured extruded first-layer areas.
- Use local/painted supports as needed under the **camera-opening bridge**, **rounded perimeter transitions**, **rear vent edges** and **joint details**. Inspect how every island starts. Do not turn all supports off just because the central roof is flat.
- The broad planar roof itself does not need a support layer underneath it. The design as a whole is **not claimed support-free**.
- Keep the 3.4 mm screw bore, nut recess and small locator channels clear. Do not fill the whole cavity indiscriminately with difficult-to-remove supports.
- The smile is fused into P15C's STL. STL has no material colors; paint it if desired. Blender retains the separate colored reference.

No G-code is supplied. Support consumption and print time have not been measured in the slicer.

## Assemble around the camera — no downward hood slide

**Power off and disconnect the battery and external supply before assembly.** Leave the camera, LiDAR, their frame and clamps installed. Give yourself roughly 250 mm working room in front, behind and on the side of the robot. In the model, camera-facing/front is -Y, rear is +Y, and the retained side panel is +X; these are mechanical scene coordinates, not a change to ROS frames.

Before starting, remove/leave out all four K cover screws and washers and the H1–H4 cover screws/washers. Do not remove the camera-clamp screws or frame screws. Keep the frame's K captive nuts seated. Park actual wires inside the frame and away from shell seams; check their connectors and service slack manually.

### 1. Rear section P15D first

- Seat the H4 nut in P15D's underside pocket while the section is off the robot. A tiny removable piece of tape can retain it; do not obstruct the mating faces.
- Fit the rear, -X-side P12 sleeve to its original hood recess. The rear P14 plugs can be fitted now or after the covers are seated.
- Hold P15D at its final height and orientation, behind the robot. Move it **straight forward horizontally (-Y)** into position. Its open half of the LiDAR aperture passes around the installed LiDAR.
- Once it is seated, install its existing rear **-X-side K screw and washer** (Y=+55 mm) loosely to hold it on the frame. Do not clamp it tight yet. This screw stays clear of the subsequent front/side insertion paths.

### 2. Front section P15C second

- Fit the front, -X-side P12 sleeve. Keep the camera opening free of loose wires, tape or printing debris. Front P14 plugs can be fitted now or later.
- Hold P15C at its final height in front of the robot, with its camera opening aligned to the camera. Move it **straight rearward horizontally (+Y)**. Do not lower it over the camera and do not tilt it into the lenses.
- The camera body passes through the existing opening; the left-side tabs and H4 roof tongue enter their receivers. Keep the 0.4 mm seam, rather than forcing the cut faces together.
- Once seated, install the existing front **-X-side K screw and washer** (Y=-55 mm) loosely. Leave H4 out until the final panel is seated. If anything catches, stop and withdraw horizontally. Identify the contact instead of pushing on the camera or loosening its calibration mounts.

### 3. Reuse side panel P15B last

- Keep its original H1–H3 captive nuts seated and its two P12 sleeves fitted.
- Bring the panel **straight inward from +X (-X travel)**. Its three original roof tongues and two lower locators meet the same original interfaces, now distributed between P15C and P15D.
- After all sections are seated, loosely install the H4 screw/washer, the H1–H3 screws/washers and the two remaining +X-side K screws/washers. Engage threads by hand before tightening all screws gently and evenly.
- Check the camera has clearance all around, the LiDAR can rotate unobstructed, plugs are seated, and no wire is trapped. The cover is not a clamp for either sensor.

Each section needs hand support until its holding screw is engaged. Do not leave loose shells resting on the electronics.

## Removal / service

Release any real harness attached to a cover. Remove H1–H4 roof screws/washers and the two +X-side K screws/washers first. Keep the two -X-side K screws supporting their sections until their turn. Then:

1. Pull P15B horizontally out to +X.
2. Support P15C, remove its front -X-side K screw/washer, and pull it horizontally forward to -Y.
3. Support P15D, remove its rear -X-side K screw/washer, and pull it horizontally rearward to +Y.

Keep the sleeves/plugs/captive nuts with their corresponding parts. There is no vertical lift over the camera. Never move a section with one of its own screws still engaged.

## What was checked, and what still needs physical checking

The Blender sensor/body poses and all objects other than the two hood sections are unchanged from R10. All 44 retained STLs are byte-identical, including P15B and the R09 ToF skirt parts. Both hood STLs and two optional coupons reimport as one closed manifold solid each, grounded at Z=0 with verified planar first-layer contact. Original hood vertices below the modified roof band are checked against the new meshes. Each of the three new removal paths is sampled over 240 mm at 0.5 mm intervals, with the installed frame, camera, LiDAR, electronics, relevant stationary covers and carried sleeves/plugs/captive nuts included. Holding K screws on the still-stationary front/rear sections are included at the stages described above. The same paths reversed provide the assembly sequence. H4 has a clear modeled 6 mm-diameter vertical driver-access envelope above the head.

This is not continuous collision detection, physical tolerance/strength validation or a wiring simulation. All modeled geometry is rigid. Camera dimensions, actual cable loops, printer accuracy and shell warping may differ on the real robot. Verify the coupons and a careful unpowered dry fit. Stop if optics or connectors touch a cover.

Earlier R07 instructions saying to assemble the hood and refit it as one piece are **superseded for R10 and R11**. Historical documents only describe unchanged parts. Current `manifest.json`, this guide and `validation.json` take precedence for the hood.
