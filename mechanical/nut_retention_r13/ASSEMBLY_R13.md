# R13 — PETG press-in nut retention

## What changes

Replace **P15B** and **P15D** only, relative to R12. All six hood-joint nuts receive retention: H1/H2/H3 in P15B, and H4/H5/H6 in P15D. Keep the R12 P15C smiling front cover, the lower skirt, ToF pockets, camera/LiDAR/frame/electronics mounts, P12 sleeves and P14 plugs. Keep all existing R12 hardware, including the six M3×12 hood-joint screws, six standard M3 nuts and six washers. The separate K frame-mount fasteners are unchanged.

The pocket remains **5.9 mm across flats × 2.8 mm deep**. Six integral ribs project inward from its flat walls. In the supplied **standard** profile, opposing ridge tips are 5.30 mm apart, giving nominal 0.10 mm interference per flat with a 5.50 mm AF nut. Each rib has a 1.20 mm-wide embedded root, a narrow 0.20 mm-wide ridge, and a 0.80 mm-long tapered entrance. The ridges stop 0.05 mm above the pocket floor. The bolt axis, 3.4 mm clearance hole, nut-bearing shoulder and outer part dimensions do not move.

The intended mechanism is local compression of the small PETG ribs as the nut is pressed in. The nut should remain in the pocket before the screw is fitted, without glue. The broad hex walls still resist nut rotation. There is no separate cap, fragile structural snap arm or additional printed assembly component. **Actual retention is printer-, material- and nut-dependent; it has not been physically verified.** This is not a locking nut and does not prevent a threaded fastener from loosening under vibration.

## Print small samples first

Use the same PETG, layer height, extrusion settings and orientation intended for the final covers. Import at **100%, millimetres**, and keep the supplied orientation. Do not auto-orient the samples to make the pocket horizontal: that would not test the relevant layers or support surface.

| Family | Represents | Start with | Optional mating receiver |
|---|---|---|---|
| Q13B | H1–H3 roof tongues on the side panel | Q13B_standard | Existing Q07A |
| Q13R | H4 roof tongue on the rear cover | Q13R_standard | Existing Q11C |
| Q13D | H5/H6 vertical side lap | Q13D_standard | Existing Q12C |

Each family also has `_light` and `_firm` alternatives:

| Profile | Opposing tip spacing | Nominal interference per face of a 5.5 mm nut |
|---|---:|---:|
| light | 5.40 mm | 0.05 mm |
| **standard — used by the supplied covers** | **5.30 mm** | **0.10 mm** |
| firm | 5.20 mm | 0.15 mm |

These nine samples are optional tests, not parts installed on the robot. They replace the old open-pocket Q07B, Q11D and Q12D test files in the R13 pack. The unchanged receiver tests remain in the complete pack; they are not needed for the initial nut-grip check.

1. Slice the three standard samples. Inspect the pocket, rib lead-ins, bolt hole and projecting lap layer by layer. Use local support where necessary, but do not leave support bonded inside the nut seat. Support/first-layer effects can matter more than nominal CAD tolerance.
2. Let the parts cool. Remove supports and strings carefully; do not file away the six ribs. Use ordinary **M3 hex nuts, nominal 5.5 mm AF × 2.4 mm thick**, as modeled. Taller nyloc nuts and other sizes are not substitutions.
3. Align the hex flats and press the nut squarely into the pocket from the open side. Support the back of the tongue/ear close to the pocket while pressing. Do not bend the whole lap, hammer the nut, heat it into place or use a powered driver to force it in.
4. Check the nut reaches the flat seat, does not rock, and cannot fall out when the sample is turned over and gently handled. Check the thread is centered through the 3.4 mm bore.
5. Start an M3 screw by hand through the bore. It should engage freely without pushing the nut out or spinning it. If using the mating coupon, tighten only gently until snug; do not crush the lap or force the seam shut.
6. If the nut drops out, try the firm sample. If it cannot seat with controlled pressure, or the plastic whitens, splits or the ribs shear off, stop and try light. If only a non-standard sample works, the full covers must be regenerated with that profile. If none works, measure your nut and report the results before printing either cover.

Grip may diminish with repeated removal or creep. Keep nuts installed when servicing the hood where possible. Reject a pocket that no longer retains the nut; do not assume a successful CAD fit proves long-term retention or strength. No specified insertion force, torque or holding-force rating is provided.

## Reprint and assemble

- **P15D rear hood:** flat roof down, as supplied. The broad roof plane and existing 5 mm bolted side ears are unchanged.
- **P15B side panel:** outer side toward the bed, exactly as the existing R12/R07 print orientation. Its curved exterior and tongue details can require local supports.
- **P15C front smiling hood:** no reprint is required. Its STL is copied byte-for-byte from R12, including the attached smile and flat roof.

Use local solid material/walls around the joints with your calibrated PETG profile; this revision does not establish a strength rating. Check slicer support, brim and printer exclusion areas. Model bounding-box checks are not sliced-toolpath validation. No G-code is supplied.

Disconnect the battery and external power. Preload H4/H5/H6 into the rear cover and H1/H2/H3 into the side panel **off the robot**, after the sample test has passed. Make sure each nut remains in place during gentle handling. Fit P12 sleeves and keep P14 plugs in their original locations. Secure real cables away from seams and screw tips.

1. Slide P15D horizontally in from the rear (-Y travel) at its installed height, leaving the camera/LiDAR and frame fixed. Loosely fit its rear -X K screw and washer.
2. Slide the unchanged P15C horizontally in from the front (+Y travel), through the camera opening. Loosely fit its front -X K screw/washer. Do not flex the 5 mm side ears as snap clips.
3. Slide P15B inward from the right (-X travel), engaging its roof tongues. Fit the remaining K hardware.
4. Install H1–H4 roof screws/washers and H5/H6 screws/washers from the outside left side. Start all threads by hand before tightening gently and evenly. Do not use screw force to align a misplaced cover or close the designed seam.
5. Check optics, LiDAR rotation, wires, nut seats and screw tips before restoring power. If a nut becomes loose, remove the cover and retrieve it; never leave loose metal hardware near powered electronics.

For removal, remove the appropriate H screws/washers and K screws/washers, support each section by hand, then withdraw **P15B rightward, P15C forward, P15D rearward**. Nuts are intended to stay with their sections. Keep sleeves and plugs safe. There is still no downward slide over the installed camera.

## Digital checks / reproducibility

The generator is `tools/cad/create_nut_retention_r13.py`; the independent checker is `tools/cad/validate_nut_retention_r13.py`. Both run in background Blender with `--disable-autoexec`. The generator defaults to `standard`; `-- --fit-profile light` or `firm` selects another profile for the full covers. The nine fit samples always contain all three profiles. Regeneration must be followed by validation and updated packaging; do not mix files from different runs.

Checks include unchanged original objects outside the two covers, unchanged retained STL hashes, closed/manifold reimport of new STLs, measured rib/taper positions, unchanged nut floors, original-surface containment sampled to 0.01 mm tolerance, protected exterior surfaces, and all three 240 mm removal paths sampled every 0.5 mm with installed sensors/frame. Intentional rib/nut overlap is explicitly measured as the retaining interference, not reported as clearance. Fastener references do not move. The original R12 master is hash-checked as unchanged.

The two edited cover meshes are triangulated for stable export. The original surfaces include some non-planar polygons; re-tessellation can produce small outward surface differences away from the pockets. The checker separately limits those differences to 0.05 mm and records the measured maximum, while retaining the tighter 0.01 mm original-material containment check. This is numerical mesh cleanup, not an intentional change to the cover dimensions or mounting layout. Unedited objects and retained STLs are checked for exact identity.

Limits: sampled rigid geometry is not continuous collision proof, material deformation, retention-force testing, a cable model or slicer validation. The coupons are the required next physical check. Historical guides apply only to unchanged components; their tape/glue-style nut-retention workarounds and old open-pocket coupons are superseded here.
