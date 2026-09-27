# R10 — hood that assembles around the installed camera

R10 addresses the **assembly path**, not a final-position clash. The previous P15A fits when closed but its camera-opening edge sweeps through the camera when lowered/raised. The previous R07 side split was intended for print-bed fit; its validation did not establish a complete camera-installed hood insertion path.

**Replace only P15A with P15C (front) and P15D (rear). Reuse the printed P15B right-side panel and every other robot part.** No camera, LiDAR, frame, sensor datum or R09 ToF pocket is moved or resized.

![Three-piece hood, separated horizontally around the installed sensor frame](hood_assembly_paths.png)

## Files

- [Complete assembled Blender model](roam_service_hood_r10.blend)
- [Assembly inspection Blender model](roam_service_hood_r10_assembly_inspection.blend)
- [Replacement prints only: P15C, P15D and two optional joint coupons](ROAM_R10_hood_replacement.zip)
- [Complete R10 STL pack](ROAM_STL_R10_Service_Hood_K1C.zip)
- [Assembly, reuse, hardware and printing instructions](ASSEMBLY_R10.md)
- [Design checks](geometry_checks.json) and [independent validation](validation.json)

## How it assembles

1. **P15D rear section:** approach horizontally from behind the robot.
2. **P15C front section:** approach horizontally from in front of the camera; the camera passes through its existing opening. The new left-side tabs and roof tongue locate the seam.
3. **Existing P15B side panel:** approach horizontally from the model's right/+X side, then install the cover screws.

The front and rear sections meet at Y=10 mm through the LiDAR opening, with a 0.4 mm seam. This makes the LiDAR opening open-sided during assembly. The camera opening and smile remain wholly in the front section. No vertical hood drop over either sensor is required.

One additional **M3 × 12 mm socket-head screw, standard M3 nut and M3 washer** secures the new H4 roof lap joint. Two shallow side tabs provide alignment; they are not snap hooks. The three existing H1–H3 joints and four K mounting screws are reused.

The replacement prints are approximately **207.85 × 116.20 × 90 mm** and **207.82 × 104.00 × 90 mm** in their supplied roof-down orientations. They fit the nominal K1C build envelope with a 4 mm brim, but local supports and real slicer exclusions must still be checked. No support-weight or print-time claim is made.

## Verification and limitations

The independent validator checks the new meshes and all three sequential removal paths with the camera, LiDAR, frame and electronics left installed, then reverses that sequence for assembly. It samples each 240 mm horizontal path at **0.5 mm intervals**. Plugs, captive nuts and P12 sleeves move with their associated sections; the moving section's cover screws/washers are removed. The left K screws may hold the still-stationary sections, as checked and described in the assembly guide. P12 contact-only QA copies are shrunk by 0.2% to avoid reporting intentional sleeve/frame face contact; delivered sleeves are unchanged.

All **44 retained STL files are byte-identical to R09**. The complete pack has 48 unique STLs / 123 pieces including optional coupons. Earlier Blender/STL revisions remain untouched. The export of the fused smile uses a 0.001 mm cleanup tolerance for sub-micron Boolean slivers; this is not a voxel remesh or a scaled robot.

These are digital, sampled checks—not a physical fit, strength, continuous-motion or wiring-clearance certification. Print Q10C/Q10D first, then dry-fit the covers without power. Actual cable routing, connector bend radii and print tolerances still need checking on the robot.

Generators: `tools/cad/create_service_hood_r10.py`, `tools/cad/validate_service_hood_r10.py`.
