# R16 — close the camera surround to the hood

## Replace only P15C relative to R15

Print **P15C_closed_camera_front_hood_R16_qty1.stl**. Keep the R15 P15B side panel, P15D rear hood, all sensors/mounts/pads, and all bolts and nuts. The other **61 STL files** in the complete pack are retained byte-for-byte from R15. No new fasteners or loose filler pieces are required.

The old camera surround ended before it reached the curved shell at its ends. A closed, manifold mesh can still contain this kind of unwanted opening: the earlier topology and collision checks did not prove that the surround enclosed the camera. R16 adds continuous hollow return walls that merge the rear of the surround into the hood. The clear camera-face opening is retained, rather than filled over.

The local additions stop within the original shell skin, preserving room for the camera supports. The roof, LiDAR opening, three-piece assembly arrangement, bolted retained-nut joints and attached smiling face remain. R15 is preserved for history; do not use its old P15C for a new print.

## Print and assemble

- Use **millimetres, 100% scale**, and the supplied roof-down orientation. The flat roof is unchanged.
- Use your calibrated PETG profile. The attached camera bridge/surround and joints can still need **local support**. Inspect sliced layers and generated support extents; this revision is not claimed support-free.
- Disconnect all power before assembly. Remove the right side panel first, then withdraw the old front hood horizontally forward after removing its fasteners. Keep the camera and LiDAR installed.
- Fit the R16 front hood horizontally from the front and reinstall the retained R15 side panel. Reuse the existing H/K fasteners; H7/H8 remain **M3×10**, not M3×12. Tighten gently, with no force needed to align the parts.
- Verify the camera body, rear bracket, cable and optics remain clear. The face opening is still nominally **162.2 × 24.0 mm** for the measured **161 × 22.8 mm** camera; the existing Q15A opening sample remains applicable. Do an unpowered dry-fit before final tightening.

If coming from a revision earlier than R15, follow `R15_ASSEMBLY_REFERENCE.md` first: its raised side/rear panels, nominal 1.20 mm upper compliant pads, measured 44.78 mm brass posts directly on P10, and omission of P11 spacers still apply. Do not mix an older-height hood with R15/R16 pieces.

## Checks and limits

The new camera-end coverage test samples both ends in side view on a **0.5 mm Y/Z grid**, against the actual polygonal silhouette. The intentional H7 sliding-lap receiver is excluded from the wall-coverage test so its assembly clearance remains open. The test must detect the R15 opening and pass on R16. It is repeated on the **exported STL after reimport**, not just the Blender scene.

Additional checks cover a single closed manifold front mesh, unchanged-object identity, local surface preservation, all eight nut-seat profiles, flat roof area, fixed-sensor clearance and the three removal paths sampled every 0.5 mm over 250 mm. The exporter validates scale, bed placement, nominal K1C bed fit with a 4 mm brim allowance, retained-file hashes and ZIP contents. Actual results are in `validation.json`, `camera_closure_checks.json` and `export_checks.json`.

The local closure region is allowed to be reshaped at intersections. Outside it, sampled front-hood surface deviation is approximately 0.051 mm maximum at an existing rear cavity edge after retriangulation, below the declared 0.06 mm tolerance. Exact joint-profile and modeled clearances are checked separately; no other object's geometry changes.

These are sampled digital checks, not physical fit, print-strength, dust/water sealing, continuous collision or optical-FOV certification. The normal removable-panel seam remains intentional. Cable routing, PETG tolerances and sliced supports must still be checked on the actual assembly.
