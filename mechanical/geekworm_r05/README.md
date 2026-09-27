# ROAM R05 — Geekworm power module

Mechanical prototype dated 2026-09-15, based on the **R04 ToF assembly**, not the older downloaded R03 exploded model. Original Blender files and the R04 print package are preserved. No ROS files or robot wiring are changed by this revision.

## Open the models

- [Complete assembled robot](roam_geekworm_r05.blend)
- [Electronics inspection view](roam_geekworm_r05_inspection.blend) — hood and sensor frame hidden to expose the new installation
- [Exploded robot](roam_geekworm_r05_exploded.blend)
- [Converter and printable mount close-up](geekworm_mount_detail.blend)
- [Standalone purchased-converter reference](geekworm_converter_reference.blend)

These are saved views of the same geometry. The exploded file is an identification view, not a collision-free disassembly animation. Blender scene scale remains **1 BU = 100 mm**; the STL exports are already in millimetres.

![Geekworm and new mount](geekworm_mount_detail.png)

## What changed

The old LM2596 reference and C02 carrier are replaced by a Geekworm **Pi5-5V5APD, 9–24 V, 3.81 mm 2-pin terminal-input** reference and a new C02 cradle. Two C08 edge retainers hold its supplied plastic base; no unmeasured OEM PCB mounting holes are used. Two thin F05 compliant strips protect the base rim.

The module is rotated so its USB-C output faces the side cable channel. The adapter has 4 mm integral legs above the retained S01 spacers. Its location avoids the camera support and neighbouring IMU. The deck, sensor frame, hood, other electronics mounts and eight ToF mounts are unchanged.

The electronics reference depicts the base, blue PCB, fan, green terminal, USB ports, switch, jumper and representative components. Small component shapes, traces, hole positions and connector details are illustrative—not an OEM manufacturing model. Other purchased hardware is inherited from R04 and has not been re-measured in this task.

## Print packages

- [Changed parts only](ROAM_R05_changed_parts.zip): new C02 ×1, C08 ×2 and optional F05 ×2.
- [Complete R05 set](ROAM_STL_R05_K1C.zip): **40 unique STL files / 116 pieces**, including optional parts; 37 unchanged R04 STL files are copied bit-for-bit.
- [R05 assembly and fit instructions](ASSEMBLY_R05.md)

Do not print the old R04 C02 or S05 for this installation. Keep the two existing S01 lower spacers. Do not print the purchased electronics; the separate `reference_only` envelope STL is only a packaging/checking aid.

## Dimensions and confidence

The supplied dimensioned photograph specifies a **59 × 44 mm base, 55 × 40 mm PCB, 22 mm overall height and 30 × 30 × 7 mm fan**. That drawing shows the barrel-input variant; the other supplied photo identifies the purchased green-terminal variant. Its connector projection is not dimensioned.

The provisional reference assumes a **5 mm base thickness, 9 mm terminal overhang, 4 mm base corner radius and 1.6 mm PCB thickness**. The USB-C cable guide assumes a 24 × 12 × 9 mm plug body. The conservative reference envelope is 68 × 44 × 22 mm including the estimated terminal overhang. These estimates must be checked on arrival, particularly the base lip that the retainers grip and the actual cable bend/access space. See [dimension ledger](dimensions.json). The JSON records the dimensions and drives the main mounting datums; some detailed component geometry and retainer offsets remain explicitly defined in the generator.

## Verification and limits

[Geometry checks](geometry_checks.json) record the source SHA-256, unchanged original objects, new-part intersection tests and repeated-part equivalence. The nominated checks cover the modeled new module and mount against the original assembly, plus estimated fan/connector guides. The first 12 mm above the fan inlet is clear in this geometry. This is **not** an airflow, structural, thermal, electrical or physical-fit certification.

New printable STLs are re-imported and checked for one connected solid, zero non-manifold edges, positive volume, millimetre dimensions and Z=0 placement. Manifest hashes verify all copied/exported files. Print orientations and renderings are inspected, but complete slicer toolpaths and real printed fit are not tested. Final material choice must account for the measured converter temperature inside the hood.

## Rebuild

Generator: [tools/cad/update_geekworm_r05.py](../../tools/cad/update_geekworm_r05.py). Tested with Blender 5.2.1 LTS. Run from the repository root, substituting the source paths if moved:

```sh
/Applications/Blender.app/Contents/MacOS/Blender --background --disable-autoexec \
  --python tools/cad/update_geekworm_r05.py -- \
  --source ~/cad/roam_tof.blend \
  --print-pack /Users/igorvaz/Downloads/ROAM_STL_R04_K1C \
  --out mechanical/geekworm_r05 --render
```

The original R04 source and pack are external dependencies, not fetched automatically. Use a fresh output directory if starting a different design revision; the generator replaces its own R05 output names. `dimensions.json` and `ASSEMBLY_R05.md` live alongside the delivered outputs and are retained during regeneration. The manufacturer reference is [Geekworm Pi5-5V5APD](https://wiki.geekworm.com/Pi5-5V5APD); dimensions used here are attributed to the supplied photos, not assumed from product naming.
