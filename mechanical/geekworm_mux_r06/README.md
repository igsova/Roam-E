# ROAM R06 — combined Geekworm and multiplexer revision

Latest integrated mechanical model: **Geekworm R05 retained, TCA9548A imported, unused MCU removed**. The Arduino Nano remains. Previous model files are preserved, not overwritten.

## Open

- [Complete robot](roam_geekworm_mux_r06.blend)
- [Electronics inspection view](roam_geekworm_mux_r06_inspection.blend) — easiest view to check the removal and the two new modules together
- [Exploded robot](roam_geekworm_mux_r06_exploded.blend)
- [Isolated multiplexer and mount](tca9548a_mount_reference.blend)

![Electronics installation](electronics_installation.png)

## Print

- [Complete R06 STL pack](ROAM_STL_R06_Geekworm_MUX_K1C.zip): 41 unique files, 116 pieces including optional parts.
- [Changes from Geekworm R05 only](ROAM_R06_changed_parts.zip): revised P04, C09 carrier, P20 clips and F06 pads.
- [Assembly, hardware and fit guide](ASSEMBLY_R06.md)
- [Geometry checks and source provenance](geometry_checks.json)

Old MCU C04 and S06 are absent. Common S01 spacer quantity is reduced to 12. Multiplexer donor C08/F05 become **C09/F06** so they do not overwrite the Geekworm C08/F05. The new P04 closes the unused MCU holes and adds two mounting holes for the multiplexer. All other geometry is preserved from the latest R05 robot.

The importer takes only collection `11 Multiplexer assembly` from `roam_tof_mux.blend`; it does not copy its older buck, deck or whole robot. Imported mux geometry remains unchanged apart from part names/metadata. The PCB outline is 21.4 × 31.5 mm; header/pad/PCB-thickness assumptions still require an actual fit check.

## Rebuild

[Generator](../../tools/cad/merge_mux_r06.py), tested with Blender 5.2.1 LTS:

```sh
/Applications/Blender.app/Contents/MacOS/Blender --background --disable-autoexec \
  --python tools/cad/merge_mux_r06.py -- --render
```

Run from the repository root. Sources are the repository's Geekworm R05 assembly/print pack and the external donor at `~/cad/roam_tof_mux.blend`. Both source hashes are recorded and checked unchanged. This script regenerates only its R06 output names. No git commit, ROS modifications, electrical configuration or robot motion is performed.
