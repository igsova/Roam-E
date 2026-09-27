# Preserved external source dependencies

Copied into the project on 2026-09-27; original files were
left untouched. These are **historical input sources**, not the current R19
assembly and not instructions to print obsolete parts.

| File | Purpose | Original source |
|---|---|---|
| `Roam_E_ROS.FCStd` | Original FreeCAD source for URDF mesh export | `/Users/igorvaz/Downloads/Roam_E_ROS.FCStd` |
| `roam_tof_R04.blend` | R04 assembled ToF robot, input to R05 generator | `~/cad/roam_tof.blend` |
| `roam_tof_mux_donor.blend` | Multiplexer collection donor for R06, **not** a current whole robot | Same outputs directory, `roam_tof_mux.blend` |
| `ROAM_STL_R04_K1C.tar.gz` | R04 print-pack source required to reproduce R05 unchanged parts | `/Users/igorvaz/Downloads/ROAM_STL_R04_K1C` |

SHA-256 at preservation:

```text
0c9d5416f4ed51ea6f1006936b50153212b99a24a41df40aaf7d04f3399921fe  Roam_E_ROS.FCStd
564c5565ad6a87afcc39314f9426d4dfe8b179134e6626c98eee9580108024f9  roam_tof_R04.blend
80ff95cb84b2947e31e19642ad8c02c46c9e3e079db68126e83283f628cc70f9  roam_tof_mux_donor.blend
6b5f653d98a3dadb1868fd4cd7a602f2ab146cc81a46c31373c31adae6862403  ROAM_STL_R04_K1C.tar.gz
```

The two Blender hashes match the original R05/R06 geometry provenance.
The complete extracted print pack was compared with its source when it was
copied. FreeCAD source is retained independently of the later Blender branch.

For R05 reconstruction, extract the archive into a **new scratch directory**
and pass that `ROAM_STL_R04_K1C` directory as `--print-pack`, and the preserved
R04 Blender file as `--source`. Use a new `--out` directory to avoid replacing
existing reviewed outputs. The R06 importer now defaults to the repo-local
donor; `--donor /path/to/file.blend` remains available.

The filenames above deliberately are not download links: these binary sources
are not in the Git checkout. Binary sources/assemblies are large and remain in the local workspace;
they have not been committed. Choose a reviewed
Git LFS or artifact-release strategy before publishing the full mechanical
history. Do not discard masters or inputs just to reduce a commit size.
