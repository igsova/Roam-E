# Maps

Saved occupancy maps (`.pgm` and `.yaml`) and serialized SLAM Toolbox pose
graphs (`.posegraph` and `.data`) are stored here. Use descriptive names when
keeping more than one mapping run.

Current checkpoints:

- `office_02`: current standalone office map and the default for AMCL
  localization. It was captured after correcting the RPLidar angle
  convention and has been validated against live scans.
- `roam_e_first`, `living_room_perimeter`, and `office_01`: historical
  development captures made before the RPLidar native clockwise/rear-zero
  angles were converted to ROS counter-clockwise/+X angles. These maps are
  retained for reference only and must not be used for localization or
  resumed for mapping.

Each name must retain all four files. The `.pgm` and `.yaml` pair is the static
occupancy map used by Nav2. The `.posegraph` and `.data` pair is the lossless
SLAM Toolbox session required to continue or refine mapping.

See the root `README.md` for the tested commands to create, save, verify, and
resume a mapping session.
