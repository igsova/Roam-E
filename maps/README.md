# Maps

Saved occupancy maps (`.pgm` and `.yaml`) and serialized SLAM Toolbox pose
graphs (`.posegraph` and `.data`) are stored here. Use descriptive names when
keeping more than one mapping run.

Current checkpoints:

- `roam_e_first`: initial whole-area mapping trial.
- `living_room_perimeter`: living-room perimeter checkpoint intended for
  continued apartment mapping.
- `office_01`: standalone office map.

Each name must retain all four files. The `.pgm` and `.yaml` pair is the static
occupancy map used by Nav2. The `.posegraph` and `.data` pair is the lossless
SLAM Toolbox session required to continue or refine mapping.

See the root `README.md` for the tested commands to create, save, verify, and
resume a mapping session.
