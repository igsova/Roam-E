# Maps

`office_02` is a 2D occupancy map of an office, built with
`slam_toolbox` from the RPLIDAR A1 and the EKF odometry. It is the default map
for AMCL localization and Nav2.

| File | Contents |
|---|---|
| `office_02.pgm`, `office_02.yaml` | Occupancy grid (5 cm cells) loaded by Nav2's map server |
| `office_02.posegraph`, `office_02.data` | Serialized `slam_toolbox` session, used to continue or refine the map |

Keep all four files together when you save a new map under another name. How to
create, save and resume a map is in [`docs/usage.md`](../docs/usage.md).
