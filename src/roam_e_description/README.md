# ISOLATED R19 camera-forward candidate — 2026-10-05

+X faces the camera, +Y is physical left, +Z up. Caster is rear.
Left/right wheel labels are swapped relative to the old model; firmware is NOT changed.
The candidate IMU net native-to-base yaw is -pi/2, with Z up.
Direction checks passed; exact angles/positions and accel calibration remain provisional.
Never source this candidate together with production or launch drive/navigation with it.
Use only a separate ROS_DOMAIN_ID and description-only preview.
Camera optical preview TF remains off by default; OpenNI retains ownership for live use.
Footprint is a rotated CAD candidate, NOT a physically verified Nav2 footprint.
