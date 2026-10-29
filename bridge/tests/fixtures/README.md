Lucy's hardware configs (`config/hardware/active.yaml`) of the robot packages,
as `/config/get` serves them, to check the packaged profiles against the real
robots:

- `inmoov_urdf.yaml`: Lucy-Robotics/inmoov_urdf `dev` @ 1bf0cd3 (inmoov_i2_urdf
  `master` ships the same file)
- `so_arm101_urdf.yaml`: Lucy-Robotics/so_arm101_urdf `dev` @ 58b702a

GPL-3.0, like this repository. Refresh them when the robots' joints change.
