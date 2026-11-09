# Mid-360 + D435i mount

A 3D-printable mount for the Livox Mid-360 and an Intel RealSense D435i. The lidar's scan axis is tilted 45° up from horizontal, and the camera is pitched 20° down. It attaches to a wall with M5 screws or to a pole with hose clamps.

| File | What it is |
|------|------------|
| [`mid360_45_d435i_20down_mount_strong.py`](mid360_45_d435i_20down_mount_strong.py) | Parametric CAD source. Edit the parameters at the top and re-run. |
| [`mid360_45_d435i_20down_mount_strong.stl`](mid360_45_d435i_20down_mount_strong.stl) | Printable mesh: watertight, 75 × 100 × 120 mm, 193.5 cm³ |
| [`extrinsics.txt`](extrinsics.txt) | Script output: the design extrinsics computed from the CAD geometry |

Regenerate from this folder:

```bash
pip install manifold3d trimesh numpy scipy
python3 mid360_45_d435i_20down_mount_strong.py > extrinsics.txt
```

## Design extrinsics

Translations are in metres and quaternions are `(x, y, z, w)`.

### Frames

| Frame | Axes |
|-------|------|
| `livox_frame` (lidar) | X downhill along the plate, Y left, Z along the scan axis |
| `camera_link` (RealSense) | X forward, Y left, Z up; origin at the left IR imager (= depth frame) |
| `camera_*_optical_frame` | Z forward, X right, Y down |

### Camera in lidar frame (`livox_frame` → `camera_link`)

```
t = [0.0842, 0.0175, -0.0669]
q = [0, -0.2164, 0, 0.9763]      # pitch -25°
```

### Lidar in camera frame (`camera_link` → `livox_frame`)

```
t = [-0.0480, -0.0175, 0.0963]
q = [0, 0.2164, 0, 0.9763]
```

### Optical frame in lidar frame (`T_lidar_optical`)

```
[ 0.0000  0.4226  0.9063  0.0842]
[-1.0000  0.0000  0.0000  0.0175]
[ 0.0000 -0.9063  0.4226 -0.0669]
[ 0       0       0       1     ]
```

### Lidar in optical frame (`T_optical_lidar`)

```
[ 0.0000 -1.0000  0.0000  0.0175]
[ 0.4226  0.0000 -0.9063 -0.0963]
[ 0.9063  0.0000  0.4226 -0.0480]
[ 0       0       0       1     ]
```

> **Color vs. depth:** these optical-frame matrices use the origin of `camera_link`, which sits at the **depth / left IR** imager. On the D435i the color sensor is about 15 mm to the left (+Y in `camera_link`), so for projection into the **RGB** image, the X translation of `T_optical_lidar` becomes about `0.0325` instead of `0.0175`. Don't hard-code either matrix. Publish only the static transform below and look up `livox_frame` → `camera_color_optical_frame` through TF. The `realsense2_camera` driver publishes the factory-calibrated `camera_link` → `camera_color_optical_frame` transform.

## ROS 2 static transform

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0842 --y 0.0175 --z -0.0669 \
  --qx 0 --qy -0.2164 --qz 0 --qw 0.9763 \
  --frame-id livox_frame --child-frame-id camera_link
```

## Caveats

- These are **design** values from CAD, not a calibration. Print tolerances and how each sensor seats on the mount will shift them by a few mm and a fraction of a degree. Once everything is printed and mounted, refine them with a target-based or targetless LiDAR–camera calibration.
- The script assumes the Mid-360 point-cloud origin is **47 mm above the sensor's base** (`LIDAR_ORIGIN_H`). Check this against the Mid-360 manual drawing; any error goes straight into the Z/X translation.
- The M3 slots on the lidar plate fit square hole patterns of 24–42 mm per side. Check them against your unit before printing.
