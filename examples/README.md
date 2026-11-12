# Sample data

| Sample | Recording | Size |
|---|---|---|
| [`room_sample/`](room_sample/) | a whole office floor, walking 152 m in 13 min | 18 MB, `.ply` only |
| [`office_sample/`](office_sample/) | one area, turning in place for 27 s | 4 MB, `.pcd` and `.ply` |

Both are in the map frame: gravity-levelled, **floor at z = 0**, origin on the floor below the lidar where recording started, +X the direction the rig faced.

**View them:**
- In the web UI (it reads `.pcd`, so `office_sample`): `cp -r examples/office_sample output/` and open `http://localhost:8080/?map=office_sample`. *Hide above* in Layers cuts away the ceiling.
- In CloudCompare, MeshLab or Open3D: open the `.ply` files.
- On ROS 1: see [`ros1/`](../ros1/) (it publishes `.pcd` files, so use `office_sample` or your own saved maps).

## `room_sample/`

A whole office floor, recorded with the rig held up (lidar 1.73 m above the floor) while walking through the rooms and corridors: 12.8 min, 152 m, 3,933 scans.

| File | Contents |
|---|---|
| `colored.ply` | 724,679 camera-colored points (`x y z rgb`), **thinned to 5 cm** for the repo. Cleaned of stray points in a point cloud editor after saving. |
| `full.ply` | the full lidar map (`x y z intensity`) as recorded, **thinned to 10 cm** (the saved map has 4.9 M points at 3 cm) |
| `info.json` | recording details |

## `office_sample/`

A map recorded with the reference rig (Livox Mid-360S + RealSense D435i on the [mount](../hardware/mount/)), on a pole with the lidar 85.2 cm above the floor, turning it in place for 27 s in an office.

| File | Contents |
|---|---|
| `colored.pcd`, `colored.ply` | 55,069 camera-colored points (`x y z rgb`), 3 cm voxels, as saved |
| `full.pcd`, `full.ply` | the full lidar map (`x y z intensity`), **thinned to 5 cm** for the repo (saved maps keep 3 cm: 220,751 points here) |
| `info.json` | recording details |

Because the camera looks 20° down from 85 cm, the colored points are mostly the floor and the lower parts of walls and furniture; the lidar covers the whole room.
