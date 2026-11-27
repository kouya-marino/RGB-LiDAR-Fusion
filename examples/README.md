# Sample data

| Sample | Recording | Size |
|---|---|---|
| [`full_floor_sample/`](full_floor_sample/) | a whole building floor, walking 159 m in 14 min | 35 MB, `.pcd` and `.ply` |
| [`partial_floor_sample/`](partial_floor_sample/) | one area, turning in place for 27 s | 4 MB, `.pcd` and `.ply` |

Both are in the map frame: gravity-levelled, **floor at z = 0**, origin on the floor below the lidar where recording started, +X the direction the rig faced.

**View them:**
- In the web UI (it reads `.pcd`): `cp -r examples/full_floor_sample output/` and open `http://localhost:8080/?map=full_floor_sample` (the same for `partial_floor_sample`). *Hide above* in Layers cuts away the ceiling.
- In CloudCompare, MeshLab or Open3D: open the `.ply` files.
- On ROS 1: see [`ros1/`](../ros1/) (it publishes `.pcd` files).

## `full_floor_sample/`

The *Full-floor* sequence of the paper: a whole building floor, recorded with the rig carried on a pole (lidar 1.55 m above the floor at the start, 1.51–1.85 m while walking) through the rooms and corridors: 14.0 min, 159 m, 7,848 scans, 92% of them colored.

| File | Contents |
|---|---|
| `colored.pcd`, `colored.ply` | 732,170 camera-colored points (`x y z rgb`) |
| `full.pcd`, `full.ply` | the full lidar map (`x y z intensity`), 404,765 points |
| `trajectory.csv` | the lidar's path in the map frame (`t,x,y,z`, 10 Hz) |
| `info.json` | recording details and the recording stats (scan rate, colored scans, callback times) |

The saved map had 2,279,491 colored and 5,916,695 lidar points at 3 cm. Both were cleaned in a point cloud editor (points seen through windows and stray points deleted; no point moved or recolored: 95.7% and 98.6% kept), then **thinned for the repo** to one point per 5 cm (colored) and 10 cm (full) voxel. The README's full-floor images and animation are made from these files with [`tools/`](../tools/).

## `partial_floor_sample/`

A map recorded with the reference rig (Livox Mid-360S + RealSense D435i on the [mount](../hardware/mount/)), on a pole with the lidar 85.2 cm above the floor, turning it in place for 27 s.

| File | Contents |
|---|---|
| `colored.pcd`, `colored.ply` | 55,069 camera-colored points (`x y z rgb`), 3 cm voxels, as saved |
| `full.pcd`, `full.ply` | the full lidar map (`x y z intensity`), **thinned to 5 cm** for the repo (saved maps keep 3 cm: 220,751 points here) |
| `info.json` | recording details |

Because the camera looks 20° down from 85 cm, the colored points are mostly the floor and the lower parts of walls and furniture; the lidar covers the whole room.
