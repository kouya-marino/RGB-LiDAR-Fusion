# Sample data

## `office_sample/`

A map recorded with the reference rig (Livox Mid-360S + RealSense D435i on the [mount](../hardware/mount/)), on a pole with the lidar 85.2 cm above the floor, turning it in place for 27 s in an office.

| File | Contents |
|---|---|
| `colored.pcd`, `colored.ply` | 55,069 camera-colored points (`x y z rgb`), 3 cm voxels, as saved |
| `full.pcd`, `full.ply` | the full lidar map (`x y z intensity`), **thinned to 5 cm** for the repo (saved maps keep 3 cm: 220,751 points here) |
| `info.json` | recording details |

The frame is the map frame: gravity-levelled, **floor at z = 0**, origin on the floor below the lidar where recording started, +X the direction the rig faced. Because the camera looks 20° down from 85 cm, the colored points are mostly the floor and the lower parts of walls and furniture; the lidar covers the whole room.

**View it:**
- In the web UI: `cp -r examples/office_sample output/` and open `http://localhost:8080/?map=office_sample`. *Hide above* in Layers cuts away the ceiling.
- In CloudCompare, MeshLab or Open3D: open the `.ply` files.
- On ROS 1: see [`ros1/`](../ros1/).
