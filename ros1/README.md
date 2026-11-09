# Using the maps on ROS 1 (Noetic)

The mapping pipeline runs on ROS 2 Humble (in Docker). Its output is plain PCD and PLY files, so a robot running **ROS 1 Noetic** can use the maps directly. No ROS 2 is needed on the robot.

## Which file

Each saved map folder (`output/<name>/`) has the map itself plus one folder per robot export (see the main README's *Web UI* section):

| File | Contents | Use it for |
|---|---|---|
| `<robot>/full.pcd`, `<robot>/full.ply` | every lidar point, `x y z intensity`, 3 cm voxels | **localization prior map** for that robot |
| `<robot>/colored.pcd`, `<robot>/colored.ply` | camera-colored points, `x y z rgb` | visualization |
| `full.*`, `colored.*` (top level) | the same in the levelled map frame, floor at z = 0 | anything that wants a floor-based frame |

## Frames

A robot export is expressed **in that robot's lidar frame at the moment mapping started**:

- **Origin:** where the rig's lidar was when you pressed Start, at the robot's lidar height above the floor (for example 125 cm for a G1).
- **+X:** the direction the rig faced at Start, flattened onto the floor.
- **+Z:** up. Roll and pitch are applied if you set them for that robot. Use them only if the robot's driver reports its lidar tilted or upside down; a Livox driver configured with roll 180 already reports an upside-down lidar upright.

So a localization stack that starts at the map origin (initial pose 0, 0, 0) is right when the robot **stands where mapping started, facing the same way**. Elsewhere, give it the initial pose, for example with RViz's *2D Pose Estimate*.

## Publish a map as a topic

[`launch/saved_map.launch`](launch/saved_map.launch) publishes a PCD with `pcl_ros` (`sudo apt install ros-noetic-pcl-ros`). No build is needed; launch the file by its path:

```bash
roslaunch /path/to/RGB-LiDAR-Fusion/ros1/launch/saved_map.launch \
    map:=/path/to/output/<name>/g1/full.pcd frame_id:=map topic:=/saved_map
```

In RViz, set the fixed frame to `map` and add a *PointCloud2* display on `/saved_map`. For a colored cloud, use `colored.pcd` and the *RGB8* color transformer.

## Use it as a localization map

Point the localization stack's map path at the robot's export: `full.pcd` for PCL-based nodes, `full.ply` for Open3D-based registration (both hold the same points). For example, a FAST-LIO-based localization package with an Open3D global registration node takes the `.ply` as its prior map.

Check the result once: drive the robot to the mapping start point, start localization at (0, 0, 0), and the live scan should overlay the map in RViz.
