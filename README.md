# RGB-LiDAR-Fusion

Real-time RGB camera and Livox Mid-360 LiDAR fusion for generating colorized 3D point clouds. Supports ROS/ROS2 and provides a foundation for 3D perception, mapping, and robotics applications.

> **Status:** early development. The first target is the Livox Mid-360; the pipeline is designed so other LiDARs and cameras can be added later.

## Demo

<!-- TODO: add a GIF/video showing RGB image | raw LiDAR | colorized point cloud side by side -->
*Coming soon: RGB camera, raw LiDAR, and colorized point cloud side by side.*

## Features

- Real-time projection of LiDAR points onto the RGB image to colorize the point cloud
- ROS and ROS 2 support
- Publishes a colorized point cloud for use in mapping, perception, and visualization (e.g. RViz)

## Hardware

| Component | Tested | Notes |
|-----------|--------|-------|
| LiDAR | Livox Mid-360 | Other Livox / LiDAR models: planned |
| Camera | Intel RealSense D435i | Uses only its RGB stream; other RGB cameras with a ROS 2 driver: planned |

### Mount and extrinsics

A 3D-printable mount holds the Mid-360 with its scan axis tilted 45° up and the D435i pitched 20° down. The CAD source, STL and **design extrinsics** (LiDAR ↔ camera transform plus a ROS 2 static TF command) are in [`hardware/mount/`](hardware/mount/).

The D435i publishes its own color intrinsics. The design extrinsics are a starting point; a calibration guide for refining them on the assembled rig is coming soon.

## Installation

*Coming soon.* This section will cover dependencies, Livox SDK/driver setup, and building with `colcon` (ROS 2) / `catkin` (ROS 1).

## Usage

*Coming soon.*

## Example output

Sample colorized point clouds (`.pcd` / `.ply`) will be added under [`examples/`](examples/).

## Roadmap

- [ ] Mid-360 + RGB camera fusion node (ROS 2)
- [ ] ROS 1 support
- [ ] Calibration guide
- [ ] Sample data and demo GIF
- [ ] Support for other LiDARs and cameras

## Contributing

Issues and pull requests are welcome, especially adapters for other LiDAR and camera hardware.
