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
| LiDAR | Livox Mid-360S | Plain Mid-360 uses the driver's older `MID360` config format (untested here); other LiDARs: planned |
| Camera | Intel RealSense D435i | Uses only its RGB stream; other RGB cameras with a ROS 2 driver: planned |

### Mount and extrinsics

A 3D-printable mount holds the Mid-360 with its scan axis tilted 45° up and the D435i pitched 20° down. The CAD source, STL and **design extrinsics** (LiDAR ↔ camera transform plus a ROS 2 static TF command) are in [`hardware/mount/`](hardware/mount/).

The D435i publishes its own color intrinsics. The design extrinsics are a starting point; a calibration guide for refining them on the assembled rig is coming soon.

## Installation

Everything runs in a ROS 2 Humble Docker container. The host needs only Docker, plus the RealSense udev rules if you use a RealSense camera.

### 1. Build the image

```bash
docker/build.sh
```

This builds `rgb-lidar-fusion:humble` with ROS 2 Humble desktop, the RealSense ROS 2 driver, [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) v1.3.1, [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) 1.2.6, PCL and cv_bridge. The container user gets your host UID/GID, so files it writes into the repo stay yours.

### 2. Network for the LiDAR

The Mid-360S talks UDP over Ethernet. Give the host's Ethernet port a static address on the lidar's subnet, then put both addresses in [`config/livox/MID360s_config.json`](config/livox/MID360s_config.json):

| | Setting | Default in this repo |
|---|---|---|
| Host | `host_net_info[0].host_ip` | `192.168.1.200` |
| LiDAR | `lidar_configs[0].ip` | `192.168.1.179` |

The lidar's factory IP is `192.168.1.1XX`, where `XX` is the last two digits of its serial number. Check with `ping`. Don't give the Ethernet connection a default gateway.

### 3. Start the container

```bash
docker/run.sh            # shell inside the container; run it again for a second shell
```

The repo is mounted at `/ws/src/rgb_lidar_fusion`. The container uses host networking (the lidar uses UDP broadcast discovery) and gets `/dev` access for USB cameras.

### 4. Check the sensors

Inside the container:

```bash
# LiDAR -> /livox/lidar (PointCloud2, ~20k points @ 10 Hz) and /livox/imu (200 Hz)
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
  -p output_data_type:=0 -p frame_id:=livox_frame \
  -p user_config_path:=/ws/src/rgb_lidar_fusion/config/livox/MID360s_config.json

# Camera -> /camera/camera/color/image_raw + camera_info
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x30
```

If the lidar driver logs `Init lds lidar success!` but no `/livox/lidar` topic appears, check the config format. A Mid-360S that gets the plain `MID360` config is discovered but never configured.

## Usage

*Coming soon.*

## Example output

Sample colorized point clouds (`.pcd` / `.ply`) will be added under [`examples/`](examples/).

## Roadmap

- [x] Docker dev environment (ROS 2 Humble, Livox + RealSense drivers)
- [ ] Mid-360 + RGB camera fusion node (ROS 2)
- [ ] ROS 1 support
- [ ] Calibration guide
- [ ] Sample data and demo GIF
- [ ] Support for other LiDARs and cameras

## Contributing

Issues and pull requests are welcome, especially adapters for other LiDAR and camera hardware.
