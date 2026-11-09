# RGB-LiDAR-Fusion

Real-time RGB camera and Livox Mid-360 LiDAR fusion for generating colorized 3D point clouds. Supports ROS/ROS2 and provides a foundation for 3D perception, mapping, and robotics applications.

> **Status:** early development. The first target is the Livox Mid-360; the pipeline is designed so other LiDARs and cameras can be added later.

## Demo

<!-- TODO: add a GIF/video showing RGB image | raw LiDAR | colorized point cloud side by side -->
*Coming soon: RGB camera, raw LiDAR, and colorized point cloud side by side.*

## Features

- Real-time colorization of every LiDAR scan (10 Hz) from the nearest-in-time RGB frame
- Occlusion rejection, so foreground objects don't paint their color onto surfaces behind them
- Best-view voxel map: keeps the sharpest observation of each voxel instead of averaging into mud
- LiDAR ↔ camera extrinsic read from TF; camera intrinsics from the camera driver
- Saves colored clouds as binary PCD (PCL `x y z rgb`) and PLY
- ROS 2 Humble, fully containerized (ROS 1 planned)

## Hardware

| Component | Tested | Notes |
|-----------|--------|-------|
| LiDAR | Livox Mid-360S | Plain Mid-360 uses the driver's older `MID360` config format (untested here); other LiDARs: planned |
| Camera | Intel RealSense D435i | Uses only its RGB stream; other RGB cameras with a ROS 2 driver: planned |

### Mount and extrinsics

A 3D-printable mount holds the Mid-360 with its scan axis tilted 45° up and the D435i pitched 20° down. The CAD source, STL and **design extrinsics** (LiDAR ↔ camera transform plus a ROS 2 static TF command) are in [`hardware/mount/`](hardware/mount/).

<table>
  <tr>
    <td align="center"><img src="hardware/mount/images/mount_cad_views.png" alt="CAD views of the mount: front-left, side, front and back" width="420"></td>
    <td align="center"><img src="hardware/mount/images/mount_fov.png" alt="Side view of the mount with the lidar and camera fields of view" width="380"></td>
  </tr>
  <tr>
    <td align="center"><sub>Mount, 100 × 75 × 120 mm</sub></td>
    <td align="center"><sub>LiDAR (45°) and camera (20° down) fields of view</sub></td>
  </tr>
</table>

The D435i publishes its own color intrinsics. The design extrinsics are a starting point; a calibration guide for refining them on the assembled rig is coming soon.

## Installation

Everything runs in a ROS 2 Humble Docker container. The host needs only Docker, plus the RealSense udev rules if you use a RealSense camera.

### 1. Build the image

```bash
docker/build.sh
```

This builds `rgb-lidar-fusion:humble` with ROS 2 Humble desktop, the RealSense ROS 2 driver, [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) v1.3.1, [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) 1.2.6, PCL and cv_bridge. The container user gets your host UID/GID, so files it writes into the repo stay yours.

### 2. Network for the LiDAR

The Mid-360S talks UDP over Ethernet. Give the host's Ethernet port a static address on the lidar's subnet, then put both addresses in [`rgb_lidar_fusion/config/livox/MID360s_config.json`](rgb_lidar_fusion/config/livox/MID360s_config.json):

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

### 4. Build the package

Inside the container:

```bash
cd /ws && colcon build && source install/setup.bash
```

### 5. Check the sensors

```bash
ros2 launch rgb_lidar_fusion sensors.launch.py
```

This starts the Livox driver (`/livox/lidar` at 10 Hz with ~20k points, `/livox/imu` at 200 Hz), the RealSense color stream (`/camera/camera/color/image_raw`, 1920×1080 at 15 fps; `color_profile:=1280x720x15` for lighter recordings), and the static `livox_frame → camera_link` transform from [`config/mount_extrinsics.yaml`](rgb_lidar_fusion/config/mount_extrinsics.yaml).

If the lidar driver logs `Init lds lidar success!` but no `/livox/lidar` topic appears, check the config format. A Mid-360S that gets the plain `MID360` config is discovered but never configured.

## Usage

```bash
ros2 launch rgb_lidar_fusion colorize.launch.py
```

This starts the sensors, the colorizer and RViz. The colorizer publishes:

| Topic | Contents |
|---|---|
| `/colored_scan` | Each colored scan, XYZRGB in `livox_frame`, 10 Hz. Only points the camera can see. |
| `/colored_map` | With `accumulate:=true`: a voxel map keeping the best view of each voxel. Without odometry this assumes the rig is **stationary**. |

Build a map while the rig stays still, then save it:

```bash
ros2 launch rgb_lidar_fusion colorize.launch.py accumulate:=true \
    output_dir:=/ws/src/rgb_lidar_fusion/output
# saves output/colored_cloud.{pcd,ply} on Ctrl-C, or on demand:
ros2 service call /colorizer/save_cloud std_srvs/srv/Trigger
```

Launch arguments: `sensors:=false` (sensors already running, or playing a bag), `rviz:=false`, `color_profile:=WxHxFPS`. Colorizer tuning is in [`config/colorizer.yaml`](rgb_lidar_fusion/config/colorizer.yaml).

**How it works.** The driver stamps each scan at the start of its 100 ms sweep. The colorizer waits until an image from after the sweep's midpoint has arrived, picks the nearest one, and drops the scan if the gap exceeds `max_dt` (50 ms). It then projects the points through the camera model, rejects points hidden behind nearer ones, and samples their colors. Images are subscribed **reliable**: a full-resolution frame (6.2 MB at 1080p) is larger than DDS shared memory, and best-effort delivery lost about half the frames in testing. With reliable delivery, 1080p at 15 fps arrives without gaps and 98% of scans are colored.

## Example output

Sample colorized point clouds (`.pcd` / `.ply`) will be added under [`examples/`](examples/).

## Roadmap

- [x] Docker dev environment (ROS 2 Humble, Livox + RealSense drivers)
- [x] Live per-scan colorization node (ROS 2)
- [x] Best-view voxel map for a stationary rig, PCD/PLY export
- [ ] Mapping while moving (LiDAR odometry, e.g. FAST-LIO)
- [ ] LiDAR–camera extrinsic calibration tool
- [ ] ROS 1 support
- [ ] Calibration guide
- [ ] Sample data and demo GIF
- [ ] Support for other LiDARs and cameras

## Contributing

Issues and pull requests are welcome, especially adapters for other LiDAR and camera hardware.
