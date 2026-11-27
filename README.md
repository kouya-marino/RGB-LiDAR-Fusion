# RGB-LiDAR-Fusion

[![Tests](https://github.com/kouya-marino/RGB-LiDAR-Fusion/actions/workflows/tests.yml/badge.svg)](https://github.com/kouya-marino/RGB-LiDAR-Fusion/actions/workflows/tests.yml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-ready-2496ED?logo=docker&logoColor=white)](docker/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Livox Mid-360](https://img.shields.io/badge/LiDAR-Livox%20Mid--360-orange)](hardware/)
[![RealSense D435i](https://img.shields.io/badge/Camera-RealSense%20D435i-0071C5?logo=intel)](hardware/)
[![GitHub stars](https://img.shields.io/github/stars/kouya-marino/RGB-LiDAR-Fusion?style=social)](https://github.com/kouya-marino/RGB-LiDAR-Fusion/stargazers)

Real-time RGB camera and Livox Mid-360 LiDAR fusion for generating colorized 3D point clouds. Supports ROS/ROS2 and provides a foundation for 3D perception, mapping, and robotics applications.

**Walk around with a lidar and a camera on a stick, and watch a colored 3D map of the place build up in your browser.** Save it as PCD/PLY, ready for your robot.

### Highlights

- 🎨 **Live colorized mapping**: every 10 Hz lidar scan is colored from the camera, with occlusion checks, and FAST-LIO2 tracks the rig as you walk
- 🌐 **Web UI**: watch the map grow in 3D, record, save, browse and download maps from any device on the network
- 📐 **Ready-to-use maps**: gravity-levelled with the floor at z = 0, plus exports for robots (Unitree Go2 / G1 heights built in, your own in the UI)
- 🎯 **Targetless calibration**: one command aligns the lidar to the camera from an ordinary scene
- 🖨️ **Printable mount** for the Mid-360 + D435i that fits a camera stick, a pole or a wall
- 🐳 **One container**: ROS 2 Humble with the RealSense driver from apt; the Livox SDK and driver, FAST-LIO2 and Three.js are vendored in the repo, so nothing is fetched from GitHub or a CDN

### Quick start

```bash
docker/build.sh && docker/run.sh                       # build the image, open a shell in the container
cd /ws && colcon build && source install/setup.bash    # inside the container
ros2 launch rgb_lidar_fusion colorize.launch.py        # sensors + mapping + web UI
```

Then open **http://localhost:8080**. First time? Set up the lidar's network as in [Installation](#installation).

> **Status:** working end to end on the reference rig (Livox Mid-360S + RealSense D435i). The pipeline is designed so other LiDARs and cameras can be added later.

## Demo

**A whole floor**, mapped by walking 152 m in 13 min with the rig held up (lidar at 1.73 m):

<p align="center">
  <img src="docs/images/demo_full_room.gif" alt="A whole office floor in three synchronized views: lidar and camera combined, camera-colored points only, full lidar colored by height" width="900">
</p>

<p align="center">
  <img src="docs/images/full_room_topdown.jpg" alt="Top-down view of the whole floor: lidar and camera combined, camera colors only, and the full lidar map with the walked path in red" width="900">
</p>
<p align="center">
  <img src="docs/images/full_room_oblique.jpg" alt="Oblique view of the whole floor with camera colors inside the lidar map, ceiling hidden" width="640">
</p>

The views, left to right: camera colors inside the full lidar map, only the camera-colored points, and the full lidar map colored by height with the walked path (red). The colored cloud was cleaned of stray points in a point cloud editor after saving; the full lidar map is as recorded. The data is in [`examples/room_sample/`](examples/room_sample/).

**One area**, turning the rig in place:

<p align="center">
  <img src="docs/images/demo_colored_map.gif" alt="An office map in three synchronized views: lidar and camera combined, camera-colored points only, full lidar colored by height" width="900">
</p>

An office mapped in 27 s with the rig on a pole, ceiling hidden with the viewer's height clip. **Left:** the result, camera colors inside the full lidar map. **Middle:** only the camera-colored points (the camera looks 20° down, so mostly floor, furniture and lower walls). **Right:** the full lidar map, colored by height. The data is in [`examples/office_sample/`](examples/office_sample/).

## Features

- Real-time colorization of every LiDAR scan (10 Hz) from the nearest-in-time RGB frame
- Occlusion rejection, so foreground objects don't paint their color onto surfaces behind them
- **Map while you walk**: FAST-LIO2 lidar-inertial odometry tracks the rig, so it can be carried on a pole
- Maps are **gravity-levelled** with the floor at z = 0, from the lidar height you enter (the mount tilts the lidar 45°)
- Two clouds per map: the **colored** points the camera sees, and the **full** lidar scan (the camera covers only part of the lidar's field of view)
- **Robot exports** on save: the same map re-expressed for a Unitree Go2 (lidar at 45 cm) and G1 (lidar at 125 cm), ready for their localization
- Best-view voxel map: keeps the sharpest observation of each voxel instead of averaging into mud
- **Web UI**: watch the map grow in 3D, start / stop / save recordings, browse and download saved maps
- LiDAR ↔ camera extrinsic read from TF; camera intrinsics from the camera driver
- Saves colored clouds as binary PCD (PCL `x y z rgb`) and PLY
- ROS 2 Humble, fully containerized; the saved maps are plain PCD/PLY and work on ROS 1 Noetic robots too ([`ros1/`](ros1/))

## Hardware

| Component | Tested | Notes |
|-----------|--------|-------|
| LiDAR | Livox Mid-360S | Plain Mid-360 uses the driver's older `MID360` config format (untested here); other LiDARs: planned |
| Camera | Intel RealSense D435i | Uses only its RGB stream; other RGB cameras with a ROS 2 driver: planned |

### Mount and extrinsics

A 3D-printable mount holds the Mid-360 with its scan axis tilted 45° up and the D435i pitched 20° down. It fits a camera stick or tripod (1/4"-20), a pole (hose clamps) or a wall, and all four lidar screws can be reached (the uphill pair through a window in the back). The CAD source, STL and **design extrinsics** (LiDAR ↔ camera transform plus a ROS 2 static TF command) are in [`hardware/mount/`](hardware/mount/).

<table>
  <tr>
    <td align="center"><img src="hardware/mount/images/mount_cad_views.png" alt="CAD views of the mount, including the back window, the camera-stick socket, and the mount with the Mid-360S and D435i fitted" width="600"></td>
    <td align="center"><img src="hardware/mount/images/mount_fov.png" alt="Side view of the mount with the lidar and camera fields of view" width="260"></td>
  </tr>
  <tr>
    <td align="center"><sub>Mount, 100 × 75 × 120 mm</sub></td>
    <td align="center"><sub>LiDAR (45°) and camera (20° down) fields of view</sub></td>
  </tr>
</table>

**Coverage:** the lidar covers **100% of the camera's field of view** (every direction in the camera image is inside the lidar's band), and the camera covers about **14% of the lidar's** field of view. So any surface in the image can be colored once the lidar has hit it; the rest of the lidar map stays uncolored.

The D435i publishes its own color intrinsics. The design extrinsics are a starting point; refine them on the assembled rig with the [calibration tool](#calibration).

## Installation

Everything runs in a ROS 2 Humble Docker container. The host needs only Docker, plus the RealSense udev rules if you use a RealSense camera.

### 1. Build the image

```bash
docker/build.sh
```

This builds `rgb-lidar-fusion:humble` with ROS 2 Humble desktop, the RealSense ROS 2 driver, PCL, cv_bridge, Livox-SDK2 v1.3.1 + livox_ros_driver2 1.2.6, and FAST-LIO2 (ROS 2). Livox and FAST-LIO are built from the copies in [`third_party/`](third_party/), so nothing is fetched from GitHub at build time. The container user gets your host UID/GID, so files it writes into the repo stay yours.

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

This starts the Livox driver (`/livox/lidar` at 10 Hz with ~20k points, `/livox/imu` at 200 Hz), FAST-LIO2 (`/cloud_registered`, `/Odometry`), the RealSense color stream (`/camera/camera/color/image_raw`, 1920×1080 at 15 fps; `color_profile:=1280x720x15` for lighter recordings), and the static transforms of the rig: `body` (IMU) → `livox_frame` from the Mid-360S manual, and `livox_frame → camera_link` from [`config/mount_extrinsics.yaml`](rgb_lidar_fusion/config/mount_extrinsics.yaml). Keep the rig still for a few seconds after launch while FAST-LIO initializes its IMU. For a rig that never moves, `odometry:=false` skips FAST-LIO.

If the lidar driver logs `Init lds lidar success!` but no `/livox/lidar` topic appears, check the config format. A Mid-360S that gets the plain `MID360` config is discovered but never configured.

## Usage

```bash
ros2 launch rgb_lidar_fusion colorize.launch.py output_dir:=/ws/src/rgb_lidar_fusion/output
```

This starts the sensors, the colorizer and the web UI. Open **http://localhost:8080** (or `http://<this-pc>:8080` from another machine on the network).

To stop, press **Ctrl+C** in the launch terminal; a map that is still recording is saved first.

### Web UI

| | |
|---|---|
| **Live preview** | Before recording: the latest colored scan over the latest full scan, levelled, with the rig's position and heading. |
| **Record a map** | Name it (required; a name that already exists asks you to type **OVERWRITE**, and the old map is only replaced once the new one is saved), enter the **lidar height above the floor** (cm, to the centre of the dome, about 13 cm above the mount's bottom edge; default 85.2, set in `colorizer.yaml`; a different value is highlighted and confirmed in the Start dialog), optionally change the **max color distance** (default 10 m: farther points are not colored, since color accuracy drops with distance; they still go into the full lidar map) and tick **Auto-stop** to stop and save after 10–3600 s (it runs in the colorizer, so it works with the browser closed). Wait for *Odometry · Camera · Held still* to turn green, press *Start recording* and confirm: a **15 s countdown** gives you time to get into position (*Start now* skips it, *Cancel* or Esc aborts; on a phone keep the screen on, since a locked screen pauses the page), and its last 3 s ask you to hold the rig still, because that moment levels the map. Then walk slowly. *Stop & save* (or *Cancel* to discard). The maps and the path walked grow live in 3D; the header shows the lidar's live height above the floor (and its lowest–highest range), the distance walked and the time, and a warning appears while you turn faster than 30°/s (those scans are not colored). The height may change freely while you walk: it is only fixed at Start. |
| **Layers** | Colored points, full lidar and path on/off; color the full lidar by height, intensity or gray; **hide above / below** a height (cut away the ceiling to see inside a room; view only); **crop X/Y** to a box (*Fit to map* sets it to the 1st–99th percentile of the map plus 0.5 m, which hides stray points seen through windows; view only); point size; fit view. |
| **Saved maps** | Collapsed list with a search box; click a map for its details, *View in 3D*, downloads (the map itself and each robot export) and *Delete* (type the map's name to confirm; not possible while that map is being recorded). `?map=<name>` links straight to one; add `&crop=x0,x1,y0,y1` (m) to share a cropped view. |

<p align="center">
  <img src="docs/images/webui_recording.png" alt="Web UI while recording: live colored map, lidar height 155 cm, auto-stop countdown, robot exports Unitree-Go2 and Unitree-G1" width="860">
</p>

<table>
  <tr>
    <td width="68%"><img src="docs/images/webui_start_dialog.png" alt="Start recording dialog: map name, lidar height 155 cm flagged as not the default, color distance 10 m, auto-stop after 900 s, robot exports Unitree-Go2 and Unitree-G1"></td>
    <td width="32%"><img src="docs/images/webui_saved_maps_list.png" alt="Saved maps list: details, Re-export, Delete, downloads for the map, Unitree-Go2 and Unitree-G1"></td>
  </tr>
  <tr>
    <td align="center"><sub>Start confirms the name, lidar height, color distance, auto-stop and robot exports, then counts down 15 s</sub></td>
    <td align="center"><sub>Saved map: downloads for the map and each robot</sub></td>
  </tr>
</table>

Each map is saved in its own folder in the repo's `output/` (on the host, so maps survive the container being stopped; `output/` is git-ignored):

```
output/<name>/
  colored.pcd  colored.ply   # x y z rgb       - points the camera saw, best view per 3 cm voxel
  full.pcd     full.ply      # x y z intensity - every lidar point, one per 3 cm voxel
  go2/  colored.* full.*     # the same, with the origin at a Go2 lidar (45 cm above the floor)
  g1/   colored.* full.*     # the same, with the origin at a G1 lidar (125 cm above the floor)
  trajectory.csv             # t, x, y, z of the lidar along the walk
  info.json                  # start time, duration, distance walked, lidar height, color distance,
                             # auto-stop, point counts, exports
```

**Frames.** The map frame is gravity-levelled: **floor at z = 0**, origin on the floor directly below the lidar where recording started, **+X** = the direction the rig faced at the start (its lidar +X, flattened). A robot export puts the origin at that robot's lidar instead (`z -= height`), as if the robot's lidar had started where the rig's did, so the robot can start localization at the map origin. Edit the robots in the web UI's **Robot exports** panel (add, remove, rename, lidar height, roll, pitch): the list is saved to `output/robots.json` and used for every recording after that; the defaults (Go2 45 cm, G1 125 cm) come from [`config/colorizer.yaml`](rgb_lidar_fusion/config/colorizer.yaml). **Roll** (0–180°, about the forward axis: 0 upright, 180 upside down) and **pitch** (−90–90°, nose down positive) describe the lidar **as the robot's driver reports it**: a Livox driver configured with roll 180 (like the G1's) already reports an upside-down lidar upright, so use 0 there. A saved map's *Re-export robots* rewrites its robot copies from the current list (exact: they are a height shift of the saved map).

### ROS interface

| | |
|---|---|
| `/colored_scan`, `/lidar_scan` | Each scan in the `map` frame: colored points (only what the camera sees) / all points with intensity, 10 Hz |
| `/colored_map`, `/lidar_map` | The growing maps while recording, for viewing (1 s / 2 s; thinned to 5 cm / 10 cm, `display_color_voxel` / `display_voxel`; saved maps keep 3 cm) |
| TF `map → camera_init` | The levelled map frame on top of FAST-LIO's world frame |
| `/colorizer/status` | JSON status, 2 Hz |
| `/colorizer/start_map`, `stop_map`, `cancel_map`, `save_cloud` | `std_srvs/Trigger`; name, height and color distance come from the `map_name`, `lidar_height` and `max_range` parameters; `auto_stop_s` (one-shot, 0 = off) stops and saves after that many seconds |

Recording from the command line:

```bash
ros2 param set /colorizer map_name kitchen
ros2 param set /colorizer lidar_height 0.852
ros2 param set /colorizer max_range 10.0        # optional: max color distance, m
ros2 param set /colorizer auto_stop_s 600       # optional: stop and save after 10 min
ros2 service call /colorizer/start_map std_srvs/srv/Trigger
ros2 service call /colorizer/stop_map std_srvs/srv/Trigger    # saves output/kitchen/
```

Launch arguments: `odometry:=false` (stationary rig, no FAST-LIO), `sensors:=false` (no sensor drivers, odometry or mount TF: everything already running elsewhere; to replay a bag see [Record and replay](#record-and-replay)), `web:=false`, `port:=8080`, `rviz:=true`, `accumulate:=true` (start recording at launch), `color_profile:=WxHxFPS`. Colorizer tuning is in [`config/colorizer.yaml`](rgb_lidar_fusion/config/colorizer.yaml).

**How it works.** FAST-LIO2 registers each scan into its world frame and publishes the rig's pose as TF. For each scan the colorizer waits until an image from after the sweep's midpoint has arrived, picks the nearest one (dropping the scan if the gap exceeds `max_dt`, 50 ms), looks up where the camera was **at that image's timestamp** through TF (`camera_init → body → livox_frame → camera_link → camera_color_optical_frame`, interpolated between odometry poses), projects the points, rejects points hidden behind nearer ones, and samples their colors. Scans taken while the rig turns faster than 30°/s are not colored (motion smears color across edges) but still go into the full map. When you press Start, the accelerometer's gravity direction (rig held still) and the height you entered fix the levelled map frame. Images are subscribed **reliable**: a full-resolution frame (6.2 MB at 1080p) is larger than DDS shared memory, and best-effort delivery lost about half the frames in testing. With reliable delivery, 1080p at 15 fps arrives without gaps; with odometry running, over 99% of scans are colored.

### Record and replay

Record the raw sensor data while you map, to re-run the walk later without the rig (for example with other settings, or after a code change). In a second shell (`docker/run.sh`):

```bash
ros2 bag record -o /ws/src/rgb_lidar_fusion/output/<name>_bag /livox/lidar /livox/imu \
  /camera/camera/color/image_raw /camera/camera/color/camera_info /tf /tf_static
```

Raw 1080p images at 15 fps take about 6 GB per minute. Start the bag before you press *Start recording* and stop it (Ctrl+C) after *Stop & save*.

To replay, with the sensors unplugged or their drivers not running, start the odometry and the mount TF without the drivers, then the colorizer and web UI without sensors, then play the bag at normal speed. Leave the recorded `/tf` out, so FAST-LIO computes the poses again:

```bash
ros2 launch rgb_lidar_fusion sensors.launch.py lidar:=false camera:=false     # shell 1
ros2 launch rgb_lidar_fusion colorize.launch.py sensors:=false                # shell 2
ros2 bag play /ws/src/rgb_lidar_fusion/output/<name>_bag --topics /livox/lidar /livox/imu \
  /camera/camera/color/image_raw /camera/camera/color/camera_info /tf_static  # shell 3
```

While the bag plays, press *Start recording* in the web UI at a moment when the rig was held still in the recording, since Start levels the map from that moment. A 48 s bag replayed this way gave the same map as the live recording: median distance between nearest points 0.9 cm, the same colors, every scan colored.

### Calibration

The lidar → camera rotation can be refined on the running rig:

```bash
ros2 run rgb_lidar_fusion calibrate            # 20 s capture, solve, overlays + report
ros2 run rgb_lidar_fusion calibrate --write    # ... and update config/mount_extrinsics.yaml
```

Keep the rig still, facing objects with clear outlines 2–4 m away. The tool keeps only points that stay put (people walking through drop out), uses the median of 9 images, leaves the floor out, refuses a capture where the rig or the odometry moved, and limits the correction to ±5°. It aligns depth edges in the lidar cloud with edges in the image, always comparing the same image cells for every candidate rotation (otherwise the search can pick a lucky subset of cells and look better while being worse). Results and before/after overlays go to `output/calibration/`; `--from <capture>.npz` re-solves a saved capture. Run at least two captures and only `--write` if they agree: on the reference rig four captures agreed within 0.22°, and that calibration is in [`config/mount_extrinsics.yaml`](rgb_lidar_fusion/config/mount_extrinsics.yaml).

## Example output

Two sample maps, each with the colored and the full cloud: a whole floor in [`examples/room_sample/`](examples/room_sample/) (`.ply`) and one area in [`examples/office_sample/`](examples/office_sample/) (`.pcd` and `.ply`); see [`examples/README.md`](examples/README.md) for how to open it.

## Roadmap

- [x] Docker dev environment (ROS 2 Humble, Livox + RealSense drivers)
- [x] Live per-scan colorization node (ROS 2)
- [x] Best-view voxel map for a stationary rig, PCD/PLY export
- [x] Full lidar map saved alongside the colored map
- [x] Web UI: live 3D map, recording controls, saved maps
- [x] Mapping while moving (FAST-LIO2), gravity-levelled maps with the floor at z = 0
- [x] Robot map exports (Unitree Go2, G1)
- [x] LiDAR–camera extrinsic calibration tool (targetless edge alignment)
- [x] ROS 1: use the saved maps on Noetic robots ([`ros1/`](ros1/))
- [x] Calibration guide (see Calibration)
- [x] Sample data and demo GIF
- [ ] Support for other LiDARs and cameras

## Contributing

Issues and pull requests are welcome, especially adapters for other LiDAR and camera hardware; see [CONTRIBUTING.md](CONTRIBUTING.md). Questions and maps you made go in [Discussions](https://github.com/kouya-marino/RGB-LiDAR-Fusion/discussions); it's great to see where it's used. Security problems: [SECURITY.md](SECURITY.md).

## Citation

If you use RGB-LiDAR-Fusion in your work, please cite it (GitHub's **Cite this repository** button gives the same in other formats):

```bibtex
@software{rawat_rgb_lidar_fusion,
  author = {Rawat, Prashant},
  title  = {{RGB-LiDAR-Fusion}: Real-time colorized 3D mapping with a Livox Mid-360 and an RGB camera},
  url    = {https://github.com/kouya-marino/RGB-LiDAR-Fusion},
  year   = {2026},
  version = {0.2.0}
}
```

## Support the project

If this project is useful to you, please give it a ⭐ on GitHub: it helps other people find it.

## License

[MIT](LICENSE). Vendored third-party code in [`third_party/`](third_party/) keeps its own license (Livox SDK 2 and ROS driver: MIT; Three.js: MIT; FAST-LIO: GPL-2.0, run as a separate process).
