# Third-party code

Vendored copies of upstream projects, so building this repo does not depend on
fetching them. Each folder is an unmodified copy of the upstream tree at the
listed version, without its git history (for `fast_lio`, also without its
`doc/`, `PCD/` and `Log/` folders: 128 MB of images and sample data the build
does not need). Their license files are kept as-is and apply to everything in
that folder.

| Folder | Upstream | Version | Commit | License |
|---|---|---|---|---|
| [`livox/Livox-SDK2`](livox/Livox-SDK2) | https://github.com/Livox-SDK/Livox-SDK2 | v1.3.1 | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4` | MIT ([`LICENSE.txt`](livox/Livox-SDK2/LICENSE.txt)); bundled libraries under `3rdparty/` carry their own licenses |
| [`livox/livox_ros_driver2`](livox/livox_ros_driver2) | https://github.com/Livox-SDK/livox_ros_driver2 | 1.2.6 | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | MIT ([`LICENSE.txt`](livox/livox_ros_driver2/LICENSE.txt)) |
| [`fast_lio`](fast_lio) | https://github.com/hku-mars/FAST_LIO (branch `ROS2`) | ROS2 branch | `cc3c9e8b1469b33b8072764d6db2348ed6dc597f`; `include/ikd-Tree` from https://github.com/hku-mars/ikd-Tree at `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` | GPL-2.0 ([`LICENSE`](fast_lio/LICENSE)); runs as its own process, our code talks to it only over ROS topics |
| [`three`](three) | https://github.com/mrdoob/three.js (npm `three`) | 0.160.0 | `build/three.module.min.js`, `examples/jsm/controls/OrbitControls.js` only | MIT ([`LICENSE`](three/LICENSE)) |

The Livox versions add Mid-360S support and are built into the Docker image by
[`docker/Dockerfile`](../docker/Dockerfile); `COLCON_IGNORE` stops colcon from
picking them up when the repo is mounted into the workspace. Three.js is
installed with the `rgb_lidar_fusion` package and served by its web UI, so the
page works without internet access.

To update: replace the folder with the new upstream tree (`git archive <tag>`),
update this table, rebuild the image, and check the lidar still streams.
