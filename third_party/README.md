# Third-party code

Vendored copies of upstream projects, so building this repo does not depend on
fetching them. Each folder is an unmodified copy of the upstream tree at the
listed version, without its git history. Their license files are kept as-is
and apply to everything in that folder.

| Folder | Upstream | Version | Commit | License |
|---|---|---|---|---|
| [`livox/Livox-SDK2`](livox/Livox-SDK2) | https://github.com/Livox-SDK/Livox-SDK2 | v1.3.1 | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4` | MIT ([`LICENSE.txt`](livox/Livox-SDK2/LICENSE.txt)); bundled libraries under `3rdparty/` carry their own licenses |
| [`livox/livox_ros_driver2`](livox/livox_ros_driver2) | https://github.com/Livox-SDK/livox_ros_driver2 | 1.2.6 | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | MIT ([`LICENSE.txt`](livox/livox_ros_driver2/LICENSE.txt)) |
| [`three`](three) | https://github.com/mrdoob/three.js (npm `three`) | 0.160.0 | `build/three.module.min.js`, `examples/jsm/controls/OrbitControls.js` only | MIT ([`LICENSE`](three/LICENSE)) |

The Livox versions add Mid-360S support and are built into the Docker image by
[`docker/Dockerfile`](../docker/Dockerfile); `COLCON_IGNORE` stops colcon from
picking them up when the repo is mounted into the workspace. Three.js is
installed with the `rgb_lidar_fusion` package and served by its web UI, so the
page works without internet access.

To update: replace the folder with the new upstream tree (`git archive <tag>`),
update this table, rebuild the image, and check the lidar still streams.
