# Tools

Scripts that turn saved maps into the numbers and figures used in the paper. They
run inside the container (`docker/run.sh`), from the repo root
(`/ws/src/rgb_lidar_fusion`), on maps saved in `output/`.

| Script | What it does |
|---|---|
| `walk_report.py` | Prints a recording's numbers: scan rate per minute, scans received vs expected, scans colored and why the others were not, camera losses, callback times, and the start-to-end position difference. Also saves `top_view.png` in the map folder: the map from above with the walked path, start and end. |
| `render_map.py` | Renders the full-floor figures: a three-panel top-down view (lidar + camera, camera colors, lidar by height with the path) and an oblique 3D view. Optional cleaned clouds (e.g. edited in a point cloud editor) replace the saved ones. |
| `render_orbit_gif.py` | Renders the README's animated full-floor view: three synchronized views (lidar + camera, camera colors, lidar by height) swaying around the map. |
| `replay_walk.py` | Replays a saved walk through the real colorizer node in simulated time, at 10 Hz with the node's 5-scan queue, and reports how many scans it keeps up with. Images and coloring are not simulated: it measures the map work that grows with the map. Needs ROS (the container). |

```bash
python3 tools/walk_report.py output/<map>
python3 tools/render_map.py output/<map> <out_dir> [<colored_cleaned.pcd> [<full_cleaned.pcd>]]
python3 tools/render_orbit_gif.py examples/full_floor_sample docs/images/demo_full_room.gif
python3 tools/replay_walk.py output/<map> <label> [minutes]
```

`walk_report.py` prints the recording stats only for maps saved by a version
that writes them to `info.json` (v0.2.0 and later).
