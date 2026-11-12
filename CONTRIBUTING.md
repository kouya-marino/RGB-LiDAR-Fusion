# Contributing

Thanks for your interest in RGB-LiDAR-Fusion. Bug reports, questions, maps you made with it, and pull requests are all welcome.

## Questions and ideas

- **Questions, help, show your maps:** use [Discussions](https://github.com/kouya-marino/RGB-LiDAR-Fusion/discussions).
- **Bugs:** open an [issue](https://github.com/kouya-marino/RGB-LiDAR-Fusion/issues/new/choose) with the bug template. Logs from `ros2 launch` and your sensor models help a lot.
- **Security problems:** please don't open a public issue; see [SECURITY.md](SECURITY.md).

## Development setup

Everything runs in the Docker image (see the README's *Installation*):

```bash
docker/build.sh && docker/run.sh
cd /ws && colcon build && source install/setup.bash
```

The repo is mounted at `/ws/src/rgb_lidar_fusion`, so edits on the host are live in the container (rebuild with `colcon build` for launch/config changes).

## Tests

Run the unit tests inside the container; they need no sensors:

```bash
cd /ws/src/rgb_lidar_fusion/rgb_lidar_fusion
python3 -m pytest -q test
```

Please add tests for new behavior, and keep them independent of hardware and of anything in `output/`.

## Pull requests

1. Fork the repo and create a branch from `main` (`feat/...`, `fix/...`, `docs/...`).
2. Keep a PR to one change; describe what it does and how you tested it (with hardware or with tests).
3. Update the README or the relevant docs when behavior or parameters change.
4. Make sure the tests pass.

PRs are squash-merged into `main`.

### Especially welcome

- Adapters for **other LiDARs** (other Livox models, Ouster, Hesai, ...) and **other RGB cameras** with a ROS 2 driver
- Mapping quality: removing moving objects from the full map, loop closure
- Mount variants for other sensor combinations (`hardware/`)

## Code style

- Python: follow the style of the surrounding code (PEP 8, type hints where they help, docstrings that say *why*).
- Vendored code in `third_party/` keeps its upstream style and license; change it only when needed and note it in `third_party/README.md`.

By contributing you agree that your contributions are licensed under the [MIT License](LICENSE), and to follow the [Code of Conduct](CODE_OF_CONDUCT.md).
