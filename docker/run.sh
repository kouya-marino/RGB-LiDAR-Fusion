#!/usr/bin/env bash
# Start the dev container, or open another shell in it if it's already running.
#   docker/run.sh                 -> interactive shell
#   docker/run.sh <cmd> [args]    -> run a command
set -euo pipefail
NAME=rgb_lidar_fusion
IMAGE=rgb-lidar-fusion:humble
REPO="$(cd "$(dirname "$0")/.." && pwd)"

if [ -n "$(docker ps -q -f name="^${NAME}$")" ]; then
    exec docker exec -it "$NAME" /entrypoint.sh "${@:-bash}"
fi

# A stopped container with this name (e.g. one started with `docker run -d`
# and stopped by a reboot) would block the name. It holds nothing worth
# keeping: maps are in the repo's output/ and builds are in named volumes.
if [ -n "$(docker ps -aq -f name="^${NAME}$")" ]; then
    echo "removing the stopped '$NAME' container left from an earlier run"
    docker rm "$NAME" >/dev/null
fi

# Let this user's container processes open windows (RViz) on the host X server
xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || true

# --network host : Mid-360 uses UDP broadcast discovery, which port mapping breaks
# --privileged + /dev : RealSense USB access, incl. re-enumeration on reset
exec docker run -it --rm --name "$NAME" \
    --network host --ipc host --privileged \
    -v /dev:/dev \
    -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /etc/localtime:/etc/localtime:ro \
    -v "$REPO":/ws/src/rgb_lidar_fusion \
    -v rgb_lidar_fusion_build:/ws/build \
    -v rgb_lidar_fusion_install:/ws/install \
    -v rgb_lidar_fusion_log:/ws/log \
    "$IMAGE" "${@:-bash}"
