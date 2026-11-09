#!/usr/bin/env bash
# Build the dev image with your host UID/GID baked in.
set -euo pipefail
cd "$(dirname "$0")"
docker build \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    -t rgb-lidar-fusion:humble \
    "$@" .
