#!/usr/bin/env bash
# Build the dev image with your host UID/GID baked in.
set -euo pipefail
cd "$(dirname "$0")/.."      # repo root is the build context (vendored Livox sources)
docker build -f docker/Dockerfile \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    -t rgb-lidar-fusion:humble \
    "$@" .
