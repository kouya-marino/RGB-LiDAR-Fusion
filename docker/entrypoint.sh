#!/usr/bin/env bash
# Sources ROS 2, the Livox driver underlay and (if built) this workspace.
# Used both as the container entrypoint and from ~/.bashrc.
source /opt/ros/humble/setup.bash
source /opt/livox_ws/install/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

# Only exec when run as the entrypoint, not when sourced from .bashrc
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    exec "$@"
fi
