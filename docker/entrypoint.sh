#!/bin/bash
# Source ROS, the bridge underlay and (if built) your workspace, then run the command.
set -e
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /opt/ifs_ws/install/setup.bash
if [ -f /sandbox/ros2_ws/install/local_setup.bash ]; then
    # shellcheck disable=SC1091
    source /sandbox/ros2_ws/install/local_setup.bash
fi
# Shared memory left over by nodes killed in a previous session.
fastdds shm clean >/dev/null 2>&1 || true
exec "$@"
