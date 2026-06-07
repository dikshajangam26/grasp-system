#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace, but suppress errors from chained/stale paths
if [ -f "/ros2_ws/install/setup.bash" ]; then
    # We use '|| true' to prevent the container from exiting if the setup script 
    # references a stale, non-existent path
    source /ros2_ws/install/setup.bash || true
fi

# Execute the command
exec "$@"