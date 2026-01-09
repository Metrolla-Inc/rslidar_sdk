#!/bin/bash
set -e

# Source ROS2 Jazzy base
source "/opt/ros/jazzy/setup.bash"

# Source the local workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source "/ros2_ws/install/setup.bash"
fi

exec "$@"