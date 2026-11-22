#!/bin/bash
# Metrolla driver contract, opt-in (docs/METROLLA_DRIVER_CONTRACT.md). Only when
# METROLLA_DRIVER_CONTRACT is set and not "0" does the image switch to the environment contract;
# otherwise everything below this block is the v1.5.18 entrypoint, byte for byte (tests/legacy pins it).
if [ -n "${METROLLA_DRIVER_CONTRACT:-}" ] && [ "${METROLLA_DRIVER_CONTRACT}" != "0" ]; then
    exec /opt/metrolla/rslidar/contract-entrypoint.sh "$@"
fi
# --- end of the Metrolla contract block; the v1.5.18 entrypoint follows ---
set -e

# Source ROS2 Jazzy base
source "/opt/ros/jazzy/setup.bash"

# Source the local workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source "/ros2_ws/install/setup.bash"
fi

exec "$@"