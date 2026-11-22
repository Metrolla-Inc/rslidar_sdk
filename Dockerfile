FROM ros:jazzy-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-rviz2 \
    libyaml-cpp-dev \
    libpcap-dev \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws/src
COPY . rslidar_sdk/

# rs_driver is the submodule src/rs_driver, pinned by this commit. Refuse to build without it:
# cloning rs_driver's default branch here (as v1.5.18 did) builds whatever the vendor has that day.
WORKDIR /ros2_ws/src/rslidar_sdk
RUN if [ ! -f src/rs_driver/CMakeLists.txt ]; then \
    echo "src/rs_driver is empty: check out the submodule (git submodule update --init) before building" >&2; \
    exit 1; \
    fi

# rslidar_msg (required dependency), pinned: fe8a95c is its newest commit since 2022, the one every
# earlier image got from an unpinned clone.
ARG RSLIDAR_MSG_COMMIT=fe8a95cb242bd294cc3d5e3422f2093fb49a56ee
WORKDIR /ros2_ws/src
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git \
    && git -C rslidar_msg checkout --quiet "${RSLIDAR_MSG_COMMIT}" \
    && test "$(git -C rslidar_msg rev-parse HEAD)" = "${RSLIDAR_MSG_COMMIT}"

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --event-handlers console_direct+

# Metrolla driver contract (opt-in with METROLLA_DRIVER_CONTRACT=1; docs/METROLLA_DRIVER_CONTRACT.md).
# selfcheck renders every model against the vendor config this image reads, so a vendor update that
# renames or duplicates a key the contract sets fails here, not on a unit.
COPY docker/contract-entrypoint.sh docker/rslidar_env.py /opt/metrolla/rslidar/
RUN chmod 0755 /opt/metrolla/rslidar/contract-entrypoint.sh /opt/metrolla/rslidar/rslidar_env.py \
    && python3 -I /opt/metrolla/rslidar/rslidar_env.py selfcheck

# Setup entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "rslidar_sdk", "start.py"]
