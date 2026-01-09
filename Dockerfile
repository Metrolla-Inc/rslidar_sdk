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

# Initialize submodules if not already present (rs_driver)
# We use a shell conditional to check if the submodule directory is empty
WORKDIR /ros2_ws/src/rslidar_sdk
RUN if [ ! -f src/rs_driver/CMakeLists.txt ]; then \
    git clone https://github.com/RoboSense-LiDAR/rs_driver.git src/rs_driver; \
    fi

# Clone rslidar_msg (required dependency)
WORKDIR /ros2_ws/src
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --event-handlers console_direct+

# Setup entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "rslidar_sdk", "start.py"]

