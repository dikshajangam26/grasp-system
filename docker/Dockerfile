FROM docker.io/library/ros:humble-ros-base

# 1. Install system and ROS 2 dependencies (Including our Vision/OpenCV fixes)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-setuptools \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-ackermann-msgs \
    ros-humble-action-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# 2. Setup a clean workspace directory
WORKDIR /ros2_ws

# 3. Install Python dependencies from requirements.txt
# Doing this BEFORE copying the source code caches the pip install layer!
COPY requirements.txt /tmp/
RUN pip install --no-cache-dir -r /tmp/requirements.txt

# 4. Copy ONLY your source code, not the build artifacts
COPY ros2_ws/src/ ./src/

# 5. Build the workspace
# We build without symlinks to ensure paths are absolute and reliable
RUN /bin/bash -c " \
    unset AMENT_PREFIX_PATH && \
    unset PYTHONPATH && \
    unset COLCON_PREFIX_PATH && \
    source /opt/ros/humble/setup.bash && \
    colcon build --event-handlers console_direct+ \
    "

# 6. Entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]