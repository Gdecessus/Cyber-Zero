FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Base tooling for building ROS2 workspaces
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (safe to re-run)
RUN rosdep init 2>/dev/null || true && rosdep update

WORKDIR /opt/roarm_ws

# Python deps
COPY requirements.txt /opt/roarm_ws/requirements.txt
RUN pip3 install --no-cache-dir -r /opt/roarm_ws/requirements.txt

# Copy sources
COPY src /opt/roarm_ws/src

# Resolve system deps and build
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Default shell with environment ready
CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.sh && source /opt/roarm_ws/install/setup.sh && exec bash"]
