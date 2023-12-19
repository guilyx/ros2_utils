FROM osrf/ros:humble-desktop

# Install build tools and ROS2 dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Create a workspace
WORKDIR /home/ros2_ws/src

# Copy the repository contents into the Docker image
COPY . /home/ros2_ws/src

# Install dependencies
RUN rosdep install --from-paths . --ignore-src -r -y

# Build the ROS2 packages
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; cd /home/ros2_ws; colcon build --symlink-install'

WORKDIR /home/ros2_ws
