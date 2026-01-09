# Physical AI and Humanoid Robotics Textbook
# ROS2 Humble + Gazebo Development Environment
#
# Build: docker build -t physical-ai-textbook:ros2-humble .
# Run:   docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix physical-ai-textbook:ros2-humble

FROM osrf/ros:humble-desktop

LABEL maintainer="Physical AI Textbook Team"
LABEL description="ROS2 Humble development environment for Physical AI textbook labs"
LABEL version="1.0.0"

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install additional ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-rqt* \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for robotics development
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# Install MuJoCo for Python
RUN pip3 install --no-cache-dir \
    mujoco>=3.0.0 \
    pyyaml \
    jupyter \
    jupyterlab \
    ipywidgets

# Create workspace directory
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]
