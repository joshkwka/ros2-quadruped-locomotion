FROM osrf/ros:jazzy-desktop

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install tools, build essentials, and Gazebo Sim suite
RUN apt-get update && apt-get install -y \
    nano \
    git \
    python3-pip \
    build-essential \
    python3-colcon-common-extensions \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz \
    libxcb-cursor0 \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Automatically source ROS 2 and your workspace for every new terminal
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
# RUN echo "if [ -f /workspaces/quadruped_ws/install/setup.bash ]; then source /workspaces/quadruped_ws/install/setup.bash; fi" >> /root/.bashrc

WORKDIR /workspaces/quadruped_ws