FROM osrf/ros:jazzy-desktop

# Install basic developer tools and ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    nano \
    git \
    python3-pip \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    libxcb-cursor0 \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Automatically source ROS 2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
