FROM osrf/ros:jazzy-desktop

# Install basic developer tools
RUN apt-get update && apt-get install -y \
    nano \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Automatically source ROS 2 when you open a terminal in the container
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
