# ROS 2 Quadruped Locomotion Simulation

This repository contains a ROS 2 (Jazzy) workspace for simulating and controlling a quadruped robot. The project bridges the gap between mechanical kinematics and modern robotics software architecture, utilizing a fully containerized development environment for strict reproducibility.

## System Architecture
* **Framework:** ROS 2 Jazzy Jalisco
* **Environment:** Docker / VS Code DevContainers (Ubuntu base)
* **Physics Engine:** Gazebo / Isaac Sim (TBD)
* **Hardware Acceleration:** NVIDIA GPU Passthrough & X11 Display Forwarding

## Prerequisites
To run this project exactly as intended without managing local dependencies, ensure you have the following installed on your host machine:
1. [Docker](https://docs.docker.com/get-docker/)
2. [Visual Studio Code](https://code.visualstudio.com/)
3. The [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension for VS Code
4. NVIDIA Container Toolkit (for GPU passthrough)

## Getting Started

### 1. Clone the Repository
```bash
git clone [https://github.com/joshkwka/ros2-quadruped-locomotion.git](https://github.com/joshkwka/ros2-quadruped-locomotion.git)
cd ros2-quadruped-locomotion
```

### 2. Open in VS Code
Open the cloned directory in Visual Studio Code:
```
code .
```
### 3. Launch the DevContainer
This project uses a `.devcontainer` to automatically install ROS 2, configure dependencies, and handle X11/GPU forwarding.
1. In VS Code, open the Command Palette (Ctrl+Shift+P on Windows/Linux, Cmd+Shift+P on Mac).
2. Type and select: `Dev Containers: Reopen in Container`.
3. Wait a few moments for the Docker image to build and attach. Once connected, open a new terminal in VS Code. You will now be operating as `root` inside the container with ROS 2 automatically sourced.

### 4. Build the Workspace
Navigate to the root of the workspace inside the container and compile the packages:
```
colcon build
source install/setup.bash
```

## Current Packages
- `quadruped_description`: Contains the URDF, 3D meshes, and physical parameters of the robot. (In Progress)