# ROS 2 Quadruped Locomotion Simulation

This repository contains a ROS 2 (Jazzy) workspace for simulating and controlling a quadruped robot. The project bridges the gap between mechanical kinematics and modern robotics software architecture, utilizing a fully containerized development environment for strict reproducibility.

## System Architecture
* **Framework:** ROS 2 Jazzy Jalisco
* **Environment:** Docker / VS Code DevContainers (Ubuntu base)
* **Physics Engine:** Gazebo
* **Hardware Acceleration:** NVIDIA GPU Passthrough & X11 Display Forwarding

## Open-Loop Trot
put video here
- This trot utilizes sinusoidal kinematics of the diagonal leg pairs, allowing for open-loop movement.
## Closed-Loop Trot
put video here
- This trot utilizes negative feedback loops using EMA filtered IMU data to implement pitch and roll PID controllers.
- NOTE: The PID Controllers have plenty of room for improved tuning, and the system can be improved with a yaw controller for straight-line traversal.

## Prerequisites
To run this project exactly as intended without managing local dependencies, ensure you have the following installed on your host machine:
1. [Docker](https://docs.docker.com/get-docker/)
2. [Visual Studio Code](https://code.visualstudio.com/)
3. The [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension for VS Code
4. NVIDIA Container Toolkit (for GPU passthrough)

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/joshkwka/ros2-quadruped-locomotion.git
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

### 5a. Launch the Visualization
To view the robot model in RViz and interact with the joint states:
```
ros2 launch quadruped_description display.launch.py
```

### 5b. Launch the Gazebo Simulation 
Launch the robot model in a Gazebo physics simulation:
```
ros2 launch quadruped_control sim_launch.py
```

### 6. Run a Locomotion Node
#### 6a. Run the "Push-Up" Node
To test the inverse kinematics, run the pushup_node launch file to see the robot push up and squat down in a loop:
```
ros2 launch quadruped_locomotion pushup_launch.py
```
NOTE: If this step fails, verify the controller state:
```
ros2 control list_controllers
```
If the controllers are inactive, they can be enabled manually:
```
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state leg_controller active
```

#### 6b. Run the Open-Loop Trot
To launch the continuous open-loop trot gait, run:
```
ros2 launch quadruped_locomotion trot_launch.py
```
NOTE: The robot is intended to hold a standing pose for 2 seconds prior to begin trotting.

#### 6c. Run the Closed-Loop Trot
To launch the closed-loop trot gait with IMU-based pitch and roll PID controllers, run:
```
ros2 launch quadruped_locomotion balance_launch.py
```

#### 6d. Run the Teleoperation Node
To control the robot via teleoperation, run:
```
ros2 launch quadruped_locomotion teleop_launch.py
```
In a second terminal, run:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Current Packages
- `quadruped_description`: Contains the URDF, 3D meshes, and physical parameters of the robot. Utilizes the open-source Go1 model provided by [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros). (Completed)
- `quadruped_locomotion`: The high-level "Brain" of the robot. This C++ package utilizes the **Eigen** library for high-performance linear algebra, handling gait scheduling, and Inverse Kinematics (IK). It translates velocity commands into synchronized joint-space trajectories for all 12 degrees of freedom. (Completed)
    - Nodes in this package include: pushup_node, trot_node, balance_node, and teleop_node. These nodes are separated to display the progress through which the project was developed.
    1. pushup_node: A simple node that loops between squatting and standing positions to test the inverse kinematics solver.
    2. trot_node: Utilizes the inverse kinematics solver and implements sinusoidal leg movements to create an open-loop trotting mechanism. The diagonal pairs of legs are symmetrically out of phase with one another for symmetric movement.
    3. balance_node: Subscribes to the IMU and applies an EMA low pass filter on the IMU data. PID controllers are implemented to balance the robot in pitch and roll for a closed-loop trot movement.
    4. teleop_node: Subscribes to standard `geometry_msgs/msg/Twist` commands to enable real-time Drive-by-Wire control. It translates linear and angular velocities into differential step lengths for steering. Features include a slew rate limiter for smooth acceleration and hard kinematic safety clamps to protect the IK solver from exceeding the robot's physical capabilities.
- `quadruped_control`: The low-level "Spine" of the robot. This C++ based package interfaces with `ros2_control` to manage PID loops and hardware abstraction. It takes the target joint angles generated by the locomotion node and converts them into hardware-level commands, managing the `ros2_control` resource manager and joint limits for the Gazebo physics engine. (Completed)

## Learnings & Technical Takeaways
- **Bridging Analytical Math and Physical URDFs:** Standard C++ math libraries evaluate angles based on standard Cartesian quadrants (e.g., std::atan2 treating the positive axis as 0°). However, physical robot URDFs often define 0° as the joint pointing straight down toward gravity. I learned to successfully map textbook trigonometric equations (like the Law of Cosines) to actual motor deflection angles by applying geometric transformations and axis-sign flips to synchronize the software's coordinate frame with the hardware's zero-state.
- **Coordinate Frame Decoupling:** To make the Inverse Kinematics (IK) modular and agnostic to the robot's body width, I separated the kinematics math into two distinct steps: generating high-level targets in the global "Trunk Frame" (center of mass), and translating those into a "Local Hip Frame" before executing the IK solver.
- **Containerized Hardware Acceleration:** Configuring a reproducible DevContainer for robotics requires more than just installing dependencies. I learned how to successfully forward X11 display sockets (/tmp/.X11-unix) and configure NVIDIA GPU passthrough into a Docker container to enable real-time, hardware-accelerated GUI applications like RViz without polluting the host machine.
- **Continuous Trajectory Generation & Phase Normalization:** Transitioning from discrete static poses to a fluid walking gait required mapping the continuous time domain into a normalized phase (0.0 to 1.0). This allowed me to decouple the diagonal leg pairs using a 180-degree phase offset.
- **Center of Mass (CoM) Compensation:** The Go1 robot is back-heavy, causing the rear legs to sag and the body to pitch upwards during open-loop movement. This called for a closed-loop solution to actively balance the body.
- **Phase-Aware Closed-Loop Control:** Implementing a raw PID controller to stabilize the robot's pitch and roll was initially insufficient, as applying a global Z-offset penalized the swing legs and caused them to drag. I separated the "standing" and "swinging" states of the legs such that the PID correction factors were only applied to the diagonal pair of legs that are in contact with the ground.

## Roadmap / Next Steps
- [x] **URDF & Simulation Environment:** Stable ROS 2 / Gazebo bridging.
- [x] **Inverse Kinematics:** 3D Cartesian space to Joint space mapping.
- [x] **Open-Loop Locomotion:** Continuous Trot gait via trajectory generation.
- [x] **Closed-Loop Balancing:** Subscribe to IMU data (Roll/Pitch) and implement a dynamic PID controller to automatically adjust leg extensions and stabilize the chassis against disturbances. 
- [x] **Teleoperation:** Map standard `cmd_vel` Twist messages from a keyboard to dynamic stride length and heading changes.
- [ ] **Autonomous Mapping & Navigation:** Integrate `slam_toolbox` to generate 2D occupancy grids of the Gazebo environment, and utilize the ROS 2 `Nav2` stack for dynamic path planning and obstacle avoidance.
- [ ] **Vision-Based Perception:** Equip the robot with a simulated RGB-D camera to process point clouds (via OpenCV/PCL). Utilize this depth data for terrain elevation mapping to enable intelligent, dynamic foothold selection on uneven ground.
- [ ] **Reinforcement Learning (RL) Locomotion:** Transition from heuristic-based PID control to AI-driven locomotion. Utilize NVIDIA Isaac Sim / Isaac Gym to train robust, neural network-based walking policies using Deep RL, focusing on dynamic fall recovery and "sim-to-real" transferability.