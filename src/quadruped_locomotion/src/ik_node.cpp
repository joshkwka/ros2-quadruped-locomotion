#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "quadruped_locomotion/ik_solver.hpp"

// Quick helper to convert radians to degrees for readability
double rad2deg(double radians) {
    return radians * (180.0 / M_PI);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ik_node");
    
    RCLCPP_INFO(node->get_logger(), "Initializing IK Solver Test...");

    // 1. Instantiate your new solver
    quadruped_locomotion::IKSolver solver;

    // 2. Define a target coordinate (X: 0, Y: 0, Z: -0.3 meters)
    // This is asking the foot to step straight down, 30cm below the hip motor.
    Eigen::Vector3d target_xyz(0.0, 0.0, -0.30);
    
    // 3. We will test the Front-Right leg (is_left_leg = false)
    bool is_left_leg = false;

    // 4. Run the math!
    Eigen::Vector3d joint_angles = solver.solveLegIK(target_xyz, is_left_leg);

    // 5. Print the results
    std::cout << "\n--- IK SOLVER RESULTS ---" << std::endl;
    std::cout << "Target XYZ : [" << target_xyz.x() << ", " << target_xyz.y() << ", " << target_xyz.z() << "]" << std::endl;
    std::cout << "Leg Side   : " << (is_left_leg ? "LEFT" : "RIGHT") << std::endl;
    std::cout << "-------------------------" << std::endl;
    std::cout << "Hip Roll   : " << joint_angles[0] << " rad  (" << rad2deg(joint_angles[0]) << " deg)" << std::endl;
    std::cout << "Thigh Pitch: " << joint_angles[1] << " rad  (" << rad2deg(joint_angles[1]) << " deg)" << std::endl;
    std::cout << "Calf Pitch : " << joint_angles[2] << " rad  (" << rad2deg(joint_angles[2]) << " deg)" << std::endl;
    std::cout << "-------------------------\n" << std::endl;

    rclcpp::shutdown();
    return 0;
}