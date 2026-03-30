#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "quadruped_locomotion/ik_solver.hpp"

using namespace std::chrono_literals;

class PushupNode : public rclcpp::Node {
public:
    PushupNode() : Node("pushup_node"), is_standing_(true) {
        // Publish joint trajectories to the leg controller
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leg_controller/joint_trajectory", 10);

        // Timer set to 2 seconds to allow the robot to finish the movement
        timer_ = this->create_wall_timer(
            2000ms, std::bind(&PushupNode::publish_trajectory, this));
            
        RCLCPP_INFO(this->get_logger(), "Node Initialized. Commencing pushups.");
    }

private:
    void publish_trajectory() {

        //////////////////////////////////////////////////////////////////////////////
        ///////                     Define target positions                    ///////
        //////////////////////////////////////////////////////////////////////////////

        // Toggle the target height between crouch and stand
        double z_target = is_standing_ ? -0.32 : -0.20;
        is_standing_ = !is_standing_;

        // Set left and right leg targets
        Eigen::Vector3d left_target(0.0, 0.05, z_target);
        Eigen::Vector3d right_target(0.0, -0.05, z_target);

        //////////////////////////////////////////////////////////////////////////////
        
        // Calculate IK
        Eigen::Vector3d FL_angles = solver_.solveLegIK(left_target, true); 
        Eigen::Vector3d FR_angles = solver_.solveLegIK(right_target, false); 
        Eigen::Vector3d RL_angles = solver_.solveLegIK(left_target, true);  
        Eigen::Vector3d RR_angles = solver_.solveLegIK(right_target, false); 

        // Construct and publish the ROS 2 message
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.header.stamp = this->now();
        msg.joint_names = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {
            FL_angles[0], FL_angles[1], FL_angles[2],
            FR_angles[0], FR_angles[1], FR_angles[2],
            RL_angles[0], RL_angles[1], RL_angles[2],
            RR_angles[0], RR_angles[1], RR_angles[2]
        };

        // Execute movement over 1.0 second
        point.time_from_start.sec = 1;
        point.time_from_start.nanosec = 0;

        msg.points.push_back(point);
        publisher_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Publishing target Z: %.2f", z_target);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    quadruped_locomotion::IKSolver solver_;
    bool is_standing_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PushupNode>());
    rclcpp::shutdown();
    return 0;
}