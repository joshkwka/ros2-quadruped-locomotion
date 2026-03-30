#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "quadruped_locomotion/ik_solver.hpp"

using namespace std::chrono_literals;

class TrotNode : public rclcpp::Node {
public:
    TrotNode() : Node("trot_node"), first_run_(true) {
        // Publish joint trajectories to the leg controller
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leg_controller/joint_trajectory", 10);

        // 50Hz (20ms) control loop
        timer_ = this->create_wall_timer(
            20ms, std::bind(&TrotNode::publish_trajectory, this));
            
        RCLCPP_INFO(this->get_logger(), "Node initialized. Commencing Walk.");
    }

private:
    void publish_trajectory() {

        //////////////////////////////////////////////////////////////////////////////
        ///////                     Define target positions                    ///////
        //////////////////////////////////////////////////////////////////////////////
        
        auto current_time_obj = this->now();
        double current_time = current_time_obj.seconds();

        // Initialize the timer on the very first loop
        if (first_run_) {
            if (current_time == 0.0) {
                // If clock hasn't started yet, wait.
                return; 
            }
            start_time_ = current_time;
            first_run_ = false;
        }

        double time_elapsed = current_time - start_time_;

        // Trot trajectory parameters
        const double L = 0.12;         // Step length (m)
        const double h = 0.05;         // Step height (m)
        const double H_stand = 0.28;   // Standing height (m)
        const double T = 1.0;          // Time for one full stride (second(s))
        const double W = 0.10;         // Sprawl width (cm)
        // double balance_offset = 0.0;   // Offset to balance the robot 

        // Get robot to initially stand
        Eigen::Vector3d pos_A(0.0, 0.0, -H_stand);
        Eigen::Vector3d pos_B(0.0, 0.0, -H_stand);

        // Start walking after time_wait seconds of standing still
        double time_wait = 2.0;
        if (time_elapsed > time_wait) {
            double walk_time = time_elapsed - time_wait;

            // Offset left/right legs by 180 degeree (0.5 in phase)
            // Phase [0.0 to 1.0) for both diagonal pairs
            double phase_A = fmod(walk_time / T, 1.0);
            double phase_B = fmod((walk_time / T) + 0.5, 1.0);

            // Lambda to calculate foot position based on its phase
            auto get_foot_pos = [&](double phase) -> Eigen::Vector3d {
                // X sweeps back and forth: starts back, moves forward, goes back
                double x = (L / 2.0) * cos(2 * M_PI * phase);
                double z = -H_stand;
                
                // If in the first half of the phase, lift the foot (Swing Phase)
                if (phase < 0.5) { 
                    z += h * sin(2 * M_PI * phase); 
                }
                
                return Eigen::Vector3d(x, 0.0, z); // Temporarily set Y to 0
            };

            pos_A = get_foot_pos(phase_A);
            pos_B = get_foot_pos(phase_B);

            // balance_offset = -0.02;  // Offset to balance the robot 

        }

        // Set Y positions for the legs based on the sprawl width (W)
        // Pair A: Front Left & Rear Right
        Eigen::Vector3d pos_FL(pos_A.x(), W, pos_A.z());
        Eigen::Vector3d pos_RR(pos_A.x(), -W, pos_A.z());
        // Eigen::Vector3d pos_RR(pos_A.x(), -W, pos_A.z() + balance_offset);

        // Pair B: Front Right & Rear Left
        Eigen::Vector3d pos_FR(pos_B.x(), -W, pos_B.z());
        Eigen::Vector3d pos_RL(pos_B.x(), W, pos_B.z());
        // Eigen::Vector3d pos_RL(pos_B.x(), W, pos_B.z() + balance_offset);

        //////////////////////////////////////////////////////////////////////////////

        // Calculate IK
        Eigen::Vector3d FL_angles = solver_.solveLegIK(pos_FL, true);  
        Eigen::Vector3d RR_angles = solver_.solveLegIK(pos_RR, false); 
        Eigen::Vector3d FR_angles = solver_.solveLegIK(pos_FR, false); 
        Eigen::Vector3d RL_angles = solver_.solveLegIK(pos_RL, true);  

        // Construct and publish the ROS 2 message
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.header.stamp = current_time_obj;
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
        
        // Execute movement over 20ms (one time-step)
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 20000000; 
        
        msg.points.push_back(point);
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Trotting... Time Elapsed: %.2f seconds", time_elapsed);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    quadruped_locomotion::IKSolver solver_;
    
    double start_time_;
    bool first_run_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrotNode>());
    rclcpp::shutdown();
    return 0;
}