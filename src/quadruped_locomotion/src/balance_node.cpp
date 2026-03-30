#include <chrono>
#include <memory>
#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "quadruped_locomotion/ik_solver.hpp"

using namespace std::chrono_literals;

class BalanceNode : public rclcpp::Node {
public:
    BalanceNode() : Node("balance_node"), first_run_(true), current_pitch_(0.0), 
    previous_pitch_error_(0.0), integral_pitch_error_(0.0), current_roll_(0.0), 
    previous_roll_error_(0.0), integral_roll_error_(0.0) {
        // Publish joint trajectories to the leg controller
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leg_controller/joint_trajectory", 10);

        // Subscribe to IMU data
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&BalanceNode::imu_callback, this, std::placeholders::_1));

        // 100Hz (10ms) control loop
        timer_ = this->create_wall_timer(
            10ms, std::bind(&BalanceNode::publish_trajectory, this));
            
        RCLCPP_INFO(this->get_logger(), "Closed-Loop Balance Node initialized.");
    }

private:
    //////////////////////////////////////////////////////////////////////////////
    ///////                        Process IMU Data                        ///////
    //////////////////////////////////////////////////////////////////////////////

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Extract quaternion
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        // Convert quaternion to pitch (Y-axis rotation)
        double sinp = 2.0 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            // Gimbal lock protection
            current_pitch_ = std::copysign(M_PI / 2.0, sinp); 
        else
            current_pitch_ = std::asin(sinp);
        
        // Convert quaternion to roll (X-axis rotation)
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        current_roll_ = std::atan2(sinr_cosp, cosr_cosp);
    }

    //////////////////////////////////////////////////////////////////////////////

    

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
        const double L = 0.10;         // Step length (10 cm)
        const double h = 0.08;         // Step height (8 cm)
        const double H_stand = 0.28;   // Standing height (28 cm)
        const double T = 1.0;          // Time for one full stride (1 second(s))
        const double W = 0.10;         // Sprawl width (10 cm)

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
        }

        //////////////////////////////////////////////////////////////////////////////
        ///////                       Pitch PID Correction                     ///////
        //////////////////////////////////////////////////////////////////////////////

        double dt = 0.01; // Time step (10ms))

        // Pitch PID
        double target_pitch = 0.05;
        double pitch_error = target_pitch - current_pitch_; 
        double Kp_pitch = 0.4; // Proportional Constant
        double Kd_pitch = 0.03; // Derivative Constant
        double Ki_pitch = 0.002; // Integral Constant
        double pitch_correction = 0.0;
        
        // Roll PID
        double target_roll = 0.0;
        double roll_error = target_roll - current_roll_;
        double Kp_roll = 0.5; 
        double Kd_roll = 0.005;
        double Ki_roll = 0.001;
        double roll_correction = 0.0;

        // ONLY apply the PID balancer when walking
        if (time_elapsed > time_wait) {
            // Calculate pitch correction
            double pitch_derivative = (pitch_error - previous_pitch_error_) / dt;
            integral_pitch_error_ += (pitch_error * dt);
            integral_pitch_error_ = std::clamp(integral_pitch_error_, -0.5, 0.5); 
            pitch_correction = 
                (Kp_pitch * pitch_error) + 
                (Ki_pitch * integral_pitch_error_) + 
                (Kd_pitch * pitch_derivative);
            pitch_correction = std::clamp(pitch_correction, -0.04, 0.04);

            // Calculate roll correction
            double roll_derivative = (roll_error - previous_roll_error_) / dt;
            integral_roll_error_ += (roll_error * dt);
            integral_roll_error_ = std::clamp(integral_roll_error_, -0.5, 0.5);
            roll_correction = 
                (Kp_roll * roll_error) + 
                (Ki_roll * integral_roll_error_) + 
                (Kd_roll * roll_derivative);
            roll_correction = std::clamp(roll_correction, -0.03, 0.03);

        } else {
            // Keep the integral error cleared out while dropping/standing
            integral_pitch_error_ = 0.0;
            integral_roll_error_ = 0.0;
        }

        previous_pitch_error_ = pitch_error;
        previous_roll_error_ = roll_error;

        // Apply corrections, and sprawl width
        // Pair A: Front Left & Rear Right
        Eigen::Vector3d pos_FL(pos_A.x(), W, pos_A.z() + pitch_correction - roll_correction);
        Eigen::Vector3d pos_RR(pos_A.x(), -W, pos_A.z() - pitch_correction + roll_correction);
        // Pair B: Front Right & Rear Left
        Eigen::Vector3d pos_FR(pos_B.x(), -W, pos_B.z() + pitch_correction + roll_correction);
        Eigen::Vector3d pos_RL(pos_B.x(), W, pos_B.z() - pitch_correction - roll_correction);

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

        // Execute movement over 10ms (one time-step)
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 10000000; 
        
        msg.points.push_back(point);
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "(Balanced) Trotting... Time Elapsed: %.2f seconds", time_elapsed);
        RCLCPP_INFO(this->get_logger(), "Pitch: %.2f DEG", current_pitch_*180/M_PI);
        RCLCPP_INFO(this->get_logger(), "Roll: %.2f DEG", current_roll_*180/M_PI);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    quadruped_locomotion::IKSolver solver_;
    
    double start_time_;
    bool first_run_;

    double current_pitch_;
    double previous_pitch_error_;
    double integral_pitch_error_;

    double current_roll_;
    double previous_roll_error_;
    double integral_roll_error_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalanceNode>());
    rclcpp::shutdown();
    return 0;
}