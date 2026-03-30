#include <chrono>
#include <memory>
#include <algorithm>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <csignal>
#include <thread>

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

        // Subscribe to ROS 2 cmd_vel for teleoperation commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                target_vx_ = msg->linear.x;
                target_wz_ = msg->angular.z;
            });

        // 100Hz (10ms) control loop
        timer_ = this->create_wall_timer(
            10ms, std::bind(&BalanceNode::publish_trajectory, this));
            
        RCLCPP_INFO(this->get_logger(), "Closed-Loop Balance & Teleop Node initialized.");
    }

    void publish_standing_pose() {
        RCLCPP_INFO(this->get_logger(), "Intercepted Ctrl+C. Resetting stance...");

        const double H_stand = 0.28;
        const double W = 0.10;
        
        Eigen::Vector3d pos_A(0.0, 0.0, -H_stand);
        Eigen::Vector3d pos_FL(pos_A.x(), W, pos_A.z());
        Eigen::Vector3d pos_RR(pos_A.x(), -W, pos_A.z());
        Eigen::Vector3d pos_FR(pos_A.x(), -W, pos_A.z());
        Eigen::Vector3d pos_RL(pos_A.x(), W, pos_A.z());

        Eigen::Vector3d FL_angles = solver_.solveLegIK(pos_FL, true);  
        Eigen::Vector3d RR_angles = solver_.solveLegIK(pos_RR, false); 
        Eigen::Vector3d FR_angles = solver_.solveLegIK(pos_FR, false); 
        Eigen::Vector3d RL_angles = solver_.solveLegIK(pos_RL, true);  

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
        
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 500000000; 
        
        msg.points.push_back(point);
        publisher_->publish(msg);
    }

private:
    //////////////////////////////////////////////////////////////////////////////
    ///////                        Process IMU Data                        ///////
    //////////////////////////////////////////////////////////////////////////////
    double filtered_pitch_ = 0.0;
    double filtered_roll_  = 0.0;
    bool imu_initialized_ = false;

    // Teleop state variables
    double target_vx_ = 0.0;
    double target_wz_ = 0.0;
    double current_vx_ = 0.0;
    double current_wz_ = 0.0;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Extract quaternion
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        // Convert quaternion to pitch (Y-axis rotation)
        double sinp = 2.0 * (qw * qy - qz * qx);
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2.0, sinp); 
        else
            pitch = std::asin(sinp);
        
        // Convert quaternion to roll (X-axis rotation)
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        // Low-pass filter (EMA)
        double alpha = 0.1;  // 0.1 = smoother, 0.3 = more responsive

        if (!imu_initialized_) {
            // Initialize filter to first measurement
            filtered_pitch_ = pitch;
            filtered_roll_  = roll;
            imu_initialized_ = true;
        } else {
            filtered_pitch_ = alpha * pitch + (1.0 - alpha) * filtered_pitch_;
            filtered_roll_  = alpha * roll  + (1.0 - alpha) * filtered_roll_;
        }

        // Store filtered values as main state
        current_pitch_ = filtered_pitch_;
        current_roll_  = filtered_roll_;
    }

    //////////////////////////////////////////////////////////////////////////////

    void publish_trajectory() {

        //////////////////////////////////////////////////////////////////////////////
        ///////                     Define target positions                    ///////
        //////////////////////////////////////////////////////////////////////////////
        
        auto current_time_obj = this->now();
        double current_time = current_time_obj.seconds();

        if (first_run_) {
            if (current_time == 0.0) return;
            start_time_ = current_time;
            last_time_  = current_time;
            first_run_ = false;
            return;
        }

        double time_elapsed = current_time - start_time_;
        double time_wait = 2.0;

        // Slew Rate Limiter for smooth acceleration
        current_vx_ = 0.05 * target_vx_ + 0.95 * current_vx_;
        current_wz_ = 0.05 * target_wz_ + 0.95 * current_wz_;

        /////////// Differential Steering ///////////
        // Scale speed down
        double linear_scale  = 0.2;  
        double angular_scale = 0.15; // Scale down the rotation too so it doesn't spin wildly

        double scaled_vx = current_vx_ * linear_scale;
        double scaled_wz = current_wz_ * angular_scale;

        // Calculate left and right step lengths
        double L_left  = scaled_vx - scaled_wz;
        double L_right = scaled_vx + scaled_wz;

        // Cap speed at 0.15
        L_left  = std::clamp(L_left, -0.20, 0.20);
        L_right = std::clamp(L_right, -0.20, 0.20);

        // Dynamic step height to stop marching if not moving
        double speed_ratio = std::clamp(std::max(std::abs(current_vx_) / 0.1, std::abs(current_wz_) / 0.5), 0.0, 1.0);
        double h_eff = 0.05 * speed_ratio; // Max height of 5cm

        const double H_stand = 0.28;   
        const double T = 1.0;          
        const double W = 0.10;         

        double phase_A = 0.0;
        double phase_B = 0.0;

        if (time_elapsed > time_wait) {
            double walk_time = time_elapsed - time_wait;
            phase_A = fmod(walk_time / T, 1.0);
            phase_B = fmod((walk_time / T) + 0.5, 1.0); 
        }

        // Lambda to calculate foot position based on phase and dynamic Length/Height
        auto get_foot_pos = [&](double phase, double L, double h) -> Eigen::Vector3d {
            double x = (L / 2.0) * cos(2 * M_PI * phase);
            double z = -H_stand;
            if (phase < 0.5) { 
                z += h * sin(2 * M_PI * phase); 
            }
            return Eigen::Vector3d(x, 0.0, z); 
        };

        // Create 4 distinct leg trajectories instead of 2 pairs
        Eigen::Vector3d pos_FL_base = get_foot_pos(phase_A, L_left,  h_eff);
        Eigen::Vector3d pos_RR_base = get_foot_pos(phase_A, L_right, h_eff);
        Eigen::Vector3d pos_FR_base = get_foot_pos(phase_B, L_right, h_eff);
        Eigen::Vector3d pos_RL_base = get_foot_pos(phase_B, L_left,  h_eff);

        //////////////////////////////////////////////////////////////////////////////
        ///////                       Pitch PID Correction                     ///////
        //////////////////////////////////////////////////////////////////////////////

        double dt = current_time - last_time_;
        last_time_ = current_time;
        if (dt <= 0.0 || dt > 0.05) dt = 0.01;

        dt = std::min(dt, 0.02); 

        // Pitch PID
        double target_pitch = 0.0;
        double pitch_error = target_pitch - current_pitch_; 
        double Kp_pitch = 0.75; // Proportional Constant
        double Kd_pitch = 0.015; // Derivative Constant
        double Ki_pitch = 0.001; // Integral Constant
        double pitch_correction = 0.0;
        
        // Roll PID
        double target_roll = 0.0;
        double roll_error = target_roll - current_roll_;
        double Kp_roll = 0.5; 
        double Kd_roll = 0.005;
        double Ki_roll = 0.00;
        double roll_correction = 0.0;

        // Deadband
        if (std::abs(pitch_error) < 0.01)
            pitch_error = 0.0;

        if (std::abs(roll_error) < 0.01)
            roll_error = 0.0;

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
            pitch_correction = std::clamp(pitch_correction, -0.03, 0.03);

            // Calculate roll correction
            double roll_derivative = (roll_error - previous_roll_error_) / dt;
            integral_roll_error_ += (roll_error * dt);
            integral_roll_error_ = std::clamp(integral_roll_error_, -0.5, 0.5);
            roll_correction = 
                (Kp_roll * roll_error) + 
                (Ki_roll * integral_roll_error_) + 
                (Kd_roll * roll_derivative);
            roll_correction = std::clamp(roll_correction, -0.02, 0.02);

        } else {
            // Keep the integral error cleared out while dropping/standing
            integral_pitch_error_ = 0.0;
            integral_roll_error_ = 0.0;
        }

        previous_pitch_error_ = pitch_error;
        previous_roll_error_ = roll_error;

        // Calculate z correction
        double front_z_offset = pitch_correction;
        double rear_z_offset  = -pitch_correction;
        double left_z_offset  = -roll_correction;
        double right_z_offset = roll_correction;

        // We apply balance offsets to standing legs, let the swinging legs prioritize their ground clearance.
        auto apply_balance = [&](Eigen::Vector3d pos, double z_off, double phase) {
            double gain = (phase < 0.5) ? (phase * 2.0) : 1.0; 
            pos.z() += (z_off * gain);
            return pos;
        };

        // Apply balance directly to the 4 distinct trajectories
        Eigen::Vector3d pos_FL = apply_balance(pos_FL_base, front_z_offset + left_z_offset, phase_A);
        Eigen::Vector3d pos_RR = apply_balance(pos_RR_base, rear_z_offset  + right_z_offset, phase_A);
        Eigen::Vector3d pos_FR = apply_balance(pos_FR_base, front_z_offset + right_z_offset, phase_B);
        Eigen::Vector3d pos_RL = apply_balance(pos_RL_base, rear_z_offset  + left_z_offset, phase_B);

        pos_FL.y() = W;   
        pos_RR.y() = -W;
        pos_FR.y() = -W;  
        pos_RL.y() = W;

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
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    quadruped_locomotion::IKSolver solver_;
    
    double start_time_;
    bool first_run_;
    double last_time_;

    double current_pitch_;
    double previous_pitch_error_;
    double integral_pitch_error_;

    double current_roll_;
    double previous_roll_error_;
    double integral_roll_error_;
};

std::shared_ptr<BalanceNode> g_node = nullptr;

void sigint_handler(int sig) {
    (void)sig;
    if (g_node) {
        g_node->publish_standing_pose();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    rclcpp::shutdown();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    g_node = std::make_shared<BalanceNode>();
    signal(SIGINT, sigint_handler);
    rclcpp::spin(g_node);
    return 0;
}