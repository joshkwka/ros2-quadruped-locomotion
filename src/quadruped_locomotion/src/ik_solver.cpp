#include "quadruped_locomotion/ik_solver.hpp"
#include <cmath>
#include <iostream>

namespace quadruped_locomotion {

IKSolver::IKSolver() {}

Eigen::Vector3d IKSolver::solveLegIK(const Eigen::Vector3d& target_xyz, bool is_left_leg) {
    // We will extract the target coordinates
    double x = target_xyz.x();
    double y = target_xyz.y();
    double z = target_xyz.z();

    // Determine the correct hip offset sign based on the leg side
    double l1 = constants::L1_HIP_OFFSET;
    double l2 = constants::L2_THIGH_LENGTH;
    double l3 = constants::L3_CALF_LENGTH;
    
    int side_sign = is_left_leg ? 1 : -1;

    // Calculate joint angles
    // 1. Calculate theta hip (Looking at the leg from the front view)
    double L_yz = std::sqrt(y*y + z*z);
    double z_plane = std::sqrt(L_yz*L_yz - l1*l1); // The downward depth

    double theta_hip = std::atan2(y, -z) - std::atan2(l1 * side_sign, z_plane);

    // 2. Calculate calf angle
    // Calculate leg length (thigh joint to calf)
    double L_leg_squared = x*x + z_plane*z_plane;
    double L_leg = std::sqrt(L_leg_squared);

    // Law of cosines
    double cos_calf = (L_leg_squared - l2*l2 - l3*l3) / (2.0 * l2 * l3);
    // Clamp result to prevent NaN crashes if the target is out of range
    cos_calf = std::max(-1.0, std::min(1.0, cos_calf)); 
    // Go1 knees bend backwards, so we invert the acos result
    double theta_calf = -std::acos(cos_calf);
    
    // 3. Calculate thigh angle
    // Angle straight down to the foot
    double alpha = std::atan2(x, z_plane);
    
    // Inner triangle angle between L_leg and L2
    double cos_thigh_offset = (l2*l2 + L_leg_squared - l3*l3) / (2.0 * l2 * L_leg);
    cos_thigh_offset = std::max(-1.0, std::min(1.0, cos_thigh_offset));
    double beta = std::acos(cos_thigh_offset);

    // Pitch the thigh forward to meet the target
    double theta_thigh = alpha + beta;

    return Eigen::Vector3d(theta_hip, theta_thigh, theta_calf);
}

} // namespace quadruped_locomotion