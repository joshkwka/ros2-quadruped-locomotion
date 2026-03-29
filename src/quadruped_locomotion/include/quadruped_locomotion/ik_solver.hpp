#ifndef QUADRUPED_LOCOMOTION_IK_SOLVER_HPP
#define QUADRUPED_LOCOMOTION_IK_SOLVER_HPP

#include <Eigen/Dense>
#include "quadruped_locomotion/constants.hpp"

namespace quadruped_locomotion {

class IKSolver {
public:
    IKSolver();
    ~IKSolver() = default;

    /**
     * @brief Solves the Inverse Kinematics for a single Go1 leg.
     * @param target_xyz The desired (X, Y, Z) foot position in the LOCAL hip frame.
     * @param is_left_leg True if calculating for FL or RL, False for FR or RR.
     * @return Eigen::Vector3d containing [hip_angle, thigh_angle, calf_angle] in radians.
     */
    Eigen::Vector3d solveLegIK(const Eigen::Vector3d& target_xyz, bool is_left_leg);

private:
    // Helper functions for the trig math
};

} // namespace quadruped_locomotion

#endif // QUADRUPED_LOCOMOTION_IK_SOLVER_HPP