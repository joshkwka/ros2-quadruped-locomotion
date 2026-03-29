#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include "quadruped_locomotion/ik_solver.hpp"

double rad2deg(double rad) { return rad * (180.0 / M_PI); }

int main() {
    quadruped_locomotion::IKSolver solver;
    
    // Test Coordinate: Straight down 30cm
    Eigen::Vector3d target(0.0, 0.0, -0.30);
    
    // Test Right Leg
    Eigen::Vector3d angles = solver.solveLegIK(target, false);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "--- IK STANDALONE TEST (RIGHT LEG) ---" << std::endl;
    std::cout << "Target: [" << target.x() << ", " << target.y() << ", " << target.z() << "]" << std::endl;
    std::cout << "Hip Roll:    " << rad2deg(angles[0]) << " deg" << std::endl;
    std::cout << "Thigh Pitch: " << rad2deg(angles[1]) << " deg" << std::endl;
    std::cout << "Calf Pitch:  " << rad2deg(angles[2]) << " deg" << std::endl;
    
    return 0;
}