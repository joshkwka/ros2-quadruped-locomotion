#ifndef QUADRUPED_LOCOMOTION_CONSTANTS_HPP
#define QUADRUPED_LOCOMOTION_CONSTANTS_HPP

namespace quadruped_locomotion {
namespace constants {

    // Unitree Go1 Leg Link Lengths (in meters)
    constexpr double L1_HIP_OFFSET   = 0.080;   // Distance from hip motor to thigh joint
    constexpr double L2_THIGH_LENGTH = 0.213;   // Distance from thigh to knee joint
    constexpr double L3_CALF_LENGTH  = 0.213;   // Distance from knee to foot center

    // Unitree Go1 Body Dimensions (in meters)
    // Distance from the trunk to the hip motors
    constexpr double BODY_LENGTH_X   = 0.1881;  // Forward/Backward offset
    constexpr double BODY_WIDTH_Y    = 0.04675; // Left/Right offset

} // namespace constants
} // namespace quadruped_locomotion

#endif // QUADRUPED_LOCOMOTION_CONSTANTS_HPP