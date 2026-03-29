#include "rclcpp/rclcpp.hpp"
#include "quadruped_locomotion/ik_solver.hpp"

class IKNode : public rclcpp::Node {
public:
    IKNode() : Node("ik_node") {
        RCLCPP_INFO(this->get_logger(), "Locomotion Brain Initialized.");
        // Future: Initialize subscribers for commands and publishers for joint states
    }

private:
    quadruped_locomotion::IKSolver solver_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKNode>());
    rclcpp::shutdown();
    return 0;
}