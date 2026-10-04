#pragma once

#include <vector>

#include "custom_robot_msgs/msg/hw_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace state_orchestrator {

// Republishes topics.hw_status (our custom_robot_msgs::msg::HwStatus - the
// robot's actual achieved state, not its commanded target) as a standard
// sensor_msgs/JointState on /joint_states, which is the fixed topic name
// robot_state_publisher listens to for computing TF. This is the only
// piece needed to make RViz reflect live motion from the real pipeline
// (hw_interface_node -> hw_status) without renaming any of our own
// topics or teaching robot_state_publisher about custom_robot_msgs.
class JointStateBridgeNode : public rclcpp::Node {
public:
    JointStateBridgeNode(const std::string& hw_status_topic, std::vector<std::string> joint_names);

private:
    void hwStatusCallback(const custom_robot_msgs::msg::HwStatus::SharedPtr msg);

    std::vector<std::string> joint_names_;
    rclcpp::Subscription<custom_robot_msgs::msg::HwStatus>::SharedPtr hw_status_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

}  // namespace state_orchestrator
