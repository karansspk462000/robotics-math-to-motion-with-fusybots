#include "state_orchestrator/visualization/joint_state_bridge_node.hpp"

#include <stdexcept>

namespace state_orchestrator {

JointStateBridgeNode::JointStateBridgeNode(const std::string& hw_status_topic,
                                            std::vector<std::string> joint_names)
    : rclcpp::Node("joint_state_bridge_node"), joint_names_(std::move(joint_names)) {
    const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    hw_status_sub_ = create_subscription<custom_robot_msgs::msg::HwStatus>(
        hw_status_topic, reliable_qos,
        std::bind(&JointStateBridgeNode::hwStatusCallback, this, std::placeholders::_1));

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", reliable_qos);
}

void JointStateBridgeNode::hwStatusCallback(const custom_robot_msgs::msg::HwStatus::SharedPtr msg) {
    if (msg->joint_positions.size() != joint_names_.size()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                               "hw_status has %zu joints but robot config declares %zu joint names - "
                               "check fusybots_six_dof.yaml's 'joints' list matches num_joints",
                               msg->joint_positions.size(), joint_names_.size());
        return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name = joint_names_;
    joint_state.position.assign(msg->joint_positions.begin(), msg->joint_positions.end());
    if (msg->joint_velocity.size() == joint_names_.size()) {
        joint_state.velocity.assign(msg->joint_velocity.begin(), msg->joint_velocity.end());
    }
    if (msg->joint_torque.size() == joint_names_.size()) {
        joint_state.effort.assign(msg->joint_torque.begin(), msg->joint_torque.end());
    }

    joint_state_pub_->publish(joint_state);
}

}  // namespace state_orchestrator
