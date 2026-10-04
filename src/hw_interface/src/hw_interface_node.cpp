#include "hw_interface/hw_interface_node.hpp"

#include "math_utils/eigen/eigen_conversions.hpp"

namespace hw_interface {

HwInterfaceNode::HwInterfaceNode(std::unique_ptr<robot_hw_interface::IRobotInterface> robot_interface,
                                  robot_config::TopicNames topics, double status_publish_rate_hz)
    : rclcpp::Node("hw_interface_node"), robot_interface_(std::move(robot_interface)) {
    dual_interface_ = dynamic_cast<robot_hw_interface::DualRobotInterface*>(robot_interface_.get());

    const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    hw_command_sub_ = create_subscription<custom_robot_msgs::msg::HwCommand>(
        topics.hw_command, reliable_qos,
        std::bind(&HwInterfaceNode::hwCommandCallback, this, std::placeholders::_1));

    hw_status_pub_ = create_publisher<custom_robot_msgs::msg::HwStatus>(topics.hw_status, reliable_qos);
    if (dual_interface_ != nullptr) {
        digital_twin_status_pub_ =
            create_publisher<custom_robot_msgs::msg::HwStatus>(topics.digital_twin_status, reliable_qos);
    }

    const auto period = std::chrono::duration<double>(1.0 / status_publish_rate_hz);
    status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&HwInterfaceNode::statusTimerCallback, this));
}

void HwInterfaceNode::hwCommandCallback(const custom_robot_msgs::msg::HwCommand::SharedPtr msg) {
    robot_hw_interface::JointCommand command;
    command.control_mode = msg->control_mode;
    command.position = math_utils::toEigenVector(msg->joint_position);
    command.velocity = math_utils::toEigenVector(msg->joint_velocity);
    command.torque = math_utils::toEigenVector(msg->joint_torque);

    std::lock_guard<std::mutex> lock(interface_mutex_);
    robot_interface_->write(command);
}

void HwInterfaceNode::statusTimerCallback() {
    robot_hw_interface::JointState state;
    robot_hw_interface::JointState twin_state;
    {
        std::lock_guard<std::mutex> lock(interface_mutex_);
        state = robot_interface_->read();
        if (dual_interface_ != nullptr) {
            twin_state = dual_interface_->lastSecondaryState();
        }
    }

    custom_robot_msgs::msg::HwStatus status;
    status.joint_positions = math_utils::toStdVector<float>(state.position);
    status.joint_velocity = math_utils::toStdVector<float>(state.velocity);
    status.joint_torque = math_utils::toStdVector<float>(state.torque);
    status.joint_current = math_utils::toStdVector<float>(state.current);
    hw_status_pub_->publish(status);

    if (dual_interface_ != nullptr) {
        custom_robot_msgs::msg::HwStatus twin_status;
        twin_status.joint_positions = math_utils::toStdVector<float>(twin_state.position);
        twin_status.joint_velocity = math_utils::toStdVector<float>(twin_state.velocity);
        twin_status.joint_torque = math_utils::toStdVector<float>(twin_state.torque);
        twin_status.joint_current = math_utils::toStdVector<float>(twin_state.current);
        digital_twin_status_pub_->publish(twin_status);
    }
}

}  // namespace hw_interface
