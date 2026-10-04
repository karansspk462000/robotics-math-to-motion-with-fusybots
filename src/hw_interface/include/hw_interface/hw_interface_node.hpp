#pragma once

#include <memory>
#include <mutex>

#include "custom_robot_msgs/msg/hw_command.hpp"
#include "custom_robot_msgs/msg/hw_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_config/robot_config.hpp"
#include "robot_hw_interface/strategy/dual_robot_interface.hpp"
#include "robot_hw_interface/interface/robot_interface.hpp"

namespace hw_interface {

// ROS boundary for IRobotInterface: subscribes topics.hw_command, calls
// write() on whichever backend(s) config selected (real hardware, digital
// twin, or both via DualRobotInterface - constructed by main(), this class
// doesn't know or care which), and publishes topics.hw_status from read()
// on a timer. Uses the exact same topic names robot_node reads
// from/writes to, since both load them from the same system_config.yaml -
// swapping backend or run mode never touches those topics.
//
// If robot_interface is a DualRobotInterface (backend.mode "both"),
// topics.hw_status always carries hardware's reading (DualRobotInterface's
// invariant) and topics.digital_twin_status additionally carries the
// twin's reading, so a downstream state machine can compare the two and
// warn on divergence without this node doing any comparison itself.
class HwInterfaceNode : public rclcpp::Node {
public:
    HwInterfaceNode(std::unique_ptr<robot_hw_interface::IRobotInterface> robot_interface,
                     robot_config::TopicNames topics, double status_publish_rate_hz = 100.0);

private:
    void hwCommandCallback(const custom_robot_msgs::msg::HwCommand::SharedPtr msg);
    void statusTimerCallback();

    std::unique_ptr<robot_hw_interface::IRobotInterface> robot_interface_;
    // Non-owning alias of robot_interface_ when it's a DualRobotInterface,
    // null otherwise; lets statusTimerCallback also publish the twin's
    // reading without robot_interface_'s static type being IRobotInterface.
    robot_hw_interface::DualRobotInterface* dual_interface_ = nullptr;
    std::mutex interface_mutex_;

    rclcpp::Subscription<custom_robot_msgs::msg::HwCommand>::SharedPtr hw_command_sub_;
    rclcpp::Publisher<custom_robot_msgs::msg::HwStatus>::SharedPtr hw_status_pub_;
    rclcpp::Publisher<custom_robot_msgs::msg::HwStatus>::SharedPtr digital_twin_status_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace hw_interface
