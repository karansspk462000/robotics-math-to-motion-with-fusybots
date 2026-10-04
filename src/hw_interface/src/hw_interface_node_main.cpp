#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "digital_twin/factory/digital_twin_factory.hpp"
#include "hw_interface/factory/hw_interface_factory.hpp"
#include "hw_interface/hw_interface_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_config/robot_config.hpp"
#include "robot_hw_interface/strategy/dual_robot_interface.hpp"

namespace {

std::unique_ptr<robot_hw_interface::IRobotInterface> buildRobotInterface(
    const robot_config::SystemConfig& config) {
    switch (config.backend_mode) {
        case robot_config::BackendMode::kHardwareOnly:
            return hw_interface::createHardwareInterface(config.hardware_method, config.robot.num_joints,
                                                           config.hardware_params,
                                                           config.robot.home_joint_position);
        case robot_config::BackendMode::kDigitalTwinOnly:
            return digital_twin::createDigitalTwin(config.digital_twin_method, config.robot.num_joints,
                                                     config.digital_twin_params,
                                                     config.robot.home_joint_position);
        case robot_config::BackendMode::kBoth:
            // Same command reaches both (DualRobotInterface::write), so
            // hardware and its digital twin can never receive different
            // commands. DualRobotInterface::read() always reports
            // hardware (primary) as ground truth - never the twin.
            return std::make_unique<robot_hw_interface::DualRobotInterface>(
                hw_interface::createHardwareInterface(config.hardware_method, config.robot.num_joints,
                                                        config.hardware_params,
                                                        config.robot.home_joint_position),
                digital_twin::createDigitalTwin(config.digital_twin_method, config.robot.num_joints,
                                                 config.digital_twin_params,
                                                 config.robot.home_joint_position));
    }
    throw std::logic_error("hw_interface_node_main: unhandled BackendMode");
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const std::string robot_config_share_dir =
        ament_index_cpp::get_package_share_directory("robot_config");
    const std::string system_config_path = robot_config_share_dir + "/config/system_config.yaml";
    const robot_config::SystemConfig config = robot_config::loadSystemConfig(system_config_path);

    auto robot_interface = buildRobotInterface(config);
    auto node = std::make_shared<hw_interface::HwInterfaceNode>(std::move(robot_interface), config.topics,
                                                                  config.hw_status_publish_rate_hz);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
