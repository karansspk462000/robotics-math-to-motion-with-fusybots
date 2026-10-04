#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_config/robot_config.hpp"
#include "state_orchestrator/visualization/joint_state_bridge_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const std::string robot_config_share_dir =
        ament_index_cpp::get_package_share_directory("robot_config");
    const std::string system_config_path = robot_config_share_dir + "/config/system_config.yaml";
    const robot_config::SystemConfig config = robot_config::loadSystemConfig(system_config_path);

    std::vector<std::string> joint_names;
    joint_names.reserve(config.robot.joint_limits.size());
    for (const auto& joint : config.robot.joint_limits) {
        joint_names.push_back(joint.name);
    }

    auto node = std::make_shared<state_orchestrator::JointStateBridgeNode>(config.topics.hw_status,
                                                                            joint_names);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
