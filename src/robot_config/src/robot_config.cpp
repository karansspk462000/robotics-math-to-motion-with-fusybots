#include "robot_config/robot_config.hpp"

#include <filesystem>
#include <stdexcept>

namespace robot_config {

namespace {

ControlMode parseControlMode(const std::string& mode) {
    if (mode == "position") return ControlMode::kPosition;
    if (mode == "velocity") return ControlMode::kVelocity;
    if (mode == "torque") return ControlMode::kTorque;
    throw std::runtime_error("robot_config: unknown control_mode '" + mode + "'");
}

BackendMode parseBackendMode(const std::string& mode) {
    if (mode == "hardware") return BackendMode::kHardwareOnly;
    if (mode == "digital_twin") return BackendMode::kDigitalTwinOnly;
    if (mode == "both") return BackendMode::kBoth;
    throw std::runtime_error("robot_config: unknown backend_mode '" + mode + "'");
}

RobotDescription loadRobotDescription(const std::string& robot_description_path) {
    const YAML::Node root = YAML::LoadFile(robot_description_path);

    RobotDescription robot;
    robot.name = root["name"].as<std::string>();
    robot.num_joints = root["num_joints"].as<int>();
    robot.dof = root["dof"].as<int>();

    const YAML::Node joints = root["joints"];
    int joint_index = 0;
    for (const auto& joint : joints) {
        JointLimits limits;
        limits.name = joint["name"].as<std::string>("j" + std::to_string(++joint_index));
        limits.position_min = joint["position_min"].as<double>();
        limits.position_max = joint["position_max"].as<double>();
        limits.velocity_max = joint["velocity_max"].as<double>();
        limits.effort_max = joint["effort_max"].as<double>();
        robot.joint_limits.push_back(limits);

        PidGains pid;
        if (const YAML::Node pid_node = joint["pid"]; pid_node) {
            pid.p = pid_node["p"].as<double>(0.0);
            pid.i = pid_node["i"].as<double>(0.0);
            pid.d = pid_node["d"].as<double>(0.0);
        }
        robot.joint_pid_gains.push_back(pid);
    }

    if (static_cast<int>(robot.joint_limits.size()) != robot.num_joints) {
        throw std::runtime_error("robot_config: '" + robot_description_path +
                                  "' declares num_joints=" + std::to_string(robot.num_joints) +
                                  " but lists " + std::to_string(robot.joint_limits.size()) + " joints");
    }

    if (const YAML::Node home_node = root["home_joint_position"]; home_node) {
        robot.home_joint_position = home_node.as<std::vector<double>>();
        if (static_cast<int>(robot.home_joint_position.size()) != robot.num_joints) {
            throw std::runtime_error("robot_config: '" + robot_description_path + "' declares num_joints=" +
                                      std::to_string(robot.num_joints) + " but home_joint_position has " +
                                      std::to_string(robot.home_joint_position.size()) + " values");
        }
    } else {
        robot.home_joint_position.assign(robot.num_joints, 0.0);
    }

    robot.kinematics_params = root["kinematics"];
    return robot;
}

TopicNames parseTopicNames(const YAML::Node& root) {
    TopicNames topics;
    if (const YAML::Node topics_node = root["topics"]; topics_node) {
        topics.hw_status = topics_node["hw_status"].as<std::string>(topics.hw_status);
        topics.user_command = topics_node["user_command"].as<std::string>(topics.user_command);
        topics.hw_command = topics_node["hw_command"].as<std::string>(topics.hw_command);
        topics.digital_twin_status =
            topics_node["digital_twin_status"].as<std::string>(topics.digital_twin_status);
        topics.motion_status = topics_node["motion_status"].as<std::string>(topics.motion_status);
    }
    return topics;
}

}  // namespace

SystemConfig loadSystemConfig(const std::string& system_config_path) {
    const YAML::Node root = YAML::LoadFile(system_config_path);

    SystemConfig config;
    config.robot_description_file = root["robot_description_file"].as<std::string>();
    config.kinematics_method = root["kinematics_method"].as<std::string>();
    config.ik_method = root["ik_method"].as<std::string>();
    config.path_planner_method = root["path_planner_method"].as<std::string>();
    config.path_interpolator_method = root["path_interpolator_method"].as<std::string>();
    config.cartesian_path_interpolator_method =
        root["cartesian_path_interpolator_method"].as<std::string>("linear_cartesian");
    config.path_interpolator_points_per_segment =
        root["path_interpolator_points_per_segment"].as<int>(config.path_interpolator_points_per_segment);
    config.cartesian_path_interpolator_points_per_segment =
        root["cartesian_path_interpolator_points_per_segment"].as<int>(
            config.cartesian_path_interpolator_points_per_segment);
    config.trajectory_planner_method = root["trajectory_planner_method"].as<std::string>();
    config.control_mode = parseControlMode(root["control_mode"].as<std::string>());
    config.topics = parseTopicNames(root);

    if (const YAML::Node backend_node = root["backend"]; backend_node) {
        config.backend_mode = parseBackendMode(backend_node["mode"].as<std::string>("digital_twin"));
        config.hardware_method = backend_node["hardware_method"].as<std::string>("dummy");
        config.hardware_params = backend_node["hardware_params"];
        config.digital_twin_method = backend_node["digital_twin_method"].as<std::string>("dummy");
        config.digital_twin_params = backend_node["digital_twin_params"];
    }

    if (const YAML::Node timing_node = root["timing"]; timing_node) {
        config.hw_status_publish_rate_hz =
            timing_node["hw_status_publish_rate_hz"].as<double>(config.hw_status_publish_rate_hz);
        config.trajectory_playback_rate_hz =
            timing_node["trajectory_playback_rate_hz"].as<double>(config.trajectory_playback_rate_hz);
        config.state_machine_poll_rate_hz =
            timing_node["state_machine_poll_rate_hz"].as<double>(config.state_machine_poll_rate_hz);
    }

    if (const YAML::Node state_machine_node = root["state_machine"]; state_machine_node) {
        config.hw_status_timeout_sec =
            state_machine_node["hw_status_timeout_sec"].as<double>(config.hw_status_timeout_sec);
        config.divergence_warn_threshold =
            state_machine_node["divergence_warn_threshold"].as<double>(config.divergence_warn_threshold);
    }

    std::filesystem::path robot_file(config.robot_description_file);
    if (robot_file.is_relative()) {
        robot_file = std::filesystem::path(system_config_path).parent_path() / robot_file;
    }
    config.robot = loadRobotDescription(robot_file.string());

    return config;
}

}  // namespace robot_config
