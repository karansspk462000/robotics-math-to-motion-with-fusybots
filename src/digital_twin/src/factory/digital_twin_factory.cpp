#include "digital_twin/factory/digital_twin_factory.hpp"

#include <stdexcept>

#include "digital_twin/strategy/dummy_digital_twin.hpp"

namespace digital_twin {

std::unique_ptr<robot_hw_interface::IRobotInterface> createDigitalTwin(
    const std::string& method, int num_joints, const YAML::Node& params,
    const std::vector<double>& home_joint_position) {
    if (method == "dummy") {
        const double max_joint_speed = params["max_joint_speed"].as<double>(2.0);
        return std::make_unique<DummyDigitalTwin>(num_joints, max_joint_speed, home_joint_position);
    }
    if (method == "pybullet" || method == "mujoco" || method == "gazebo") {
        throw std::runtime_error("digital_twin_factory: method '" + method + "' is not implemented yet");
    }
    throw std::runtime_error("digital_twin_factory: unknown digital_twin_method '" + method + "'");
}

}  // namespace digital_twin
