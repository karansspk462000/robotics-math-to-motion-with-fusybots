#pragma once

#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "robot_hw_interface/interface/robot_interface.hpp"

namespace hw_interface {

// Builds the real-hardware IRobotInterface named by `method` ("dummy" or
// "arduino"). `params` is the method-specific YAML block (e.g. "arduino"
// expects serial_port/baud_rate). `home_joint_position` seeds "dummy"'s
// reported state before any command arrives - unused by "arduino", which
// reads the real actuators' actual position instead.
std::unique_ptr<robot_hw_interface::IRobotInterface> createHardwareInterface(
    const std::string& method, int num_joints, const YAML::Node& params,
    const std::vector<double>& home_joint_position);

}  // namespace hw_interface
