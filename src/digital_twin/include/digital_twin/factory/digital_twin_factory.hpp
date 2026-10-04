#pragma once

#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "robot_hw_interface/interface/robot_interface.hpp"

namespace digital_twin {

// Builds the digital twin IRobotInterface named by `method`. "dummy" is
// implemented; "pybullet"/"mujoco"/"gazebo" are recognized names for
// whichever physics engine gets picked later - the interface (this
// factory + IRobotInterface) stays the same regardless of which one it
// ends up being, only this function's body needs a new branch.
// `home_joint_position` seeds "dummy"'s reported state before any command
// arrives.
std::unique_ptr<robot_hw_interface::IRobotInterface> createDigitalTwin(
    const std::string& method, int num_joints, const YAML::Node& params,
    const std::vector<double>& home_joint_position);

}  // namespace digital_twin
