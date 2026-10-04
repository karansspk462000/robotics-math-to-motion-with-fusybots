#pragma once

#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "kinematics/interface/kinematics_interface.hpp"

namespace kinematics {

// Builds the IFKStrategy named by `method` ("analytical" or "kdl"; "poe"/
// "pinocchio" are recognized names not yet implemented) using
// method-specific parameters from `kinematics_params` (the raw YAML node
// robot_config leaves uninterpreted, since only the chosen method knows its
// own schema - e.g. for "analytical" it expects a `link_lengths` map).
std::shared_ptr<IFKStrategy> createFKStrategy(const std::string& method,
                                               const YAML::Node& kinematics_params);

// Builds the IIKStrategy named by `method` ("analytical" or "kdl"),
// wired to the given FK strategy - each concrete strategy dynamic_casts
// it to the concrete FK type it actually needs (they're method-specific
// pairs, not interchangeable).
std::unique_ptr<IIKStrategy> createIKStrategy(const std::string& method,
                                               std::shared_ptr<IFKStrategy> fk_strategy);

}  // namespace kinematics
