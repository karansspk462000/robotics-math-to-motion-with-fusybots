#pragma once

// Shared kinematics_params builders for the robot used across test files -
// fusybots_six_dof_spherical_wrist's own KDL/analytical params.
#include <yaml-cpp/yaml.h>

namespace test_helpers {

inline YAML::Node kdlParams() {
    YAML::Node params;
    params["urdf_path"] = "package://robot_description/urdf/fusybots_six_dof_spherical_wrist.urdf.xacro";
    params["start_link"] = "base_link";
    params["end_link"] = "ee_link";
    return params;
}

inline YAML::Node analyticalParams() {
    YAML::Node params;
    params["link_lengths"]["l1"] = 0.2;
    params["link_lengths"]["d2"] = 0.01;
    params["link_lengths"]["l3"] = 0.3;
    params["link_lengths"]["l5"] = 0.32;
    params["link_lengths"]["l6"] = 0.25;
    params["link_lengths"]["l7"] = 0.0;  // spherical wrist - see fusybots_six_dof_spherical_wrist.yaml
    params["link_lengths"]["le"] = 0.10;
    return params;
}

}  // namespace test_helpers
