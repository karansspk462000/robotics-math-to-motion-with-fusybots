#pragma once

#include <string>

#include <Eigen/Dense>

namespace robot_hw_interface {

// Command sent to a robot backend (real hardware or a digital twin).
// control_mode selects which of position/velocity/torque the backend
// should actually track; the others may be left empty. Mirrors
// custom_robot_msgs::msg::HwCommand so the ROS boundary node's conversion
// is a direct field-by-field copy.
struct JointCommand {
    std::string control_mode;  // "position" | "velocity" | "torque"
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd torque;
};

// Status read back from a robot backend. Mirrors
// custom_robot_msgs::msg::HwStatus plus velocity/torque, since a backend
// (real or simulated) generally has more to report than just position.
struct JointState {
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd torque;
    Eigen::VectorXd current;
};

}  // namespace robot_hw_interface
