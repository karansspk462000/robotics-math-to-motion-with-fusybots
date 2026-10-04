#include "hw_interface/strategy/dummy_hardware_interface.hpp"

namespace hw_interface {

using robot_hw_interface::JointCommand;
using robot_hw_interface::JointState;

DummyHardwareInterface::DummyHardwareInterface(int num_joints, const std::vector<double>& home_joint_position)
    : num_joints_(num_joints) {
    state_.position = Eigen::VectorXd::Zero(num_joints_);
    if (static_cast<int>(home_joint_position.size()) == num_joints_) {
        state_.position = Eigen::Map<const Eigen::VectorXd>(home_joint_position.data(), num_joints_);
    }
    state_.velocity = Eigen::VectorXd::Zero(num_joints_);
    state_.torque = Eigen::VectorXd::Zero(num_joints_);
    state_.current = Eigen::VectorXd::Zero(num_joints_);
}

void DummyHardwareInterface::write(const JointCommand& command) {
    if (command.control_mode == "position" && command.position.size() == num_joints_) {
        state_.position = command.position;
    } else if (command.control_mode == "velocity" && command.velocity.size() == num_joints_) {
        state_.velocity = command.velocity;
    } else if (command.control_mode == "torque" && command.torque.size() == num_joints_) {
        state_.torque = command.torque;
    }
}

JointState DummyHardwareInterface::read() {
    return state_;
}

int DummyHardwareInterface::numJoints() const {
    return num_joints_;
}

}  // namespace hw_interface
