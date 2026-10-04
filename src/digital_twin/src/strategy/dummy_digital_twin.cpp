#include "digital_twin/strategy/dummy_digital_twin.hpp"

#include <algorithm>
#include <cmath>

namespace digital_twin {

using robot_hw_interface::JointCommand;
using robot_hw_interface::JointState;

DummyDigitalTwin::DummyDigitalTwin(int num_joints, double max_joint_speed,
                                    const std::vector<double>& home_joint_position)
    : num_joints_(num_joints), max_joint_speed_(max_joint_speed) {
    Eigen::VectorXd home = Eigen::VectorXd::Zero(num_joints_);
    if (static_cast<int>(home_joint_position.size()) == num_joints_) {
        home = Eigen::Map<const Eigen::VectorXd>(home_joint_position.data(), num_joints_);
    }

    target_command_.control_mode = "position";
    target_command_.position = home;
    target_command_.velocity = Eigen::VectorXd::Zero(num_joints_);
    target_command_.torque = Eigen::VectorXd::Zero(num_joints_);

    state_.position = home;
    state_.velocity = Eigen::VectorXd::Zero(num_joints_);
    state_.torque = Eigen::VectorXd::Zero(num_joints_);
    state_.current = Eigen::VectorXd::Zero(num_joints_);

    last_read_time_ = std::chrono::steady_clock::now();
}

void DummyDigitalTwin::write(const JointCommand& command) {
    target_command_ = command;
}

JointState DummyDigitalTwin::read() {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_read_time_).count();
    last_read_time_ = now;

    if (target_command_.control_mode == "velocity" &&
        target_command_.velocity.size() == num_joints_) {
        state_.velocity = target_command_.velocity;
        state_.position += state_.velocity * dt;
    } else if (target_command_.control_mode == "position" &&
               target_command_.position.size() == num_joints_) {
        const Eigen::VectorXd error = target_command_.position - state_.position;
        const double max_step = max_joint_speed_ * dt;
        for (int i = 0; i < num_joints_; ++i) {
            const double step = std::clamp(error[i], -max_step, max_step);
            state_.position[i] += step;
            state_.velocity[i] = dt > 0.0 ? step / dt : 0.0;
        }
    } else if (target_command_.control_mode == "torque" &&
               target_command_.torque.size() == num_joints_) {
        state_.torque = target_command_.torque;
    }

    return state_;
}

int DummyDigitalTwin::numJoints() const {
    return num_joints_;
}

}  // namespace digital_twin
