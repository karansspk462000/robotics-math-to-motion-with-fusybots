#include "robot_hw_interface/strategy/dual_robot_interface.hpp"

#include <stdexcept>

namespace robot_hw_interface {

DualRobotInterface::DualRobotInterface(std::unique_ptr<IRobotInterface> primary,
                                        std::unique_ptr<IRobotInterface> secondary)
    : primary_(std::move(primary)), secondary_(std::move(secondary)) {
    if (!primary_ || !secondary_) {
        throw std::invalid_argument("DualRobotInterface: primary/secondary must not be null");
    }
    if (primary_->numJoints() != secondary_->numJoints()) {
        throw std::invalid_argument(
            "DualRobotInterface: primary/secondary backends disagree on joint count");
    }
}

void DualRobotInterface::write(const JointCommand& command) {
    primary_->write(command);
    secondary_->write(command);
}

JointState DualRobotInterface::read() {
    const JointState primary_state = primary_->read();
    last_secondary_state_ = secondary_->read();
    return primary_state;
}

int DualRobotInterface::numJoints() const {
    return primary_->numJoints();
}

}  // namespace robot_hw_interface
