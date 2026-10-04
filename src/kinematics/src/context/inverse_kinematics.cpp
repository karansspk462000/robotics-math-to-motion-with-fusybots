#include "kinematics/context/kinematics.hpp"

#include <stdexcept>

namespace kinematics {

InverseKinematics::InverseKinematics(std::unique_ptr<IIKStrategy> strategy)
    : strategy_(std::move(strategy)) {
    if (!strategy_) {
        throw std::invalid_argument("InverseKinematics: strategy must not be null");
    }
}

Eigen::VectorXd InverseKinematics::solve(const CartesianPosition& target_pose,
                                          const Eigen::VectorXd& initial_guess) const {
    return strategy_->computeJointPositions(target_pose, initial_guess);
}

Eigen::MatrixXd InverseKinematics::computeJacobian(const Eigen::VectorXd& joint_positions) const {
    return strategy_->computeJacobian(joint_positions);
}

void InverseKinematics::setStrategy(std::unique_ptr<IIKStrategy> strategy) {
    if (!strategy) {
        throw std::invalid_argument("InverseKinematics: strategy must not be null");
    }
    strategy_ = std::move(strategy);
}

}  // namespace kinematics
