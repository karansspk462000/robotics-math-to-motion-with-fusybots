#include "kinematics/context/kinematics.hpp"

#include <stdexcept>

namespace kinematics {

ForwardKinematics::ForwardKinematics(std::shared_ptr<IFKStrategy> strategy)
    : strategy_(std::move(strategy)) {
    if (!strategy_) {
        throw std::invalid_argument("ForwardKinematics: strategy must not be null");
    }
}

CartesianPosition ForwardKinematics::solve(const Eigen::VectorXd& joint_positions) const {
    return strategy_->computeCartesianPosition(joint_positions);
}

void ForwardKinematics::setStrategy(std::shared_ptr<IFKStrategy> strategy) {
    if (!strategy) {
        throw std::invalid_argument("ForwardKinematics: strategy must not be null");
    }
    strategy_ = std::move(strategy);
}

}  // namespace kinematics
