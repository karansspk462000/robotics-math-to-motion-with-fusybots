#include "state_orchestrator/state_manager/robot_state_machine.hpp"

#include "state_orchestrator/state_manager/states/robot_states.hpp"

namespace state_orchestrator {

RobotStateMachine::RobotStateMachine(robot_config::BackendMode backend_mode,
                                      double hw_status_timeout_sec, double divergence_warn_threshold)
    : backend_mode_(backend_mode),
      hw_status_timeout_sec_(hw_status_timeout_sec),
      divergence_warn_threshold_(divergence_warn_threshold),
      current_state_(&StartingRobotState::instance()) {}

void RobotStateMachine::onSoftwareUp() { current_state_->onSoftwareUp(*this); }

void RobotStateMachine::onHwStatusReceived(double now_sec, const Eigen::VectorXd& position) {
    last_hw_status_time_sec_ = now_sec;
    latched_position_ = position;
    current_state_->onHwStatusReceived(*this, now_sec, position);
}

void RobotStateMachine::onDigitalTwinStatusReceived(const Eigen::VectorXd& position) {
    if (latched_position_.size() == position.size() && latched_position_.size() > 0) {
        last_divergence_ = (latched_position_ - position).cwiseAbs().maxCoeff();
        divergence_warning_active_ = last_divergence_ > divergence_warn_threshold_;
    }
}

void RobotStateMachine::checkTimeout(double now_sec) { current_state_->checkTimeout(*this, now_sec); }

bool RobotStateMachine::consumeStateChanged() {
    const bool changed = state_changed_;
    state_changed_ = false;
    return changed;
}

void RobotStateMachine::transitionTo(const IRobotState& new_state) {
    if (&new_state != current_state_) {
        current_state_ = &new_state;
        state_changed_ = true;
    }
}

const IRobotState& RobotStateMachine::readyStateForBackendMode() const {
    switch (backend_mode_) {
        case robot_config::BackendMode::kHardwareOnly: return HardwareReadyState::instance();
        case robot_config::BackendMode::kDigitalTwinOnly: return DigitalTwinReadyState::instance();
        case robot_config::BackendMode::kBoth: return HardwareAndDigitalTwinReadyState::instance();
    }
    return HardwareReadyState::instance();
}

bool RobotStateMachine::hwStatusTimedOut(double now_sec) const {
    return (now_sec - last_hw_status_time_sec_) > hw_status_timeout_sec_;
}

}  // namespace state_orchestrator
