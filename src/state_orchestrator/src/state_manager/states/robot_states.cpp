#include "state_orchestrator/state_manager/states/robot_states.hpp"

#include "state_orchestrator/state_manager/robot_state_machine.hpp"

namespace state_orchestrator {

const StartingRobotState& StartingRobotState::instance() {
    static const StartingRobotState s;
    return s;
}

void StartingRobotState::onSoftwareUp(RobotStateMachine& machine) const {
    machine.transitionTo(SoftwareUpState::instance());
}

const SoftwareUpState& SoftwareUpState::instance() {
    static const SoftwareUpState s;
    return s;
}

void SoftwareUpState::onHwStatusReceived(RobotStateMachine& machine, double /*now_sec*/,
                                          const Eigen::VectorXd& /*position*/) const {
    machine.transitionTo(machine.readyStateForBackendMode());
}

void ReadyStateBase::checkTimeout(RobotStateMachine& machine, double now_sec) const {
    if (machine.hwStatusTimedOut(now_sec)) {
        machine.transitionTo(HardwareLinkLostState::instance());
    }
}

const HardwareReadyState& HardwareReadyState::instance() {
    static const HardwareReadyState s;
    return s;
}

const DigitalTwinReadyState& DigitalTwinReadyState::instance() {
    static const DigitalTwinReadyState s;
    return s;
}

const HardwareAndDigitalTwinReadyState& HardwareAndDigitalTwinReadyState::instance() {
    static const HardwareAndDigitalTwinReadyState s;
    return s;
}

const HardwareLinkLostState& HardwareLinkLostState::instance() {
    static const HardwareLinkLostState s;
    return s;
}

void HardwareLinkLostState::onHwStatusReceived(RobotStateMachine& machine, double /*now_sec*/,
                                                const Eigen::VectorXd& /*position*/) const {
    // Recovery: the next hw_status re-latches (done unconditionally by
    // RobotStateMachine before this dispatch) and returns to ready.
    machine.transitionTo(machine.readyStateForBackendMode());
}

}  // namespace state_orchestrator
