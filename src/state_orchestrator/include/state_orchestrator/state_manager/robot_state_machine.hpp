#pragma once

#include <Eigen/Dense>

#include "robot_config/robot_config.hpp"
#include "state_orchestrator/state_manager/interface/robot_state_interface.hpp"
#include "state_orchestrator/state_manager/robot_state.hpp"

namespace state_orchestrator {

// GoF State pattern Context: tracks the robot's overall readiness and
// enforces two safety rules end-to-end, regardless of backend_mode:
//   1. hw_status is always assumed to be hardware's reading (that
//      invariant lives in robot_hw_interface::DualRobotInterface, not
//      here) - this class just latches it as the position of record.
//   2. If hw_status goes stale (hardware link lost, or whatever's
//      publishing it died), the robot must stop advancing any in-progress
//      command and instead hold at the last known good position until
//      hw_status resumes - RobotNode reads latchedPosition() for
//      that hold command while isFaulted().
//
// All state-dependent decision logic (what onSoftwareUp/onHwStatusReceived/
// checkTimeout actually do, and what isReady()/isFaulted() return) is
// delegated to whichever IRobotState (state_manager/states/) is currently
// active - this class holds the data every state needs (latched position,
// timestamps, thresholds) and dispatches events to the current state,
// rather than deciding transitions itself. Adding a new state (e.g. a
// future kCalibrating for humanoids) means adding one new IRobotState
// implementation - this class does not need to change.
class RobotStateMachine {
public:
    RobotStateMachine(robot_config::BackendMode backend_mode, double hw_status_timeout_sec,
                       double divergence_warn_threshold);

    // Not copyable/movable: this is safety-critical latched state (fault
    // status, last known good position) meant to have exactly one live
    // instance (RobotNode's state_machine_ member). A copy would silently
    // create a second instance whose state can diverge from the real one -
    // deleting these makes that a compile error instead of a runtime hazard.
    RobotStateMachine(const RobotStateMachine&) = delete;
    RobotStateMachine& operator=(const RobotStateMachine&) = delete;
    RobotStateMachine(RobotStateMachine&&) = delete;
    RobotStateMachine& operator=(RobotStateMachine&&) = delete;

    // Call once, after the node has finished setting up its
    // publishers/subscriptions/timers.
    void onSoftwareUp();

    // Call every time a fresh hw_status arrives. Latches position
    // unconditionally (including during/after a fault, so recovery is
    // just "the next hw_status re-latches everything"), then lets the
    // current state decide whether this causes a transition.
    void onHwStatusReceived(double now_sec, const Eigen::VectorXd& position);

    // Call every time a fresh digital_twin_status arrives (backend.mode
    // "both" only). Updates the divergence check against the latched
    // hardware position; does not affect state transitions - not
    // state-dependent, so no dispatch to the current IRobotState.
    void onDigitalTwinStatusReceived(const Eigen::VectorXd& position);

    // Call periodically (e.g. 10 Hz) to detect hw_status going stale.
    void checkTimeout(double now_sec);

    RobotState state() const { return current_state_->id(); }
    bool isReady() const { return current_state_->isReady(); }
    bool isFaulted() const { return current_state_->isFaulted(); }

    // True exactly once per transition; callers should log the new state
    // when this is true, then it clears itself.
    bool consumeStateChanged();

    // Last known good position from hw_status - what to command during a
    // fault, and what a fresh hw_status re-latches on recovery.
    const Eigen::VectorXd& latchedPosition() const { return latched_position_; }

    bool divergenceWarning() const { return divergence_warning_active_; }
    double lastDivergence() const { return last_divergence_; }
    double divergenceThreshold() const { return divergence_warn_threshold_; }

    // --- Called by IRobotState implementations only (state_manager/states/) ---

    // Switches the active state; marks consumeStateChanged() true iff this
    // actually changes which state is active.
    void transitionTo(const IRobotState& new_state);

    // Which "ready" state matches the configured backend_mode - used by
    // SoftwareUpState/HardwareLinkLostState to know which ready state to
    // transition into (backend_mode_ is Context data, not any one state's).
    const IRobotState& readyStateForBackendMode() const;

    // Whether now_sec is past hw_status_timeout_sec_ since the last latched
    // hw_status - used by ReadyStateBase::checkTimeout.
    bool hwStatusTimedOut(double now_sec) const;

private:
    robot_config::BackendMode backend_mode_;
    double hw_status_timeout_sec_;
    double divergence_warn_threshold_;

    const IRobotState* current_state_;
    bool state_changed_ = true;
    double last_hw_status_time_sec_ = -1.0;
    Eigen::VectorXd latched_position_;

    double last_divergence_ = 0.0;
    bool divergence_warning_active_ = false;
};

}  // namespace state_orchestrator
