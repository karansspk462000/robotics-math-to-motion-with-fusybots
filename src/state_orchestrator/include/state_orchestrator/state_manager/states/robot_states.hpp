#pragma once

#include "state_orchestrator/state_manager/interface/robot_state_interface.hpp"

namespace state_orchestrator {

// Every concrete IRobotState lives in this one file - each is small (most
// override one method), so a class-per-file split just added navigation
// overhead without any real payoff. Adding a NEW state still never
// requires editing any existing class body here (OCP is about not
// modifying existing code, not about which file it lives in) - it just
// means appending a new class to this file instead of adding a new one.

// Initial state before RobotNode has finished setting up (before
// onSoftwareUp() is called). hw_status arriving this early is still
// latched by RobotStateMachine (unconditional, see its onHwStatusReceived),
// but does not by itself advance the state - only onSoftwareUp() does.
class StartingRobotState : public IRobotState {
public:
    static const StartingRobotState& instance();

    RobotState id() const override { return RobotState::kStartingRobot; }
    void onSoftwareUp(RobotStateMachine& machine) const override;
};

// Software is up but no hw_status has arrived yet. The first hw_status
// advances straight to whichever "ready" state matches backend_mode.
class SoftwareUpState : public IRobotState {
public:
    static const SoftwareUpState& instance();

    RobotState id() const override { return RobotState::kSoftwareUp; }
    void onHwStatusReceived(RobotStateMachine& machine, double now_sec,
                             const Eigen::VectorXd& position) const override;
};

// Shared behavior for every "ready" state (kHardwareReady/kDigitalTwinReady/
// kHardwareAndDigitalTwinReady) - they differ only in id() (which backend
// mode they represent), never in behavior, so that behavior lives here
// once instead of being triplicated across three leaf classes.
class ReadyStateBase : public IRobotState {
public:
    bool isReady() const override { return true; }

    // A fresh hw_status while already ready just re-latches (done
    // unconditionally by RobotStateMachine before dispatch) - no
    // transition needed, hence no override here.

    void checkTimeout(RobotStateMachine& machine, double now_sec) const override;
};

// backend.mode "hardware" - all ReadyStateBase behavior, only id() differs.
class HardwareReadyState : public ReadyStateBase {
public:
    static const HardwareReadyState& instance();

    RobotState id() const override { return RobotState::kHardwareReady; }
};

// backend.mode "digital_twin" - all ReadyStateBase behavior, only id() differs.
class DigitalTwinReadyState : public ReadyStateBase {
public:
    static const DigitalTwinReadyState& instance();

    RobotState id() const override { return RobotState::kDigitalTwinReady; }
};

// backend.mode "both" - all ReadyStateBase behavior, only id() differs.
class HardwareAndDigitalTwinReadyState : public ReadyStateBase {
public:
    static const HardwareAndDigitalTwinReadyState& instance();

    RobotState id() const override { return RobotState::kHardwareAndDigitalTwinReady; }
};

// hw_status went stale past the configured timeout. RobotNode reads
// isFaulted()/latchedPosition() while here to hold the last known good
// position instead of advancing any in-progress trajectory. Recovery is
// automatic: the next hw_status re-latches and returns to ready.
class HardwareLinkLostState : public IRobotState {
public:
    static const HardwareLinkLostState& instance();

    RobotState id() const override { return RobotState::kHardwareLinkLost; }
    bool isFaulted() const override { return true; }
    void onHwStatusReceived(RobotStateMachine& machine, double now_sec,
                             const Eigen::VectorXd& position) const override;
};

}  // namespace state_orchestrator
