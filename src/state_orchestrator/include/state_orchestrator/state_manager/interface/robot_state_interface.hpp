#pragma once

#include <Eigen/Dense>

#include "state_orchestrator/state_manager/robot_state.hpp"

namespace state_orchestrator {

class RobotStateMachine;

// GoF State pattern: each concrete RobotState value gets its own class
// implementing this interface, holding the behavior that used to live as
// scattered if-guards inside RobotStateMachine's event methods. Adding a
// new state (e.g. a future kCalibrating for humanoids) means adding one new
// class here - existing state classes never need to change, satisfying
// Open/Closed in a way a shared transition table doesn't (a table is still
// one shared artifact every new state has to edit).
//
// Concrete states are stateless (no instance data - all real state lives in
// RobotStateMachine, the Context), so each is a single shared instance
// (see each class's static instance() accessor) rather than something
// constructed per transition.
//
// Default implementations are no-ops / "not ready" / "not faulted" so each
// concrete state only overrides what actually differs for it - most states
// only override 1-2 of these.
class IRobotState {
public:
    virtual ~IRobotState() = default;

    // Identity tag for logging/tests (RobotStateMachine::state(),
    // toString()) - polymorphism replaces the transition-logic switch that
    // used to make this class hard to extend, not the identity concept
    // itself; keeping an id is what lets RobotNode log human-readable state
    // names without needing typeid/dynamic_cast.
    virtual RobotState id() const = 0;

    virtual bool isReady() const { return false; }
    virtual bool isFaulted() const { return false; }

    virtual void onSoftwareUp(RobotStateMachine& machine) const { (void)machine; }
    virtual void onHwStatusReceived(RobotStateMachine& machine, double now_sec,
                                     const Eigen::VectorXd& position) const {
        (void)machine;
        (void)now_sec;
        (void)position;
    }
    virtual void checkTimeout(RobotStateMachine& machine, double now_sec) const {
        (void)machine;
        (void)now_sec;
    }
};

}  // namespace state_orchestrator
