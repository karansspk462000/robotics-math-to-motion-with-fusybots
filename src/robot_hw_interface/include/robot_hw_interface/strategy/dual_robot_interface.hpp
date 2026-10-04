#pragma once

#include <memory>

#include "robot_hw_interface/interface/robot_interface.hpp"

namespace robot_hw_interface {

// Composite IRobotInterface for "both" mode: write() sends the exact same
// command to both backends (e.g. real hardware and its digital twin, so
// they never receive different commands and can't silently diverge).
//
// read() is NOT configurable to prefer either side: it always reports
// `primary`'s state as ground truth. Construct with real hardware as
// `primary` and the digital twin as `secondary` - never the other way
// round. This is a deliberate hard invariant, not a knob, because
// accidentally reporting simulated state as if it were the real robot's
// is exactly the failure mode this class exists to prevent.
//
// read() still queries both backends every call (so the secondary keeps
// advancing, e.g. a digital twin's internal integration) and caches the
// secondary's reading for comparison - see lastSecondaryState() - so a
// caller (e.g. a state-machine/orchestrator layer) can detect and log
// divergence between hardware and its twin without this class itself
// needing any logging/ROS dependency.
class DualRobotInterface : public IRobotInterface {
public:
    DualRobotInterface(std::unique_ptr<IRobotInterface> primary,
                        std::unique_ptr<IRobotInterface> secondary);

    void write(const JointCommand& command) override;
    JointState read() override;
    int numJoints() const override;

    const JointState& lastSecondaryState() const { return last_secondary_state_; }

private:
    std::unique_ptr<IRobotInterface> primary_;
    std::unique_ptr<IRobotInterface> secondary_;
    JointState last_secondary_state_;
};

}  // namespace robot_hw_interface
