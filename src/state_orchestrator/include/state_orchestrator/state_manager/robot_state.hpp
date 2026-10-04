#pragma once

#include <string>

namespace state_orchestrator {

// States move forward automatically as their entry condition is met - there
// is no external "advance" call. RobotNode just feeds events
// (onSoftwareUp/onHwStatusReceived/onDigitalTwinStatusReceived) and a
// periodic checkTimeout() to RobotStateMachine, which delegates the actual
// decision to whichever IRobotState is currently active (see
// state_manager/states/). This enum is purely an identity tag - which
// concrete IRobotState is active decides behavior, not this value.
enum class RobotState {
    kStartingRobot,
    kSoftwareUp,
    kHardwareReady,                  // backend.mode "hardware", hw_status flowing
    kDigitalTwinReady,                // backend.mode "digital_twin", hw_status flowing
    kHardwareAndDigitalTwinReady,     // backend.mode "both", hw_status flowing
    kHardwareLinkLost,                // was ready, hw_status went stale - fault
};

std::string toString(RobotState state);

}  // namespace state_orchestrator
