#include "state_orchestrator/state_manager/robot_state.hpp"

namespace state_orchestrator {

std::string toString(RobotState state) {
    switch (state) {
        case RobotState::kStartingRobot: return "STARTING_ROBOT";
        case RobotState::kSoftwareUp: return "SOFTWARE_UP";
        case RobotState::kHardwareReady: return "HARDWARE_READY";
        case RobotState::kDigitalTwinReady: return "DIGITAL_TWIN_READY";
        case RobotState::kHardwareAndDigitalTwinReady: return "HARDWARE_AND_DIGITAL_TWIN_READY";
        case RobotState::kHardwareLinkLost: return "HARDWARE_LINK_LOST";
    }
    return "UNKNOWN";
}

}  // namespace state_orchestrator
