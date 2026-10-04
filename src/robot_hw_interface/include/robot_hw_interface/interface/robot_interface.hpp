#pragma once

#include "robot_hw_interface/common/robot_interface_types.hpp"

namespace robot_hw_interface {

// The read()/write() contract every robot backend implements - real
// hardware (hw_interface package) and a digital twin (digital_twin
// package) are equally valid implementations of this interface, and
// completely interchangeable from the ROS boundary node's point of view.
// This is intentionally the same shape ros2_control's SystemInterface
// uses, kept minimal for our needs.
class IRobotInterface {
public:
    virtual ~IRobotInterface() = default;

    // Sends command to the backend. For a real hardware backend this means
    // writing to actuator registers/serial port; for a digital twin it
    // means stepping (or queuing a target for) the simulation.
    virtual void write(const JointCommand& command) = 0;

    // Reads the backend's current joint state.
    virtual JointState read() = 0;

    virtual int numJoints() const = 0;
};

}  // namespace robot_hw_interface
