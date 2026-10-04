#pragma once

#include <string>

#include "robot_hw_interface/interface/robot_interface.hpp"

namespace hw_interface {

// Reference real-hardware backend: talks to an Arduino (or anything
// speaking the same simple line protocol) over a POSIX serial port.
//
// Wire protocol (adapt to your firmware as needed):
//   write() sends one line: "<M> <q0> <q1> ... <qN-1>\n"
//     where <M> is 'P' (position), 'V' (velocity) or 'T' (torque),
//     matching JointCommand::control_mode.
//   read() blocks (up to a timeout) for one line the firmware streams
//   back: "S <p0>..<pN-1> <v0>..<vN-1> <t0>..<tN-1> <c0>..<cN-1>\n"
//     (position, velocity, torque, current, each N values). If no line
//     arrives before the timeout, the last known state is returned rather
//     than blocking indefinitely.
//
// This is unverified against real hardware (no Arduino attached in this
// environment) - treat the protocol as a documented starting point, not a
// tested contract, and adjust framing/parsing to match your firmware.
class ArduinoHardwareInterface : public robot_hw_interface::IRobotInterface {
public:
    ArduinoHardwareInterface(const std::string& serial_port, int baud_rate, int num_joints,
                              int read_timeout_ms = 100);
    ~ArduinoHardwareInterface() override;

    void write(const robot_hw_interface::JointCommand& command) override;
    robot_hw_interface::JointState read() override;
    int numJoints() const override;

private:
    std::string readLine();

    int file_descriptor_;
    int num_joints_;
    robot_hw_interface::JointState last_known_state_;
};

}  // namespace hw_interface
