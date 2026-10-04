#pragma once

#include <vector>

#include "robot_hw_interface/interface/robot_interface.hpp"

namespace hw_interface {

// In-memory stand-in for real hardware: write() is echoed straight back by
// read(), as if every command were tracked perfectly and instantly. Useful
// for exercising the rest of the stack without any real I/O, and as the
// template for a real backend (see ArduinoHardwareInterface).
class DummyHardwareInterface : public robot_hw_interface::IRobotInterface {
public:
    // home_joint_position seeds state_.position (what read() reports
    // before any write() call); defaults to all-zero if left empty.
    explicit DummyHardwareInterface(int num_joints, const std::vector<double>& home_joint_position = {});

    void write(const robot_hw_interface::JointCommand& command) override;
    robot_hw_interface::JointState read() override;
    int numJoints() const override;

private:
    int num_joints_;
    robot_hw_interface::JointState state_;
};

}  // namespace hw_interface
