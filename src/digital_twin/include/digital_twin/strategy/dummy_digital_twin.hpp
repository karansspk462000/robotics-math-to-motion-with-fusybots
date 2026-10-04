#pragma once

#include <chrono>
#include <vector>

#include "robot_hw_interface/interface/robot_interface.hpp"

namespace digital_twin {

// Placeholder IRobotInterface backend standing in for a real physics
// simulator (pybullet/mujoco/gazebo - see digital_twin_factory.cpp for
// those extension points). Rather than snapping instantly to a commanded
// target (which a real simulator never would), it integrates position
// towards the target at max_joint_speed and integrates velocity commands
// directly, using wall-clock time between read() calls as dt - just enough
// dynamics to behave like *a* robot rather than a teleporting one.
class DummyDigitalTwin : public robot_hw_interface::IRobotInterface {
public:
    // home_joint_position seeds both the reported state and the held
    // position target (so read() doesn't immediately drift back towards
    // zero before any real command arrives); defaults to all-zero if left
    // empty.
    explicit DummyDigitalTwin(int num_joints, double max_joint_speed = 2.0,
                               const std::vector<double>& home_joint_position = {});

    void write(const robot_hw_interface::JointCommand& command) override;
    robot_hw_interface::JointState read() override;
    int numJoints() const override;

private:
    int num_joints_;
    double max_joint_speed_;

    robot_hw_interface::JointCommand target_command_;
    robot_hw_interface::JointState state_;
    std::chrono::steady_clock::time_point last_read_time_;
};

}  // namespace digital_twin
