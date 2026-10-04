#pragma once

#include <vector>

#include <Eigen/Dense>

namespace trajectory_planner {

struct MotionConstraints {
    double max_velocity = 1.0;      // rad/s (or m/s, depending on joint type)
    double max_acceleration = 1.0;  // rad/s^2

    // Optional per-joint velocity limits (robot_config's JointLimits::
    // velocity_max, one entry per joint, empty if not supplied). The
    // generator picks whichever joint would be the bottleneck - the one
    // that can't keep up at max_velocity given how much of the move is
    // "its share" - and uses that joint's own limit instead of
    // max_velocity, capped by max_velocity so a caller can still ask for
    // something slower. Left empty, behavior is unchanged: max_velocity
    // alone (as one shared number for every joint) is used, same as
    // before this field existed.
    std::vector<double> joint_velocity_max;
};

// A single time-stamped point in a planned joint-space trajectory.
struct TrajectoryPoint {
    double time_from_start = 0.0;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd acceleration;
};

using Trajectory = std::vector<TrajectoryPoint>;

}  // namespace trajectory_planner
