#pragma once

#include "motion_planner/motion_planner_types.hpp"
#include "trajectory_planner/common/trajectory_types.hpp"

namespace motion_planner {

// Top-level port: plan a trajectory from the robot's current joint state to
// a target (joint- or Cartesian-space). This is the only interface a ROS
// (or any other) integration layer should depend on - it has no ROS types
// in its signature, so it can be used unmodified from a node, a test, or a
// simulator.
class IMotionPlanner {
public:
    virtual ~IMotionPlanner() = default;
    virtual trajectory_planner::Trajectory plan(const Eigen::VectorXd& current_joint_position,
                                                  const MotionTarget& target,
                                                  const trajectory_planner::MotionConstraints& constraints) = 0;
};

}  // namespace motion_planner
