#pragma once

#include <Eigen/Dense>

namespace motion_planner {

enum class PlanningSpace { kJoint, kCartesian };

// A target expressed in either joint space or Cartesian space; the caller
// sets planning_space to say which member is valid.
struct MotionTarget {
    PlanningSpace planning_space = PlanningSpace::kJoint;
    Eigen::VectorXd joint_position;
    Eigen::Matrix4d cartesian_pose = Eigen::Matrix4d::Identity();
};

}  // namespace motion_planner
