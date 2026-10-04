#pragma once

#include <Eigen/Dense>

namespace collision_checking {

// Port interface for "is this joint configuration allowed": no collision,
// or any other constraint a caller cares about. Concrete implementations
// (PlanningScene today; an octomap-backed or GPU-accelerated checker
// later) implement this; consumers (path_planner::RrtStarPathPlanner,
// motion_planner::MotionPlanner) depend only on this shape - in practice,
// via the even more generic std::function callable both already accept
// (see IsStateValidFn in rrt_star_path_planner.hpp), so neither package
// needs to depend on this one directly. This interface exists for
// collision_checking's own consumers/tests, and for anything that wants a
// named type instead of a bare std::function.
class ICollisionChecker {
public:
    virtual ~ICollisionChecker() = default;
    virtual bool isStateValid(const Eigen::VectorXd& joint_positions) const = 0;
};

}  // namespace collision_checking
