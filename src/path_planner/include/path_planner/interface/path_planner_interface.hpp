#pragma once

#include <vector>

#include <Eigen/Dense>

namespace path_planner {

// Port interface for geometric path planning: finds a sequence of waypoints
// from start to goal in joint space, with no notion of timing/velocity.
// Concrete strategies (straight-line, RRT, ...) implement this; consumers
// (e.g. motion_planner) depend only on this interface.
class IPathPlanner {
public:
    virtual ~IPathPlanner() = default;
    virtual std::vector<Eigen::VectorXd> planPath(const Eigen::VectorXd& start,
                                                    const Eigen::VectorXd& goal) = 0;
};

}  // namespace path_planner
