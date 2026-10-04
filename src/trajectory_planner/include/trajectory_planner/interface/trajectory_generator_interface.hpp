#pragma once

#include <vector>

#include "trajectory_planner/common/trajectory_types.hpp"

namespace trajectory_planner {

// Port interface for time-parameterizing a dense geometric path (as produced
// by an IPathInterpolator) into a timed trajectory subject to velocity and
// acceleration limits. Concrete strategies (trapezoidal, S-curve, ...)
// implement this; consumers (e.g. motion_planner) depend only on this
// interface.
class ITrajectoryGenerator {
public:
    virtual ~ITrajectoryGenerator() = default;
    virtual Trajectory generateTrajectory(const std::vector<Eigen::VectorXd>& path,
                                           const MotionConstraints& constraints) = 0;
};

}  // namespace trajectory_planner
