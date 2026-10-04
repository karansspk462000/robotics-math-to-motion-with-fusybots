#pragma once

#include <vector>

#include <Eigen/Dense>

namespace path_interpolator {

// Port interface for densifying/smoothing a sparse geometric path (as
// produced by an IPathPlanner) into closely spaced waypoints suitable for
// time-parameterization by a trajectory generator. Concrete strategies
// (linear, spline, ...) implement this.
class IPathInterpolator {
public:
    virtual ~IPathInterpolator() = default;
    virtual std::vector<Eigen::VectorXd> interpolatePath(
        const std::vector<Eigen::VectorXd>& path) = 0;
};

}  // namespace path_interpolator
