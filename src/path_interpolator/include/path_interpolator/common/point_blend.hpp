#pragma once

#include <Eigen/Dense>

#include "path_interpolator/common/pose_space.hpp"

namespace path_interpolator {

// Blends two same-typed points at parameter t in [0, 1]. The one primitive
// every interpolation behavior (linear, spline, bezier, quintic, ...) needs
// a joint-vs-Cartesian version of; everything else about a behavior (which
// segments, which t values, how many points) doesn't depend on space at
// all, so behavior classes should call this rather than duplicate
// themselves per space. Throws if space is kCartesian and a or b isn't a
// 7-element packed pose.
Eigen::VectorXd blendPoint(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double t, PoseSpace space);

}  // namespace path_interpolator
