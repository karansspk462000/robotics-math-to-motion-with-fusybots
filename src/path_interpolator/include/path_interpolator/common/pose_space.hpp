#pragma once

namespace path_interpolator {

// Which kind of point a strategy is interpolating - the one thing that
// varies between an otherwise-identical "joint" and "Cartesian" version of
// the same interpolation behavior (linear, spline, bezier, quintic, ...).
// A behavior class takes this at construction and delegates to blendPoint()
// (see point_blend.hpp) wherever it needs to blend two same-typed points,
// instead of duplicating a joint-vs-Cartesian version of itself.
enum class PoseSpace {
    kJoint,       // arbitrary-length joint vector; every component is lerped
    kCartesian,   // packed pose [x, y, z, qx, qy, qz, qw]; position is
                  // lerped, orientation is slerped (see math_utils::packPoseToVector7)
};

}  // namespace path_interpolator
