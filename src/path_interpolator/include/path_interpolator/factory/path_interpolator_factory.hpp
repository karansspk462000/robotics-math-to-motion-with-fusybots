#pragma once

#include <memory>
#include <string>

#include "path_interpolator/interface/path_interpolator_interface.hpp"

namespace path_interpolator {

// Builds the IPathInterpolator named by `method` (e.g. "linear",
// "linear_cartesian"). points_per_segment controls output density: the
// trajectory_planner stage does NOT synthesize any new positions of its
// own, it only picks already-existing points from this output at the
// right time - so points_per_segment must be dense enough to cover the
// whole eventual trajectory (its total duration / its own time_step_), or
// the motion will visibly stair-step (long runs of a repeated position,
// then a jump) with a velocity signal inconsistent with that stair-
// stepped position. There's no way to guarantee sufficiency in general
// (the trajectory's duration isn't known until after interpolation runs),
// so pick this generously for how slow/long your moves actually get.
std::unique_ptr<IPathInterpolator> createPathInterpolator(const std::string& method,
                                                            int points_per_segment = 1000);

}  // namespace path_interpolator
