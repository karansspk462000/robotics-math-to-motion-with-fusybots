#pragma once

#include <memory>
#include <string>

#include "path_planner/interface/path_planner_interface.hpp"
#include "path_planner/strategy/rrt_star_path_planner.hpp"

namespace path_planner {

// Builds the IPathPlanner named by `method` (currently only "rrt_star").
// lower_bound/upper_bound are required - pass the real sampling bounds
// (e.g. this robot's joint position_min/position_max); a mismatched or
// empty bound throws a clear error rather than silently misbehaving.
//
// is_state_valid/edge_cost are optional - leave them default-constructed
// (empty std::function) to get RrtStarPathPlanner's own defaults (always
// valid; plain Euclidean cost). Pass a real one to plug in collision
// checking (see the collision_checking package - PlanningScene wrapped in
// a lambda) or a cost preference (see jointLimitMarginCostFn) without this
// factory or path_planner needing to depend on either.
std::unique_ptr<IPathPlanner> createPathPlanner(const std::string& method,
                                                  const Eigen::VectorXd& lower_bound = Eigen::VectorXd(),
                                                  const Eigen::VectorXd& upper_bound = Eigen::VectorXd(),
                                                  IsStateValidFn is_state_valid = nullptr,
                                                  EdgeCostFn edge_cost = nullptr);

}  // namespace path_planner
