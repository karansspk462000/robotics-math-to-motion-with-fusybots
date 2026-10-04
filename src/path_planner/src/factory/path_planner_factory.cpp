#include "path_planner/factory/path_planner_factory.hpp"

#include <stdexcept>

namespace path_planner {

std::unique_ptr<IPathPlanner> createPathPlanner(const std::string& method,
                                                  const Eigen::VectorXd& lower_bound,
                                                  const Eigen::VectorXd& upper_bound,
                                                  IsStateValidFn is_state_valid,
                                                  EdgeCostFn edge_cost) {
    if (method == "rrt_star") {
        if (lower_bound.size() == 0 || upper_bound.size() == 0) {
            throw std::runtime_error(
                "path_planner_factory: 'rrt_star' requires real lower_bound/upper_bound "
                "sampling bounds to be passed to createPathPlanner - none were given");
        }
        RrtStarConfig config;
        config.lower_bound = lower_bound;
        config.upper_bound = upper_bound;
        return std::make_unique<RrtStarPathPlanner>(
            config, is_state_valid ? std::move(is_state_valid) : IsStateValidFn(alwaysValid),
            edge_cost ? std::move(edge_cost) : EdgeCostFn(euclideanEdgeCost));
    }
    throw std::runtime_error("path_planner_factory: unknown path_planner_method '" + method + "'");
}

}  // namespace path_planner
