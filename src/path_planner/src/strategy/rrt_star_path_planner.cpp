#include "path_planner/strategy/rrt_star_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace path_planner {

EdgeCostFn jointLimitMarginCostFn(const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
                                    double weight, double margin_scale) {
    return [lower_bound, upper_bound, weight, margin_scale](const Eigen::VectorXd& from,
                                                              const Eigen::VectorXd& to) {
        double penalty = 0.0;
        for (int i = 0; i < to.size(); ++i) {
            const double margin_to_nearest_limit =
                std::max(0.0, std::min(to[i] - lower_bound[i], upper_bound[i] - to[i]));
            penalty += std::exp(-margin_to_nearest_limit / margin_scale);
        }
        return euclideanEdgeCost(from, to) + weight * penalty;
    };
}

RrtStarPathPlanner::RrtStarPathPlanner(RrtStarConfig config, IsStateValidFn is_state_valid, EdgeCostFn edge_cost)
    : config_(std::move(config)),
      is_state_valid_(std::move(is_state_valid)),
      edge_cost_(std::move(edge_cost)),
      rng_(config_.seed) {
    if (config_.lower_bound.size() == 0 || config_.upper_bound.size() == 0) {
        throw std::invalid_argument(
            "RrtStarPathPlanner: lower_bound/upper_bound must be set to real sampling bounds "
            "(e.g. this robot's joint position_min/position_max) - RRT* cannot sample without them");
    }
    if (config_.lower_bound.size() != config_.upper_bound.size()) {
        throw std::invalid_argument("RrtStarPathPlanner: lower_bound/upper_bound size mismatch");
    }
}

Eigen::VectorXd RrtStarPathPlanner::sampleState(const Eigen::VectorXd& goal) {
    std::uniform_real_distribution<double> bias_roll(0.0, 1.0);
    if (bias_roll(rng_) < config_.goal_bias) {
        return goal;
    }
    Eigen::VectorXd sample(config_.lower_bound.size());
    for (int i = 0; i < sample.size(); ++i) {
        std::uniform_real_distribution<double> dist(config_.lower_bound[i], config_.upper_bound[i]);
        sample[i] = dist(rng_);
    }
    return sample;
}

int RrtStarPathPlanner::nearestNodeIndex(const Eigen::VectorXd& state) const {
    int best_index = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < tree_.size(); ++i) {
        const double dist = (tree_[i].state - state).norm();
        if (dist < best_dist) {
            best_dist = dist;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}

std::vector<int> RrtStarPathPlanner::nodesWithinRadius(const Eigen::VectorXd& state) const {
    std::vector<int> result;
    for (std::size_t i = 0; i < tree_.size(); ++i) {
        if ((tree_[i].state - state).norm() <= config_.rewire_radius) {
            result.push_back(static_cast<int>(i));
        }
    }
    return result;
}

Eigen::VectorXd RrtStarPathPlanner::steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    const Eigen::VectorXd delta = to - from;
    const double dist = delta.norm();
    if (dist <= config_.step_size) {
        return to;
    }
    return from + delta * (config_.step_size / dist);
}

bool RrtStarPathPlanner::isEdgeValid(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const {
    // Discretize the segment and check each sample - checking only the two
    // endpoints would miss an obstacle sitting entirely between them.
    const double dist = (to - from).norm();
    const int num_checks = std::max(1, static_cast<int>(std::ceil(dist / (config_.step_size * 0.25))));
    for (int i = 0; i <= num_checks; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(num_checks);
        if (!is_state_valid_(from + t * (to - from))) {
            return false;
        }
    }
    return true;
}

void RrtStarPathPlanner::propagateCostToDescendants(int node_index) {
    // No children-adjacency list is kept (this is a flat vector<Node>, not
    // a real tree structure), so finding node_index's children means an
    // O(n) rescan - not the fastest possible, but tree sizes here are
    // bounded by max_iterations, and correctness (every descendant's cost
    // staying consistent with its ancestors after a rewire) matters more
    // than micro-optimizing this for what this class is for right now.
    for (std::size_t i = 0; i < tree_.size(); ++i) {
        if (tree_[i].parent_index == node_index) {
            tree_[i].cost = tree_[node_index].cost + edge_cost_(tree_[node_index].state, tree_[i].state);
            propagateCostToDescendants(static_cast<int>(i));
        }
    }
}

std::vector<Eigen::VectorXd> RrtStarPathPlanner::extractPath(int goal_node_index) const {
    std::vector<Eigen::VectorXd> path;
    int current = goal_node_index;
    while (current != -1) {
        path.push_back(tree_[current].state);
        current = tree_[current].parent_index;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Eigen::VectorXd> RrtStarPathPlanner::planPath(const Eigen::VectorXd& start,
                                                           const Eigen::VectorXd& goal) {
    if (start.size() != goal.size() || start.size() != config_.lower_bound.size()) {
        throw std::invalid_argument(
            "RrtStarPathPlanner: start/goal/bounds dimension mismatch - this instance was "
            "configured for a fixed dimensionality (see class doc comment's known limitation)");
    }
    if (!is_state_valid_(start)) {
        throw std::runtime_error("RrtStarPathPlanner: start state is not valid");
    }
    if (!is_state_valid_(goal)) {
        throw std::runtime_error("RrtStarPathPlanner: goal state is not valid");
    }

    tree_.clear();
    tree_.push_back({start, -1, 0.0});

    int best_goal_index = -1;
    double best_goal_cost = std::numeric_limits<double>::max();

    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        const Eigen::VectorXd sample = sampleState(goal);
        const int nearest_index = nearestNodeIndex(sample);
        const Eigen::VectorXd new_state = steer(tree_[nearest_index].state, sample);

        if (!is_state_valid_(new_state) || !isEdgeValid(tree_[nearest_index].state, new_state)) {
            continue;
        }

        // RRT*'s actual difference from plain RRT: instead of always
        // parenting the new node to its nearest neighbor, consider every
        // nearby node and pick whichever gives the lowest cost-to-come -
        // this is what lets the tree's paths improve over time instead of
        // staying locked into whichever connection happened to form first.
        std::vector<int> near_indices = nodesWithinRadius(new_state);
        if (near_indices.empty()) {
            near_indices.push_back(nearest_index);
        }

        int best_parent = nearest_index;
        double best_cost = tree_[nearest_index].cost + edge_cost_(tree_[nearest_index].state, new_state);
        for (int idx : near_indices) {
            const double candidate_cost = tree_[idx].cost + edge_cost_(tree_[idx].state, new_state);
            if (candidate_cost < best_cost && isEdgeValid(tree_[idx].state, new_state)) {
                best_cost = candidate_cost;
                best_parent = idx;
            }
        }

        const int new_index = static_cast<int>(tree_.size());
        tree_.push_back({new_state, best_parent, best_cost});

        // Rewire: for every other nearby node, check whether reaching it
        // THROUGH the just-added node would now be cheaper than its
        // current parent - this is the step that lets earlier, suboptimal
        // connections get corrected later as the tree fills in, and it's
        // what makes this RRT*, not plain RRT.
        for (int idx : near_indices) {
            if (idx == best_parent) continue;
            const double rewired_cost = best_cost + edge_cost_(new_state, tree_[idx].state);
            if (rewired_cost < tree_[idx].cost && isEdgeValid(new_state, tree_[idx].state)) {
                tree_[idx].parent_index = new_index;
                tree_[idx].cost = rewired_cost;
                propagateCostToDescendants(idx);
            }
        }

        const double dist_to_goal = (new_state - goal).norm();
        if (dist_to_goal <= config_.goal_tolerance && best_cost < best_goal_cost) {
            best_goal_index = new_index;
            best_goal_cost = best_cost;
        }
    }

    if (best_goal_index == -1) {
        throw std::runtime_error(
            "RrtStarPathPlanner: no path found to within goal_tolerance after max_iterations - "
            "increase max_iterations/step_size/rewire_radius, or the goal may be unreachable "
            "given the injected IsStateValidFn");
    }

    std::vector<Eigen::VectorXd> path = extractPath(best_goal_index);
    // Land exactly on goal, not just "within tolerance" of it - but only if
    // the tree's own last node isn't already exactly there (goal_bias makes
    // sampling `goal` itself common), or the caller ends up with a
    // duplicated, zero-length final segment that the interpolator then
    // wastefully densifies into `points_per_segment` copies of one point.
    if (path.empty() || !path.back().isApprox(goal)) {
        path.push_back(goal);
    }
    return path;
}

}  // namespace path_planner
