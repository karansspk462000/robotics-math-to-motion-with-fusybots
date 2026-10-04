#pragma once

#include <functional>
#include <random>
#include <vector>

#include "path_planner/interface/path_planner_interface.hpp"

namespace path_planner {

// Per-dimension sampling bounds and tuning knobs for RrtStarPathPlanner.
// lower_bound/upper_bound MUST be set to the real range this planner should
// sample within (e.g. this robot's joint position_min/position_max) -
// RRT* has no other way to know what a "reasonable" state looks like.
// Defaults for the rest are reasonable starting points for a joint-space
// search in radians, not universal constants - tune per robot if the tree
// converges too slowly or too coarsely.
struct RrtStarConfig {
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
    double step_size = 0.1;
    double goal_bias = 0.1;
    double goal_tolerance = 0.05;
    double rewire_radius = 0.5;
    int max_iterations = 2000;
    // Fixed by default: deterministic and reproducible (same seed, same
    // path, every run) rather than "true" randomness - deliberate, so a
    // test suite exercising this class isn't flaky. Pass a different seed
    // if you actually want run-to-run variation.
    unsigned int seed = 12345u;
};

// True if `state` may be part of a path (no collision, no other
// constraint the caller cares about). Defaults to "everything is valid" -
// this is the deliberately "dummy" half of this planner: the RRT*
// *algorithm* below is real, but with no real collision/obstacle model
// wired into this workspace yet, the only honest default to inject here is
// "always valid". Inject a real one (e.g. querying an occupancy grid or a
// self-collision checker) the same way any other Strategy dependency is
// injected in this codebase, once one exists - this class does not need to
// change to use it; see test_path_planner.cpp's
// RrtStarPathPlanner.AvoidsAnInjectedObstacle for a worked example with a
// real (if synthetic) obstacle.
using IsStateValidFn = std::function<bool(const Eigen::VectorXd&)>;

inline bool alwaysValid(const Eigen::VectorXd&) { return true; }

// Cost of moving from `from` to `to`, accumulated along the tree as
// cost-to-come and what RRT*'s parent-selection/rewiring steps actually
// optimize. Defaults to plain Euclidean distance (euclideanEdgeCost) -
// pure "shortest path". Inject a different one to make the search prefer
// paths that also avoid, e.g., joint limits (see jointLimitMarginCostFn
// below) - the algorithm itself doesn't change, only what it treats as
// "expensive". A validity check (IsStateValidFn) and a cost function
// answer different questions: validity is binary (allowed or not), cost is
// "how much do we want to avoid this, given it's still allowed" - a
// near-joint-limit state should usually be a *worse* choice, not an
// *invalid* one.
using EdgeCostFn = std::function<double(const Eigen::VectorXd& from, const Eigen::VectorXd& to)>;

inline double euclideanEdgeCost(const Eigen::VectorXd& from, const Eigen::VectorXd& to) {
    return (to - from).norm();
}

// An EdgeCostFn that adds a penalty for `to` being close to its own joint
// limits (lower_bound/upper_bound), on top of the plain Euclidean step
// length - a near-limit state stays VALID (this never rejects anything,
// that's IsStateValidFn's job), it just becomes a more expensive part of
// the tree to route through, so RRT*'s cost-driven parent/rewire choices
// naturally prefer routes that keep more margin from the limits when a
// similarly-short alternative exists. `margin_scale` (same units as the
// bounds, e.g. radians) sets how quickly the penalty decays with distance
// from the nearest limit - roughly `weight` right at the limit, decaying
// to near-zero by about 3x margin_scale away from it.
//
// A similar singularity-avoidance cost (penalizing configurations near a
// kinematic singularity, e.g. via a manipulability measure from the
// Jacobian) would follow the exact same shape - an EdgeCostFn closure -
// but needs kinematics::InverseKinematics::computeJacobian, which
// path_planner deliberately does not depend on (see this package's
// dimension-agnostic, kinematics-agnostic design). Build that closure at
// the composition root instead (same place PlanningScene's checker gets
// wrapped into an IsStateValidFn - see collision_checking package),
// capturing the real InverseKinematics Context, and pass the resulting
// EdgeCostFn in here unchanged.
EdgeCostFn jointLimitMarginCostFn(const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound,
                                    double weight, double margin_scale = 0.1);

// Standard RRT* (Karaman & Frazzoli, 2011): incrementally builds a tree of
// collision-free states rooted at `start`, using nearest-neighbor steering
// plus a rewiring step that keeps improving the tree's costs as it grows -
// unlike plain RRT, which never revisits a connection decision once made.
// Returns the lowest-cost path found to within goal_tolerance of `goal`
// after max_iterations, by walking the tree's parent pointers back to
// start.
//
// This class is constructed with FIXED-dimension bounds (whatever
// lower_bound/upper_bound it's given), so it can only stand in for a
// path_planner_method used at ONE dimensionality - e.g. joint-space
// planning (6 dims on this robot). MotionPlanner does NOT use this for
// Cartesian-space planning - there was never a real "planning" step there
// to begin with (no Cartesian obstacle model exists), so
// MotionPlanner::planCartesianSpace builds its sparse 2-point
// {current, target} path directly rather than calling any IPathPlanner.
// If a real Cartesian-space planner is ever needed, it'll need its own
// dimension-appropriate instance, not this same joint-space one.
class RrtStarPathPlanner : public IPathPlanner {
public:
    explicit RrtStarPathPlanner(RrtStarConfig config, IsStateValidFn is_state_valid = alwaysValid,
                                 EdgeCostFn edge_cost = euclideanEdgeCost);

    std::vector<Eigen::VectorXd> planPath(const Eigen::VectorXd& start,
                                           const Eigen::VectorXd& goal) override;

private:
    struct Node {
        Eigen::VectorXd state;
        int parent_index;
        double cost;  // cost-to-come from start, along the tree's current edges
    };

    Eigen::VectorXd sampleState(const Eigen::VectorXd& goal);
    int nearestNodeIndex(const Eigen::VectorXd& state) const;
    std::vector<int> nodesWithinRadius(const Eigen::VectorXd& state) const;
    Eigen::VectorXd steer(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;
    bool isEdgeValid(const Eigen::VectorXd& from, const Eigen::VectorXd& to) const;
    void propagateCostToDescendants(int node_index);
    std::vector<Eigen::VectorXd> extractPath(int goal_node_index) const;

    RrtStarConfig config_;
    IsStateValidFn is_state_valid_;
    EdgeCostFn edge_cost_;
    std::mt19937 rng_;
    std::vector<Node> tree_;
};

}  // namespace path_planner
