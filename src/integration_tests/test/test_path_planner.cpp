// Unit tests for path_planner in isolation - no kinematics/motion_planner
// dependency. RrtStarPathPlanner is now the only IPathPlanner Strategy in
// this package (the old trivial "just return {start, goal}" planner was
// removed - that was never really "path planning", and it's not something
// path_planner should offer at all: linear/cubic/curve are interpolation
// concepts, which path_interpolator already owns). These tests lock in
// real algorithmic properties (reaches the goal, respects bounds, and -
// the actual point of RRT* over a straight line - avoids an obstacle).
#include <gtest/gtest.h>

#include "path_planner/strategy/rrt_star_path_planner.hpp"

using path_planner::IsStateValidFn;
using path_planner::RrtStarConfig;
using path_planner::RrtStarPathPlanner;

namespace {

RrtStarConfig makeConfig(int dims, double lower, double upper) {
    RrtStarConfig config;
    config.lower_bound = Eigen::VectorXd::Constant(dims, lower);
    config.upper_bound = Eigen::VectorXd::Constant(dims, upper);
    return config;
}

}  // namespace

TEST(RrtStarPathPlanner, RequiresRealBounds) {
    RrtStarConfig config;  // lower_bound/upper_bound left empty
    EXPECT_THROW(RrtStarPathPlanner planner(config), std::invalid_argument);
}

TEST(RrtStarPathPlanner, ReachesGoalWithNoObstacles) {
    const RrtStarConfig config = makeConfig(3, -3.0, 3.0);
    RrtStarPathPlanner planner(config);

    Eigen::VectorXd start(3), goal(3);
    start << 0.0, 0.0, 0.0;
    goal << 2.0, 1.5, -1.0;

    const auto path = planner.planPath(start, goal);
    ASSERT_GE(path.size(), 2u);
    EXPECT_TRUE(path.front().isApprox(start));
    EXPECT_TRUE(path.back().isApprox(goal));

    // Every consecutive pair must be a real, bounded-length edge (no
    // teleporting between disconnected tree branches). A rewired parent
    // can be any node within rewire_radius, not just step_size away (that
    // bound only applies to a node's FIRST connection, via steer() from its
    // nearest neighbor) - the last edge lands exactly on goal, so it's
    // exempted (goal_tolerance, not rewire_radius, bounds it).
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
        const double edge_length = (path[i] - path[i - 1]).norm();
        EXPECT_LE(edge_length, config.rewire_radius + 1e-9)
            << "edge " << i << " is longer than rewire_radius - path is not a valid tree walk";
    }
}

TEST(RrtStarPathPlanner, EveryWaypointRespectsBounds) {
    RrtStarConfig config = makeConfig(2, -1.0, 1.0);
    RrtStarPathPlanner planner(config);

    Eigen::VectorXd start(2), goal(2);
    start << -0.9, -0.9;
    goal << 0.9, 0.9;

    const auto path = planner.planPath(start, goal);
    for (const auto& point : path) {
        for (int i = 0; i < point.size(); ++i) {
            EXPECT_GE(point[i], config.lower_bound[i] - 1e-9);
            EXPECT_LE(point[i], config.upper_bound[i] + 1e-9);
        }
    }
}

TEST(RrtStarPathPlanner, ThrowsWhenStartOrGoalIsInvalid) {
    IsStateValidFn reject_origin = [](const Eigen::VectorXd& state) { return state.norm() > 0.5; };
    RrtStarPathPlanner planner(makeConfig(2, -3.0, 3.0), reject_origin);

    Eigen::VectorXd invalid_start = Eigen::VectorXd::Zero(2);  // norm 0, rejected
    Eigen::VectorXd goal(2);
    goal << 2.0, 2.0;

    EXPECT_THROW(planner.planPath(invalid_start, goal), std::runtime_error);
}

// The actual point of RRT* over a straight line: with a real (if synthetic)
// obstacle injected as IsStateValidFn, the returned path must not pass
// through it - a straight line from start to goal here would go directly
// through the forbidden region, so this only passes if the algorithm
// genuinely routed around it.
TEST(RrtStarPathPlanner, AvoidsAnInjectedObstacle) {
    const Eigen::Vector2d obstacle_center(1.0, 0.0);
    const double obstacle_radius = 0.4;

    IsStateValidFn outside_circular_obstacle = [&](const Eigen::VectorXd& state) {
        return (state - obstacle_center).norm() > obstacle_radius;
    };

    RrtStarConfig config = makeConfig(2, -0.5, 2.5);
    config.step_size = 0.15;
    config.rewire_radius = 0.6;
    config.goal_tolerance = 0.1;
    config.max_iterations = 4000;

    RrtStarPathPlanner planner(config, outside_circular_obstacle);

    Eigen::VectorXd start(2), goal(2);
    start << 0.0, 0.0;
    goal << 2.0, 0.0;

    // Sanity: confirm the straight line start->goal really would cross the
    // obstacle, so this test is actually exercising obstacle avoidance and
    // not a target that never needed it.
    const double midpoint_distance_to_obstacle = ((start + goal) / 2.0 - obstacle_center).norm();
    ASSERT_LT(midpoint_distance_to_obstacle, obstacle_radius)
        << "test setup is wrong if the straight line doesn't already cross the obstacle";

    const auto path = planner.planPath(start, goal);
    for (const auto& point : path) {
        EXPECT_GT((point - obstacle_center).norm(), obstacle_radius - 1e-6)
            << "a waypoint landed inside the injected obstacle - RRT* failed to avoid it";
    }
}

// jointLimitMarginCostFn is the "optimization" hook (see rrt_star_path_
// planner.hpp): validity (IsStateValidFn) says what's ALLOWED, this says
// what's EXPENSIVE among allowed states. Test the function's own behavior
// directly first - independent of the randomized search - before trusting
// it inside a full planPath() call.
TEST(JointLimitMarginCostFn, PenalizesNearLimitMoreThanCenter) {
    Eigen::VectorXd lower(1), upper(1);
    lower << -1.0;
    upper << 1.0;
    const auto cost_fn = path_planner::jointLimitMarginCostFn(lower, upper, /*weight=*/1.0, /*margin_scale=*/0.1);

    Eigen::VectorXd from(1), to_center(1), to_near_limit(1);
    from << 0.0;
    to_center << 0.1;       // far from either limit
    to_near_limit << 0.99;  // 0.01 from the upper limit

    // Same base Euclidean distance from `from` is NOT required here - the
    // point is that the near-limit state costs strictly more than plain
    // Euclidean distance would predict, because of the added penalty.
    const double cost_center = cost_fn(from, to_center);
    const double cost_near_limit = cost_fn(from, to_near_limit);
    const double euclidean_near_limit = path_planner::euclideanEdgeCost(from, to_near_limit);

    EXPECT_GT(cost_near_limit, euclidean_near_limit)
        << "a near-limit destination must cost more than plain distance - that's the whole point "
           "of injecting this cost function instead of leaving the default";
    EXPECT_GT(cost_near_limit - euclidean_near_limit, cost_center - path_planner::euclideanEdgeCost(from, to_center))
        << "the penalty itself must be larger closer to the limit, not a flat additive constant";
}

TEST(JointLimitMarginCostFn, ZeroWeightMatchesPlainEuclideanCost) {
    Eigen::VectorXd lower(2), upper(2);
    lower << -1.0, -1.0;
    upper << 1.0, 1.0;
    const auto cost_fn = path_planner::jointLimitMarginCostFn(lower, upper, /*weight=*/0.0, 0.1);

    Eigen::VectorXd from(2), to(2);
    from << 0.0, 0.0;
    to << 0.95, -0.95;  // right next to two limits at once

    EXPECT_NEAR(cost_fn(from, to), path_planner::euclideanEdgeCost(from, to), 1e-12)
        << "weight=0 must be a true no-op, not just 'small effect' - otherwise turning this off "
           "isn't really turning it off";
}

// Correctness regression: plugging in a custom EdgeCostFn must not break
// the search itself (still reaches the goal, still respects bounds) - the
// cost function changes which valid path is preferred, never whether one
// is found at all.
TEST(RrtStarPathPlanner, WorksCorrectlyWithJointLimitMarginCost) {
    Eigen::VectorXd lower(3), upper(3);
    lower << -2.0, -2.0, -2.0;
    upper << 2.0, 2.0, 2.0;

    RrtStarConfig config = makeConfig(3, -2.0, 2.0);
    RrtStarPathPlanner planner(config, path_planner::alwaysValid,
                                 path_planner::jointLimitMarginCostFn(lower, upper, /*weight=*/2.0, 0.2));

    Eigen::VectorXd start(3), goal(3);
    start << -1.0, -1.0, -1.0;
    goal << 1.0, 1.0, 1.0;

    const auto path = planner.planPath(start, goal);
    ASSERT_GE(path.size(), 2u);
    EXPECT_TRUE(path.front().isApprox(start));
    EXPECT_TRUE(path.back().isApprox(goal));
    for (const auto& point : path) {
        for (int i = 0; i < point.size(); ++i) {
            EXPECT_GE(point[i], lower[i] - 1e-9);
            EXPECT_LE(point[i], upper[i] + 1e-9);
        }
    }
}

TEST(RrtStarPathPlanner, DeterministicGivenSameSeed) {
    RrtStarConfig config = makeConfig(3, -2.0, 2.0);
    config.seed = 42u;

    Eigen::VectorXd start(3), goal(3);
    start << 0.0, 0.0, 0.0;
    goal << 1.5, -1.0, 0.5;

    RrtStarPathPlanner planner_a(config);
    RrtStarPathPlanner planner_b(config);

    const auto path_a = planner_a.planPath(start, goal);
    const auto path_b = planner_b.planPath(start, goal);

    ASSERT_EQ(path_a.size(), path_b.size());
    for (std::size_t i = 0; i < path_a.size(); ++i) {
        EXPECT_TRUE(path_a[i].isApprox(path_b[i]))
            << "same seed must produce the same path - this is what keeps this test suite "
               "non-flaky despite the algorithm being randomized";
    }
}
