// Regression tests scoped to the trajectory_planner package alone: no
// kinematics/motion_planner dependency. See test_motion_planner.cpp for the
// TrapezoidalTrajectoryGenerator test that exercises it through the full
// pipeline instead (it needs a real FK-derived path, so it lives there).
#include <gtest/gtest.h>

#include <algorithm>

#include "trajectory_planner/factory/trajectory_generator_factory.hpp"

// TrapezoidalTrajectoryGenerator used one scalar max_velocity shared by
// every joint - fine only when every joint has similar speed headroom
// relative to how far it travels in a given move. Here joint 1 travels
// only 0.1 rad while joint 0 travels 1.0 rad, but joint 1's own
// velocity_max (0.05 rad/s) is far more restrictive relative to its small
// share of the move than joint 0's (10 rad/s) - so running the whole move
// at a shared max_velocity of 2.0 rad/s (of the arc-length parameter)
// pushes joint 1 well past its own limit. Feeding joint_velocity_max in
// lets the generator find joint 1 as the bottleneck and slow the whole
// move down to respect it, exactly like a real controller synchronizing
// multiple joints to whichever one is slowest for a given move.
TEST(TrapezoidalTrajectoryGenerator, PerJointVelocityBottleneckIsRespected) {
    const std::vector<Eigen::VectorXd> path = {(Eigen::VectorXd(2) << 0.0, 0.0).finished(),
                                                 (Eigen::VectorXd(2) << 1.0, 0.1).finished()};

    trajectory_planner::MotionConstraints constraints;
    constraints.max_velocity = 2.0;
    constraints.max_acceleration = 5.0;

    auto generator_naive = trajectory_planner::createTrajectoryGenerator("trapezoidal");
    const auto naive_trajectory = generator_naive->generateTrajectory(path, constraints);
    double naive_joint1_peak = 0.0;
    for (const auto& point : naive_trajectory) {
        naive_joint1_peak = std::max(naive_joint1_peak, std::abs(point.velocity[1]));
    }
    EXPECT_GT(naive_joint1_peak, 0.05)
        << "test setup is wrong if the naive scalar profile doesn't already violate joint 1's limit";

    constraints.joint_velocity_max = {10.0, 0.05};
    auto generator_bottlenecked = trajectory_planner::createTrajectoryGenerator("trapezoidal");
    const auto bottlenecked_trajectory = generator_bottlenecked->generateTrajectory(path, constraints);
    double bottlenecked_joint1_peak = 0.0;
    for (const auto& point : bottlenecked_trajectory) {
        bottlenecked_joint1_peak = std::max(bottlenecked_joint1_peak, std::abs(point.velocity[1]));
    }
    EXPECT_LE(bottlenecked_joint1_peak, 0.05 + 1e-6)
        << "joint 1 exceeded its own velocity_max (0.05) - bottleneck detection failed to slow the move down";

    EXPECT_GT(bottlenecked_trajectory.back().time_from_start, naive_trajectory.back().time_from_start)
        << "respecting the bottleneck joint should take longer than ignoring it";
}
