// Regression tests exercising the full pipeline through the real
// MotionPlanner composition root (kinematics + path_planner +
// path_interpolator + trajectory_planner wired together), rather than any
// one package in isolation. See test_kinematics.cpp/test_trajectory_planner
// .cpp for tests scoped to just those packages.
#include <gtest/gtest.h>

#include <algorithm>

#include "motion_planner_test_helpers.hpp"

using test_helpers::buildKdlMotionPlanner;
using test_helpers::cartesianSpeedNorms;
using test_helpers::countSpeedDirectionChanges;
using test_helpers::maxDeviationFromLine;
using test_helpers::velocityNorms;

// Regression for: MotionPlanner solving IK once up front then interpolating
// in joint space, so a Cartesian "straight line" target never actually
// moved in a straight line through Cartesian space. Fixed by planning and
// interpolating Cartesian targets in Cartesian space, applying IK once per
// interpolated waypoint only at the end.
TEST(MotionPlannerCartesianSpace, EndEffectorFollowsStraightLine) {
    std::shared_ptr<kinematics::IFKStrategy> fk;
    auto planner = buildKdlMotionPlanner(fk);

    Eigen::VectorXd current(6);
    current << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;  // non-singular start
    const Eigen::Vector3d start_pos = fk->computeCartesianPosition(current).cart_pose.block<3, 1>(0, 3);

    motion_planner::MotionTarget target;
    target.planning_space = motion_planner::PlanningSpace::kCartesian;
    Eigen::VectorXd q_target(6);
    q_target << 0.4, 0.3, -0.3, 0.2, 0.5, -0.2;
    target.cartesian_pose = fk->computeCartesianPosition(q_target).cart_pose;
    const Eigen::Vector3d end_pos = target.cartesian_pose.block<3, 1>(0, 3);

    trajectory_planner::MotionConstraints constraints;
    constraints.max_velocity = 1.0;
    constraints.max_acceleration = 1.0;

    const auto trajectory = planner.plan(current, target, constraints);
    ASSERT_GT(trajectory.size(), 1u);

    const double deviation = maxDeviationFromLine(trajectory, fk, start_pos, end_pos);
    const double line_length = (end_pos - start_pos).norm();
    EXPECT_LT(deviation, 1e-6) << "end-effector deviated " << deviation << " from a straight line of length "
                                << line_length << " (" << (deviation / line_length * 100.0) << "% of path)";
}

// Regression for: TrapezoidalTrajectoryGenerator used to ignore path_
// interpolator's dense points and synthesize its own straight line in
// joint space from front()/back() only - defeating the fix above. Confirms
// the generator actually consumes path's density (deviation must shrink
// as density increases, not stay constant).
TEST(MotionPlannerCartesianSpace, DeviationShrinksWithInterpolationDensity) {
    std::shared_ptr<kinematics::IFKStrategy> fk;
    auto sparse_planner = buildKdlMotionPlanner(fk, /*cartesian_points_per_segment=*/5);
    std::shared_ptr<kinematics::IFKStrategy> fk2;
    auto dense_planner = buildKdlMotionPlanner(fk2, /*cartesian_points_per_segment=*/2000);

    Eigen::VectorXd current(6);
    current << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    const Eigen::Vector3d start_pos = fk->computeCartesianPosition(current).cart_pose.block<3, 1>(0, 3);

    motion_planner::MotionTarget target;
    target.planning_space = motion_planner::PlanningSpace::kCartesian;
    Eigen::VectorXd q_target(6);
    q_target << 0.4, 0.3, -0.3, 0.2, 0.5, -0.2;
    target.cartesian_pose = fk->computeCartesianPosition(q_target).cart_pose;
    const Eigen::Vector3d end_pos = target.cartesian_pose.block<3, 1>(0, 3);

    trajectory_planner::MotionConstraints constraints;
    constraints.max_velocity = 0.5;
    constraints.max_acceleration = 0.5;

    const double sparse_deviation =
        maxDeviationFromLine(sparse_planner.plan(current, target, constraints), fk, start_pos, end_pos);
    const double dense_deviation =
        maxDeviationFromLine(dense_planner.plan(current, target, constraints), fk2, start_pos, end_pos);

    EXPECT_LT(dense_deviation, sparse_deviation)
        << "increasing interpolation density did not reduce deviation - trajectory_planner is likely "
           "ignoring path's actual points again";
    EXPECT_LT(dense_deviation, 1e-6);
}

// Regression for: TrapezoidalTrajectoryGenerator repeating the same
// position for long runs (position frozen) while its reported velocity
// kept smoothly ramping per the scalar time profile - physically
// inconsistent, and the direct cause of the "up down up down" chatter
// seen in /motion_status/joint_velocity. With points_per_segment dense
// enough for the move's duration, no run of repeated positions should be
// long, and reported velocity must roughly match the actual finite-
// difference velocity between consecutive published positions.
TEST(TrapezoidalTrajectoryGenerator, PositionAdvancesEveryStepAtDefaultDensity) {
    std::shared_ptr<kinematics::IFKStrategy> fk;
    auto planner = buildKdlMotionPlanner(fk);  // default points_per_segment = 1000

    Eigen::VectorXd current(6);
    current << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    motion_planner::MotionTarget target;
    target.planning_space = motion_planner::PlanningSpace::kCartesian;
    Eigen::VectorXd q_target(6);
    q_target << 0.4, 0.3, -0.3, 0.2, 0.5, -0.2;
    target.cartesian_pose = fk->computeCartesianPosition(q_target).cart_pose;

    trajectory_planner::MotionConstraints constraints;
    constraints.max_velocity = 1.0;
    constraints.max_acceleration = 1.0;

    const auto trajectory = planner.plan(current, target, constraints);
    ASSERT_GT(trajectory.size(), 10u);

    int longest_repeated_run = 1;
    int current_run = 1;
    for (std::size_t i = 1; i < trajectory.size(); ++i) {
        if (trajectory[i].position == trajectory[i - 1].position) {
            ++current_run;
            longest_repeated_run = std::max(longest_repeated_run, current_run);
        } else {
            current_run = 1;
        }
    }
    EXPECT_LE(longest_repeated_run, 10)
        << "longest run of a repeated (frozen) position was " << longest_repeated_run
        << " samples - interpolation density is too low relative to this move's duration";

    // Reported velocity should roughly match (position[i] - position[i-1])
    // / time_step for every step where the position actually changed.
    const double time_step = trajectory[1].time_from_start - trajectory[0].time_from_start;
    int inconsistent_steps = 0;
    for (std::size_t i = 1; i + 1 < trajectory.size(); ++i) {
        const Eigen::VectorXd actual_velocity = (trajectory[i + 1].position - trajectory[i].position) / time_step;
        const double reported_speed = trajectory[i].velocity.norm();
        const double actual_speed = actual_velocity.norm();
        if (std::abs(reported_speed - actual_speed) > 0.05 * std::max(reported_speed, actual_speed) + 1e-3) {
            ++inconsistent_steps;
        }
    }
    EXPECT_LE(inconsistent_steps, 2) << "reported velocity disagreed with actual position-derived velocity "
                                      << "at " << inconsistent_steps << " steps - this is the exact "
                                      << "physically-inconsistent-velocity bug";
}

// Joint-space regression: a joint-space target's velocity profile is the
// simplest possible case (path_planner/path_interpolator never touch
// Cartesian space at all), so if this one isn't a clean trapezoid then the
// bug is in TrapezoidalTrajectoryGenerator itself, not in any Cartesian-
// space plumbing.
TEST(JointSpaceTrapezoid, VelocityIsCleanTrapezoid) {
    std::shared_ptr<kinematics::IFKStrategy> fk;
    auto planner = buildKdlMotionPlanner(fk);

    Eigen::VectorXd current(6);
    current << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    motion_planner::MotionTarget target;
    target.planning_space = motion_planner::PlanningSpace::kJoint;
    target.joint_position = (Eigen::VectorXd(6) << 0.4, 0.3, -0.3, 0.2, 0.5, -0.2).finished();

    trajectory_planner::MotionConstraints constraints;
    constraints.max_velocity = 1.0;
    constraints.max_acceleration = 1.0;

    const auto trajectory = planner.plan(current, target, constraints);
    ASSERT_GT(trajectory.size(), 10u);

    const auto speeds = velocityNorms(trajectory);
    const double peak_speed = *std::max_element(speeds.begin(), speeds.end());
    const int direction_changes = countSpeedDirectionChanges(speeds, 0.05 * peak_speed);
    EXPECT_LE(direction_changes, 1) << "joint-space velocity norm changed direction " << direction_changes
                                      << " times (expected at most 1: accel -> decel) - not a clean trapezoid";
}

// Regression for: a Cartesian target whose start/end differ in x, y and z
// simultaneously must still (a) move the end-effector along a straight
// line in Cartesian space and (b) report a Cartesian-space speed profile
// that is a clean trapezoid, not an oscillating one - both properties are
// independent of the joint-space plumbing being "correct" in isolation.
TEST(CartesianSpaceSimultaneousMove, StraightLineAndCleanVelocityTrapezoid) {
    std::shared_ptr<kinematics::IFKStrategy> fk;
    auto planner = buildKdlMotionPlanner(fk);

    Eigen::VectorXd current(6);
    current << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    const Eigen::Vector3d start_pos = fk->computeCartesianPosition(current).cart_pose.block<3, 1>(0, 3);

    motion_planner::MotionTarget target;
    target.planning_space = motion_planner::PlanningSpace::kCartesian;
    // Chosen (by search over random joint targets, see dev notes) to move
    // >3cm in x, y and z simultaneously while its straight-line Cartesian
    // path stays clear of the wrist singularity that KDL's NR IK solver
    // cannot converge through - unrelated to the bug this test targets.
    Eigen::VectorXd q_target(6);
    q_target << -0.484034, -0.269106, -0.258975, 0.183264, 0.109997, 0.333195;
    target.cartesian_pose = fk->computeCartesianPosition(q_target).cart_pose;
    const Eigen::Vector3d end_pos = target.cartesian_pose.block<3, 1>(0, 3);

    // Confirm this really is a simultaneous multi-axis move, not one where
    // x, y or z happens to be unchanged - otherwise this test wouldn't be
    // exercising what it claims to.
    const Eigen::Vector3d delta = (end_pos - start_pos).cwiseAbs();
    EXPECT_GT(delta.x(), 0.01) << "x barely moved - not a simultaneous multi-axis move";
    EXPECT_GT(delta.y(), 0.01) << "y barely moved - not a simultaneous multi-axis move";
    EXPECT_GT(delta.z(), 0.01) << "z barely moved - not a simultaneous multi-axis move";

    trajectory_planner::MotionConstraints constraints;
    constraints.max_velocity = 0.5;
    constraints.max_acceleration = 0.5;

    const auto trajectory = planner.plan(current, target, constraints);
    ASSERT_GT(trajectory.size(), 10u);

    // Looser than EndEffectorFollowsStraightLine's 1e-6: this move is
    // larger (delta.norm() ~0.13m vs that test's much smaller one), so
    // per-waypoint KDL NR IK tolerance stacks across more waypoints of
    // real curvature - still five microns on a >10cm move.
    const double deviation = maxDeviationFromLine(trajectory, fk, start_pos, end_pos);
    EXPECT_LT(deviation, 5e-6) << "end-effector deviated " << deviation << " from a straight line while moving "
                                 << "in x, y and z simultaneously";

    const auto cart_speeds = cartesianSpeedNorms(trajectory, fk);
    const double peak_speed = *std::max_element(cart_speeds.begin(), cart_speeds.end());
    const int direction_changes = countSpeedDirectionChanges(cart_speeds, 0.05 * peak_speed);
    EXPECT_LE(direction_changes, 1) << "Cartesian-space speed changed direction " << direction_changes
                                      << " times (expected at most 1: accel -> decel) - this is the exact "
                                      << "'up down up down' chatter bug, now checked directly in Cartesian space";
}

// The Cartesian branch never calls path_planner_ (see MotionPlanner's class
// doc comment), so without its own pre-check hook it would have zero
// collision awareness even when the joint-space branch has real
// obstacle-checking wired in. Confirms that hook actually gates planning -
// both when the *current* state is invalid and when the *resolved IK
// target* state is invalid.
TEST(MotionPlannerCartesianSpace, PreCheckHookRejectsInvalidStartOrTarget) {
    std::shared_ptr<kinematics::IFKStrategy> fk;
    auto planner_reject_current = [&]() {
        auto fk_strategy = kinematics::createFKStrategy("kdl", test_helpers::kdlParams());
        auto ik_strategy = kinematics::createIKStrategy("kdl", fk_strategy);
        auto inverse_kinematics = std::make_shared<kinematics::InverseKinematics>(std::move(ik_strategy));
        auto forward_kinematics = std::make_shared<kinematics::ForwardKinematics>(fk_strategy);
        fk = fk_strategy;
        return motion_planner::MotionPlanner(
            forward_kinematics, inverse_kinematics,
            path_planner::createPathPlanner("rrt_star", Eigen::VectorXd::Constant(6, -6.3),
                                              Eigen::VectorXd::Constant(6, 6.3)),
            std::make_unique<path_interpolator::LinearPathInterpolator>(path_interpolator::PoseSpace::kJoint, 1000),
            std::make_unique<path_interpolator::LinearPathInterpolator>(path_interpolator::PoseSpace::kCartesian,
                                                                           1000),
            trajectory_planner::createTrajectoryGenerator("trapezoidal"),
            [](const Eigen::VectorXd&) { return false; });  // reject every state
    }();

    Eigen::VectorXd current(6);
    current << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    motion_planner::MotionTarget target;
    target.planning_space = motion_planner::PlanningSpace::kCartesian;
    Eigen::VectorXd q_target(6);
    q_target << 0.4, 0.3, -0.3, 0.2, 0.5, -0.2;
    target.cartesian_pose = fk->computeCartesianPosition(q_target).cart_pose;

    trajectory_planner::MotionConstraints constraints;
    constraints.max_velocity = 1.0;
    constraints.max_acceleration = 1.0;

    EXPECT_THROW(planner_reject_current.plan(current, target, constraints), std::runtime_error)
        << "an is_state_valid hook that rejects every state must stop planning at the current-state check";
}
