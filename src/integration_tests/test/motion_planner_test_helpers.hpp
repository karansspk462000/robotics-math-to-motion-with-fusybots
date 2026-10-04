#pragma once

// Shared helpers for motion_planner end-to-end regression tests: building a
// real MotionPlanner wired to the KDL kinematics strategy, and analyzing a
// resulting Trajectory for straight-line deviation / velocity-profile shape.
#include <algorithm>
#include <memory>
#include <vector>

#include "kinematics/context/kinematics.hpp"
#include "kinematics/factory/kinematics_factory.hpp"
#include "motion_planner/motion_planner.hpp"
#include "path_interpolator/factory/path_interpolator_factory.hpp"
#include "path_interpolator/strategy/linear_path_interpolator.hpp"
#include "path_planner/factory/path_planner_factory.hpp"
#include "robot_params.hpp"
#include "trajectory_planner/factory/trajectory_generator_factory.hpp"

namespace test_helpers {

inline motion_planner::MotionPlanner buildKdlMotionPlanner(std::shared_ptr<kinematics::IFKStrategy>& fk_out,
                                                             int cartesian_points_per_segment = 1000) {
    auto fk_strategy = kinematics::createFKStrategy("kdl", kdlParams());
    auto ik_strategy = kinematics::createIKStrategy("kdl", fk_strategy);
    auto inverse_kinematics = std::make_shared<kinematics::InverseKinematics>(std::move(ik_strategy));
    auto forward_kinematics = std::make_shared<kinematics::ForwardKinematics>(fk_strategy);
    fk_out = fk_strategy;

    // Matches fusybots_six_dof_spherical_wrist.yaml's real joint position_min/max -
    // path_planner_'s only remaining role is joint-space planning (see
    // motion_planner.hpp), so it needs real 6-dim bounds, not a
    // placeholder box.
    Eigen::VectorXd joint_lower(6), joint_upper(6);
    joint_lower << -6.28318, -1.5708, -1.5708, -1.5708, -3.14159, -6.28318;
    joint_upper << 6.28318, 1.5708, 1.5708, 1.5708, 3.14159, 6.28318;

    return motion_planner::MotionPlanner(
        forward_kinematics, inverse_kinematics,
        path_planner::createPathPlanner("rrt_star", joint_lower, joint_upper),
        std::make_unique<path_interpolator::LinearPathInterpolator>(path_interpolator::PoseSpace::kJoint, 1000),
        std::make_unique<path_interpolator::LinearPathInterpolator>(path_interpolator::PoseSpace::kCartesian,
                                                                       cartesian_points_per_segment),
        trajectory_planner::createTrajectoryGenerator("trapezoidal"));
}

inline double maxDeviationFromLine(const trajectory_planner::Trajectory& trajectory,
                                    const std::shared_ptr<kinematics::IFKStrategy>& fk,
                                    const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos) {
    const Eigen::Vector3d line_dir = (end_pos - start_pos).normalized();
    double max_deviation = 0.0;
    for (const auto& point : trajectory) {
        const Eigen::Vector3d pos = fk->computeCartesianPosition(point.position).cart_pose.block<3, 1>(0, 3);
        const double along = (pos - start_pos).dot(line_dir);
        const Eigen::Vector3d closest = start_pos + along * line_dir;
        max_deviation = std::max(max_deviation, (pos - closest).norm());
    }
    return max_deviation;
}

// A clean trapezoid's speed sequence has exactly one direction change
// (increasing during accel, decreasing during decel); the cruise phase's
// step-to-step speed delta is treated as noise and ignored via
// `noise_floor`. The "up down up down" chatter bug shows up here as many
// direction changes instead of at most one.
inline int countSpeedDirectionChanges(const std::vector<double>& speeds, double noise_floor) {
    int sign_changes = 0;
    int last_sign = 0;
    for (std::size_t i = 1; i < speeds.size(); ++i) {
        const double delta = speeds[i] - speeds[i - 1];
        if (std::abs(delta) < noise_floor) {
            continue;
        }
        const int sign = delta > 0.0 ? 1 : -1;
        if (last_sign != 0 && sign != last_sign) {
            ++sign_changes;
        }
        last_sign = sign;
    }
    return sign_changes;
}

inline std::vector<double> velocityNorms(const trajectory_planner::Trajectory& trajectory) {
    std::vector<double> speeds;
    speeds.reserve(trajectory.size());
    for (const auto& point : trajectory) {
        speeds.push_back(point.velocity.norm());
    }
    return speeds;
}

// Cartesian-space speed at each sample, computed from the actual FK'd
// positions of the trajectory (not from joint-space velocity), since a
// clean joint-space trapezoid does not, by itself, guarantee a clean
// Cartesian-space one (that's exactly the bug this whole redesign fixed).
inline std::vector<double> cartesianSpeedNorms(const trajectory_planner::Trajectory& trajectory,
                                                 const std::shared_ptr<kinematics::IFKStrategy>& fk) {
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(trajectory.size());
    for (const auto& point : trajectory) {
        positions.push_back(fk->computeCartesianPosition(point.position).cart_pose.block<3, 1>(0, 3));
    }
    std::vector<double> speeds;
    speeds.reserve(trajectory.size());
    speeds.push_back(0.0);
    for (std::size_t i = 1; i < trajectory.size(); ++i) {
        const double dt = trajectory[i].time_from_start - trajectory[i - 1].time_from_start;
        speeds.push_back(dt > 1e-9 ? (positions[i] - positions[i - 1]).norm() / dt : 0.0);
    }
    return speeds;
}

}  // namespace test_helpers
