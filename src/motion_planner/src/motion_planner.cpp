#include "motion_planner/motion_planner.hpp"

#include <stdexcept>

#include "math_utils/pose/pose_conversions.hpp"

namespace motion_planner {

MotionPlanner::MotionPlanner(std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics,
                              std::shared_ptr<kinematics::InverseKinematics> inverse_kinematics,
                              std::unique_ptr<path_planner::IPathPlanner> path_planner,
                              std::unique_ptr<path_interpolator::IPathInterpolator> path_interpolator,
                              std::unique_ptr<path_interpolator::IPathInterpolator> cartesian_path_interpolator,
                              std::unique_ptr<trajectory_planner::ITrajectoryGenerator> trajectory_generator,
                              IsJointStateValidFn is_state_valid)
    : forward_kinematics_(std::move(forward_kinematics)),
      inverse_kinematics_(std::move(inverse_kinematics)),
      path_planner_(std::move(path_planner)),
      path_interpolator_(std::move(path_interpolator)),
      cartesian_path_interpolator_(std::move(cartesian_path_interpolator)),
      trajectory_generator_(std::move(trajectory_generator)),
      is_state_valid_(std::move(is_state_valid)) {
    if (!path_planner_ || !path_interpolator_ || !cartesian_path_interpolator_ || !trajectory_generator_) {
        throw std::invalid_argument(
            "MotionPlanner: path_planner/path_interpolator/cartesian_path_interpolator/"
            "trajectory_generator must not be null");
    }
}

trajectory_planner::Trajectory MotionPlanner::plan(
    const Eigen::VectorXd& current_joint_position, const MotionTarget& target,
    const trajectory_planner::MotionConstraints& constraints) {
    if (target.planning_space == PlanningSpace::kJoint) {
        return planJointSpace(current_joint_position, target.joint_position, constraints);
    }
    return planCartesianSpace(current_joint_position, target.cartesian_pose, constraints);
}

trajectory_planner::Trajectory MotionPlanner::planJointSpace(
    const Eigen::VectorXd& current_joint_position, const Eigen::VectorXd& target_joint_position,
    const trajectory_planner::MotionConstraints& constraints) {
    const std::vector<Eigen::VectorXd> sparse_path =
        path_planner_->planPath(current_joint_position, target_joint_position);
    const std::vector<Eigen::VectorXd> dense_path = path_interpolator_->interpolatePath(sparse_path);
    return trajectory_generator_->generateTrajectory(dense_path, constraints);
}

trajectory_planner::Trajectory MotionPlanner::planCartesianSpace(
    const Eigen::VectorXd& current_joint_position, const Eigen::Matrix4d& target_cartesian_pose,
    const trajectory_planner::MotionConstraints& constraints) {
    if (!forward_kinematics_ || !inverse_kinematics_) {
        throw std::runtime_error(
            "MotionPlanner: Cartesian target requested but no forward_kinematics/inverse_kinematics was provided");
    }
    if (is_state_valid_ && !is_state_valid_(current_joint_position)) {
        throw std::runtime_error("MotionPlanner: current joint position is not a valid state - refusing to plan");
    }

    // Interpolate in Cartesian space first, so the end-effector actually
    // follows a straight Cartesian line rather than whatever curve joint-
    // space interpolation of a single IK solution happens to produce.
    // No IPathPlanner involved here (unlike planJointSpace) - there is no
    // Cartesian-space obstacle model in this workspace, so there is
    // nothing for a "planner" to decide; the sparse path is always just
    // the two endpoints, built directly rather than routed through
    // path_planner_ (which is joint-space-only - see
    // RrtStarPathPlanner's class doc comment).
    const Eigen::Matrix4d current_cartesian_pose = forward_kinematics_->solve(current_joint_position).cart_pose;

    const std::vector<Eigen::VectorXd> sparse_cartesian_path = {
        math_utils::packPoseToVector7(current_cartesian_pose),
        math_utils::packPoseToVector7(target_cartesian_pose)};
    const std::vector<Eigen::VectorXd> dense_cartesian_path =
        cartesian_path_interpolator_->interpolatePath(sparse_cartesian_path);

    // IK, once per interpolated waypoint, each seeded by the previous
    // waypoint's solution so the resulting joint path stays continuous
    // (no branch jumps between adjacent, nearly-identical Cartesian points).
    std::vector<Eigen::VectorXd> joint_path;
    joint_path.reserve(dense_cartesian_path.size());
    Eigen::VectorXd seed = current_joint_position;
    for (const Eigen::VectorXd& packed_pose : dense_cartesian_path) {
        kinematics::CartesianPosition waypoint_pose;
        waypoint_pose.cart_pose = math_utils::unpackVector7ToPose(packed_pose);
        seed = inverse_kinematics_->solve(waypoint_pose, seed);
        joint_path.push_back(seed);
    }

    // Pre-check only the two endpoints (see class doc comment/constructor
    // comment) - not every point on dense_cartesian_path/joint_path, which
    // would need a real Cartesian-space obstacle model this workspace
    // doesn't have.
    if (is_state_valid_ && !joint_path.empty() && !is_state_valid_(joint_path.back())) {
        throw std::runtime_error("MotionPlanner: target pose's IK solution is not a valid state - refusing to plan");
    }

    return trajectory_generator_->generateTrajectory(joint_path, constraints);
}

}  // namespace motion_planner
