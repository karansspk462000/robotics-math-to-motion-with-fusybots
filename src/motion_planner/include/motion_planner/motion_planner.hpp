#pragma once

#include <functional>
#include <memory>

#include "kinematics/context/kinematics.hpp"
#include "kinematics/interface/kinematics_interface.hpp"
#include "motion_planner/motion_planner_interface.hpp"
#include "path_interpolator/interface/path_interpolator_interface.hpp"
#include "path_planner/interface/path_planner_interface.hpp"
#include "trajectory_planner/interface/trajectory_generator_interface.hpp"

namespace motion_planner {

// Concrete IMotionPlanner. Pure C++, no ROS dependency: it is a thin
// composition root that wires kinematics + path_planner + path_interpolator
// + trajectory_planner together - it implements none of those algorithms
// itself. Each stage is injected as an interface, so it can be unit-tested
// or reused from any transport (ROS node, simulator, CLI) without
// modification, and any stage can be swapped independently of the others.
//
// Cartesian targets are interpolated in Cartesian space, not joint space:
// cartesian_path_interpolator_ operates on packed poses (see
// math_utils::packPoseToVector7) all the way through, and IK is applied
// once per interpolated waypoint only at the very end, right before
// trajectory generation. This is what makes a Cartesian target actually
// move the end-effector in a straight line through Cartesian space -
// solving IK once up front and then interpolating in joint space, as an
// earlier version of this class did, does not: a straight line in joint
// space is not a straight line in Cartesian space. path_planner_ plays no
// role in the Cartesian branch (there's no Cartesian-space obstacle model
// in this workspace, so nothing for a planner to decide there) - it's used
// only by planJointSpace, where it's now expected to be a real, obstacle-
// aware Strategy (see path_planner::RrtStarPathPlanner), not a trivial
// straight-line stand-in.
class MotionPlanner : public IMotionPlanner {
public:
    // is_state_valid is optional (defaults to empty, meaning "don't check"
    // - preserves existing behavior/call sites that don't pass one) and is
    // used only by planCartesianSpace, to pre-check current/target before
    // planning: unlike planJointSpace (which routes through path_planner_,
    // itself collision-aware if given a real checker), the Cartesian
    // branch never calls path_planner_ (see class doc comment above), so
    // without this hook it would have zero collision awareness even when
    // one exists for joint-space moves. This deliberately checks only the
    // two endpoints, not the dense interpolated path in between - a
    // dimension-appropriate Cartesian-space planner would be needed for
    // that, which does not exist (see path_planner::RrtStarPathPlanner's
    // known limitation).
    using IsJointStateValidFn = std::function<bool(const Eigen::VectorXd&)>;

    MotionPlanner(std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics,
                  std::shared_ptr<kinematics::InverseKinematics> inverse_kinematics,
                  std::unique_ptr<path_planner::IPathPlanner> path_planner,
                  std::unique_ptr<path_interpolator::IPathInterpolator> path_interpolator,
                  std::unique_ptr<path_interpolator::IPathInterpolator> cartesian_path_interpolator,
                  std::unique_ptr<trajectory_planner::ITrajectoryGenerator> trajectory_generator,
                  IsJointStateValidFn is_state_valid = nullptr);

    trajectory_planner::Trajectory plan(const Eigen::VectorXd& current_joint_position,
                                          const MotionTarget& target,
                                          const trajectory_planner::MotionConstraints& constraints) override;

private:
    trajectory_planner::Trajectory planJointSpace(const Eigen::VectorXd& current_joint_position,
                                                    const Eigen::VectorXd& target_joint_position,
                                                    const trajectory_planner::MotionConstraints& constraints);
    trajectory_planner::Trajectory planCartesianSpace(const Eigen::VectorXd& current_joint_position,
                                                        const Eigen::Matrix4d& target_cartesian_pose,
                                                        const trajectory_planner::MotionConstraints& constraints);

    std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics_;
    std::shared_ptr<kinematics::InverseKinematics> inverse_kinematics_;
    std::unique_ptr<path_planner::IPathPlanner> path_planner_;
    std::unique_ptr<path_interpolator::IPathInterpolator> path_interpolator_;
    std::unique_ptr<path_interpolator::IPathInterpolator> cartesian_path_interpolator_;
    std::unique_ptr<trajectory_planner::ITrajectoryGenerator> trajectory_generator_;
    IsJointStateValidFn is_state_valid_;
};

}  // namespace motion_planner
