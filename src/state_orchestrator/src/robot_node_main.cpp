#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "collision_checking/planning_scene.hpp"
#include "kinematics/context/kinematics.hpp"
#include "kinematics/factory/kinematics_factory.hpp"
#include "motion_planner/motion_planner.hpp"
#include "path_interpolator/factory/path_interpolator_factory.hpp"
#include "path_planner/factory/path_planner_factory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_config/robot_config.hpp"
#include "state_orchestrator/robot_node.hpp"
#include "trajectory_planner/factory/trajectory_generator_factory.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Everything below is config-driven: swapping robots or swapping any
    // pipeline stage (kinematics method, path planner, interpolator,
    // trajectory generator, control mode) means editing YAML, not code -
    // this is what lets the same node run an arm today and a humanoid limb
    // later.
    const std::string robot_config_share_dir =
        ament_index_cpp::get_package_share_directory("robot_config");
    const std::string system_config_path = robot_config_share_dir + "/config/system_config.yaml";
    const robot_config::SystemConfig config = robot_config::loadSystemConfig(system_config_path);

    auto fk_strategy = kinematics::createFKStrategy(config.kinematics_method, config.robot.kinematics_params);
    auto ik_strategy = kinematics::createIKStrategy(config.ik_method, fk_strategy);
    auto inverse_kinematics = std::make_shared<kinematics::InverseKinematics>(std::move(ik_strategy));
    // Context wrapping the same fk_strategy instance ik_strategy was built
    // from above - shared (not moved) so RobotNode and
    // MotionPlanner can each hold their own shared_ptr to this one
    // Context, exactly like inverse_kinematics already does.
    auto forward_kinematics = std::make_shared<kinematics::ForwardKinematics>(fk_strategy);

    // Real sampling bounds for path_planner_method "rrt_star" - the only
    // path_planner_method, joint-space only (see RrtStarPathPlanner's
    // class doc comment for why Cartesian-space planning doesn't use this
    // at all).
    Eigen::VectorXd joint_lower(config.robot.joint_limits.size());
    Eigen::VectorXd joint_upper(config.robot.joint_limits.size());
    for (std::size_t i = 0; i < config.robot.joint_limits.size(); ++i) {
        joint_lower[static_cast<int>(i)] = config.robot.joint_limits[i].position_min;
        joint_upper[static_cast<int>(i)] = config.robot.joint_limits[i].position_max;
    }

    // The live, queryable model of "what's in the world" - empty today
    // (no obstacle-authoring format exists in system_config.yaml yet, and
    // no vision package is updating it - see the README's future-scope
    // list), so isStateValid() is trivially true for every state until
    // something actually calls addObject()/removeObject(). Shared (not
    // owned exclusively by one consumer) because both path_planner and
    // motion_planner need to observe the same live scene below, and a
    // future vision-fed updater will need to mutate the same instance
    // both of them are already holding.
    auto planning_scene = std::make_shared<collision_checking::PlanningScene>(forward_kinematics);

    // Wraps PlanningScene's method into the generic bool(VectorXd) shape
    // both path_planner::RrtStarPathPlanner and motion_planner::MotionPlanner
    // accept - this is the only place in the program that needs to know
    // both collision_checking AND path_planner/motion_planner exist;
    // neither of those packages depends on collision_checking directly.
    auto is_state_valid = [planning_scene](const Eigen::VectorXd& joint_positions) {
        return planning_scene->isStateValid(joint_positions);
    };

    // Optimization: prefer paths that keep some margin from the joint
    // limits over equally-short ones that don't, without rejecting
    // near-limit states outright (that's is_state_valid's job, not this
    // one's) - see jointLimitMarginCostFn's doc comment for the weight/
    // margin_scale tuning knobs, and for how a future singularity-avoidance
    // cost term would plug in the same way.
    auto edge_cost = path_planner::jointLimitMarginCostFn(joint_lower, joint_upper, /*weight=*/0.05);

    auto motion_planner = std::make_unique<motion_planner::MotionPlanner>(
        forward_kinematics,
        inverse_kinematics,
        path_planner::createPathPlanner(config.path_planner_method, joint_lower, joint_upper, is_state_valid,
                                          edge_cost),
        path_interpolator::createPathInterpolator(config.path_interpolator_method,
                                                    config.path_interpolator_points_per_segment),
        path_interpolator::createPathInterpolator(config.cartesian_path_interpolator_method,
                                                    config.cartesian_path_interpolator_points_per_segment),
        trajectory_planner::createTrajectoryGenerator(config.trajectory_planner_method,
                                                        1.0 / config.trajectory_playback_rate_hz),
        is_state_valid);

    auto node = std::make_shared<state_orchestrator::RobotNode>(std::move(motion_planner), forward_kinematics,
                                                                 inverse_kinematics, config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
