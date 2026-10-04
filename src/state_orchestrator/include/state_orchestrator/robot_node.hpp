#pragma once

#include <memory>
#include <mutex>

#include "custom_robot_msgs/msg/hw_command.hpp"
#include "custom_robot_msgs/msg/hw_status.hpp"
#include "custom_robot_msgs/msg/motion_status.hpp"
#include "custom_robot_msgs/msg/user_command.hpp"
#include "kinematics/context/kinematics.hpp"
#include "kinematics/interface/kinematics_interface.hpp"
#include "motion_planner/motion_planner_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_config/robot_config.hpp"
#include "state_orchestrator/state_manager/robot_state_machine.hpp"

namespace state_orchestrator {

// The robot's central orchestrator node: the ROS boundary for motion
// planning today, and the future home for hw_interface wiring and
// robot-state management as those get built out. motion_planner itself
// stays ROS-free; this class only does message translation, subscriptions/
// publishers, trajectory playback timing, and robot-state bookkeeping via
// RobotStateMachine (readiness, hw_status-loss latching, hw/twin
// divergence warnings).
class RobotNode : public rclcpp::Node {
public:
    RobotNode(std::unique_ptr<motion_planner::IMotionPlanner> motion_planner,
              std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics,
              std::shared_ptr<kinematics::InverseKinematics> inverse_kinematics,
              const robot_config::SystemConfig& config);

private:
    void hwStatusCallback(const custom_robot_msgs::msg::HwStatus::SharedPtr msg);
    void digitalTwinStatusCallback(const custom_robot_msgs::msg::HwStatus::SharedPtr msg);
    void userCommandCallback(const custom_robot_msgs::msg::UserCommand::SharedPtr msg);
    void trajectoryPlaybackTimerCallback();
    void stateMachineTimerCallback();
    void logStateChangeIfAny();
    custom_robot_msgs::msg::HwCommand buildHoldCommand(const Eigen::VectorXd& position) const;

    std::unique_ptr<motion_planner::IMotionPlanner> motion_planner_;
    std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics_;
    std::shared_ptr<kinematics::InverseKinematics> inverse_kinematics_;
    robot_config::ControlMode control_mode_;
    // One entry per joint (robot_config's JointLimits::velocity_max) - lets
    // TrapezoidalTrajectoryGenerator find whichever joint would be the
    // bottleneck for a given move instead of treating every joint as
    // equally fast. See trapezoidal_trajectory_generator.cpp's
    // effectiveMaxVelocity.
    std::vector<double> joint_velocity_max_;
    RobotStateMachine state_machine_;

    rclcpp::Subscription<custom_robot_msgs::msg::HwStatus>::SharedPtr hw_status_sub_;
    rclcpp::Subscription<custom_robot_msgs::msg::HwStatus>::SharedPtr digital_twin_status_sub_;
    rclcpp::Subscription<custom_robot_msgs::msg::UserCommand>::SharedPtr user_command_sub_;
    rclcpp::Publisher<custom_robot_msgs::msg::HwCommand>::SharedPtr hw_command_pub_;
    rclcpp::Publisher<custom_robot_msgs::msg::MotionStatus>::SharedPtr motion_status_pub_;
    rclcpp::TimerBase::SharedPtr playback_timer_;
    rclcpp::TimerBase::SharedPtr state_machine_timer_;

    std::mutex state_mutex_;
    Eigen::VectorXd last_joint_position_;
    trajectory_planner::Trajectory active_trajectory_;
    std::size_t active_trajectory_index_ = 0;
};

}  // namespace state_orchestrator
