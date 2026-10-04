#include "state_orchestrator/robot_node.hpp"

#include "math_utils/eigen/eigen_conversions.hpp"
#include "math_utils/pose/pose_conversions.hpp"

namespace state_orchestrator {

namespace {

std::string toControlModeString(robot_config::ControlMode mode) {
    switch (mode) {
        case robot_config::ControlMode::kPosition: return "position";
        case robot_config::ControlMode::kVelocity: return "velocity";
        case robot_config::ControlMode::kTorque: return "torque";
    }
    return "position";
}

}  // namespace

RobotNode::RobotNode(std::unique_ptr<motion_planner::IMotionPlanner> motion_planner,
                                    std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics,
                                    std::shared_ptr<kinematics::InverseKinematics> inverse_kinematics,
                                    const robot_config::SystemConfig& config)
    : rclcpp::Node("robot_node"),
      motion_planner_(std::move(motion_planner)),
      forward_kinematics_(std::move(forward_kinematics)),
      inverse_kinematics_(std::move(inverse_kinematics)),
      control_mode_(config.control_mode),
      state_machine_(config.backend_mode, config.hw_status_timeout_sec, config.divergence_warn_threshold) {
    joint_velocity_max_.reserve(config.robot.joint_limits.size());
    for (const auto& limits : config.robot.joint_limits) {
        joint_velocity_max_.push_back(limits.velocity_max);
    }

    const rclcpp::QoS reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const robot_config::TopicNames& topics = config.topics;

    hw_status_sub_ = create_subscription<custom_robot_msgs::msg::HwStatus>(
        topics.hw_status, reliable_qos,
        std::bind(&RobotNode::hwStatusCallback, this, std::placeholders::_1));

    if (config.backend_mode == robot_config::BackendMode::kBoth) {
        digital_twin_status_sub_ = create_subscription<custom_robot_msgs::msg::HwStatus>(
            topics.digital_twin_status, reliable_qos,
            std::bind(&RobotNode::digitalTwinStatusCallback, this, std::placeholders::_1));
    }

    user_command_sub_ = create_subscription<custom_robot_msgs::msg::UserCommand>(
        topics.user_command, reliable_qos,
        std::bind(&RobotNode::userCommandCallback, this, std::placeholders::_1));

    hw_command_pub_ = create_publisher<custom_robot_msgs::msg::HwCommand>(topics.hw_command, reliable_qos);
    motion_status_pub_ =
        create_publisher<custom_robot_msgs::msg::MotionStatus>(topics.motion_status, reliable_qos);

    const auto playback_period = std::chrono::duration<double>(1.0 / config.trajectory_playback_rate_hz);
    const auto state_machine_period = std::chrono::duration<double>(1.0 / config.state_machine_poll_rate_hz);
    playback_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(playback_period),
        std::bind(&RobotNode::trajectoryPlaybackTimerCallback, this));
    state_machine_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(state_machine_period),
        std::bind(&RobotNode::stateMachineTimerCallback, this));

    state_machine_.onSoftwareUp();
    logStateChangeIfAny();
}

void RobotNode::logStateChangeIfAny() {
    if (state_machine_.consumeStateChanged()) {
        RCLCPP_INFO(get_logger(), "Robot state -> %s", toString(state_machine_.state()).c_str());
    }
}

void RobotNode::hwStatusCallback(const custom_robot_msgs::msg::HwStatus::SharedPtr msg) {
    const Eigen::VectorXd position = math_utils::toEigenVector(msg->joint_positions);
    const Eigen::VectorXd velocity = math_utils::toEigenVector(msg->joint_velocity);

    custom_robot_msgs::msg::MotionStatus motion_status;
    motion_status.joint_position = math_utils::toStdVector(position);
    motion_status.joint_velocity = math_utils::toStdVector(velocity);
    try {
        const Eigen::Matrix4d cartesian_pose = forward_kinematics_->solve(position).cart_pose;
        motion_status.cartesian_pose = math_utils::toStdVector(math_utils::packPoseToVector6Rpy(cartesian_pose));

        if (velocity.size() == position.size()) {
            const Eigen::MatrixXd jacobian = inverse_kinematics_->computeJacobian(position);
            const Eigen::VectorXd cartesian_velocity = jacobian * velocity;
            motion_status.cartesian_velocity = math_utils::toStdVector(cartesian_velocity);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Forward kinematics for motion_status failed: %s",
                               e.what());
    }
    motion_status_pub_->publish(motion_status);

    std::lock_guard<std::mutex> lock(state_mutex_);
    last_joint_position_ = position;
    state_machine_.onHwStatusReceived(now().seconds(), position);
    logStateChangeIfAny();
}

void RobotNode::digitalTwinStatusCallback(const custom_robot_msgs::msg::HwStatus::SharedPtr msg) {
    const Eigen::VectorXd position = math_utils::toEigenVector(msg->joint_positions);

    std::lock_guard<std::mutex> lock(state_mutex_);
    state_machine_.onDigitalTwinStatusReceived(position);
    if (state_machine_.divergenceWarning()) {
        RCLCPP_WARN(get_logger(),
                    "hw_status and digital_twin_status have diverged: max |delta| = %.4f rad "
                    "(threshold %.4f) - reporting hardware as ground truth",
                    state_machine_.lastDivergence(), state_machine_.divergenceThreshold());
    }
}

void RobotNode::stateMachineTimerCallback() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool was_faulted = state_machine_.isFaulted();
    state_machine_.checkTimeout(now().seconds());
    if (!was_faulted && state_machine_.isFaulted()) {
        RCLCPP_ERROR(get_logger(),
                     "hw_status timed out - hardware link considered lost. Holding last known "
                     "position and rejecting new motion commands until hw_status resumes.");
        active_trajectory_.clear();
        active_trajectory_index_ = 0;
    }
    logStateChangeIfAny();
}

void RobotNode::userCommandCallback(const custom_robot_msgs::msg::UserCommand::SharedPtr msg) {
    motion_planner::MotionTarget target;
    trajectory_planner::MotionConstraints constraints;
    constraints.max_velocity = msg->max_velocity;
    constraints.max_acceleration = msg->max_acceleration;
    constraints.joint_velocity_max = joint_velocity_max_;

    Eigen::VectorXd current_joint_position;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!state_machine_.isReady()) {
            RCLCPP_WARN(get_logger(), "Ignoring motion command: robot state is %s, not ready",
                        toString(state_machine_.state()).c_str());
            return;
        }
        current_joint_position = last_joint_position_;
    }

    if (msg->planning_space == "cartesian") {
        target.planning_space = motion_planner::PlanningSpace::kCartesian;
        if (msg->target_cartesian_pose.size() != 16) {
            RCLCPP_ERROR(get_logger(),
                         "Ignoring motion command: target_cartesian_pose must have 16 elements "
                         "(row-major 4x4 transform), got %zu",
                         msg->target_cartesian_pose.size());
            return;
        }
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                target.cartesian_pose(row, col) = msg->target_cartesian_pose[row * 4 + col];
            }
        }
    } else {
        target.planning_space = motion_planner::PlanningSpace::kJoint;
        target.joint_position = math_utils::toEigenVector(msg->target_joint_position);
    }

    trajectory_planner::Trajectory trajectory;
    try {
        trajectory = motion_planner_->plan(current_joint_position, target, constraints);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Motion planning failed: %s", e.what());
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    // Re-check readiness here, not just before planning: plan() can take a
    // while, and on a single-threaded executor nothing else can run in
    // between anyway - but if this node is ever moved to a
    // MultiThreadedExecutor, the robot could fault mid-plan, and without
    // this second check a plan computed against a now-stale "ready"
    // assumption would silently overwrite the fault handling that already
    // cleared active_trajectory_.
    if (!state_machine_.isReady()) {
        RCLCPP_WARN(get_logger(), "Discarding motion plan: robot state changed to %s while planning",
                    toString(state_machine_.state()).c_str());
        return;
    }
    active_trajectory_ = std::move(trajectory);
    active_trajectory_index_ = 0;
}

custom_robot_msgs::msg::HwCommand RobotNode::buildHoldCommand(const Eigen::VectorXd& position) const {
    custom_robot_msgs::msg::HwCommand hw_command;
    hw_command.control_mode = toControlModeString(robot_config::ControlMode::kPosition);
    hw_command.joint_position = math_utils::toStdVector(position);
    hw_command.joint_velocity = math_utils::toStdVector(Eigen::VectorXd::Zero(position.size()));
    return hw_command;
}

void RobotNode::trajectoryPlaybackTimerCallback() {
    custom_robot_msgs::msg::HwCommand hw_command;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (state_machine_.isFaulted()) {
            // Hold at the last known good position rather than send
            // nothing (silence could look like "no command needed") or
            // keep advancing a trajectory planned against a robot state
            // we can no longer confirm.
            if (state_machine_.latchedPosition().size() == 0) {
                return;
            }
            hw_command = buildHoldCommand(state_machine_.latchedPosition());
        } else if (active_trajectory_index_ >= active_trajectory_.size()) {
            // No trajectory in flight: keep publishing hw_command as a
            // hold at the robot's current position instead of publishing
            // nothing - a downstream consumer should never have to treat
            // "no hw_command" as meaning "hold", only an explicit hold
            // command should mean that.
            if (last_joint_position_.size() == 0) {
                return;
            }
            hw_command = buildHoldCommand(last_joint_position_);
        } else {
            const trajectory_planner::TrajectoryPoint& point = active_trajectory_[active_trajectory_index_];
            hw_command.control_mode = toControlModeString(control_mode_);
            hw_command.joint_position = math_utils::toStdVector(point.position);
            hw_command.joint_velocity = math_utils::toStdVector(point.velocity);
            ++active_trajectory_index_;
        }
        // publish() happens once, below, after the lock releases in every
        // branch - not while still holding state_mutex_ (previously two of
        // the three branches published from inside the lock, the third
        // didn't; kept the lock held longer than necessary for no reason).
    }

    hw_command_pub_->publish(hw_command);
}

}  // namespace state_orchestrator
