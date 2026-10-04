#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace robot_config {

enum class ControlMode { kPosition, kVelocity, kTorque };

// Which robot backend(s) hw_interface_node should talk to. write() always
// goes to whichever backend(s) are active; in kBoth both backends get the
// exact same command (see robot_hw_interface::DualRobotInterface).
enum class BackendMode { kHardwareOnly, kDigitalTwinOnly, kBoth };

struct JointLimits {
    // Must match the corresponding URDF <joint> name exactly - used as-is
    // for sensor_msgs/JointState.name by the RViz joint-state bridge, so
    // TF/robot_state_publisher can match it back to the right URDF joint.
    std::string name;
    double position_min = 0.0;
    double position_max = 0.0;
    double velocity_max = 0.0;
    double effort_max = 0.0;
};

struct PidGains {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
};

// ROS topic names used by the orchestrator node(s). Centralized here so
// renaming a topic is a one-line YAML edit instead of a code change/rebuild.
struct TopicNames {
    std::string hw_status = "/hw_status";
    // Operator/UI-facing input: a target (joint or Cartesian) plus motion
    // constraints, published by whatever is commanding the robot (a
    // teleop UI, a script, ...) and consumed by robot_node.
    std::string user_command = "/user_command";
    std::string hw_command = "/hw_command";
    // Only published when backend.mode is "both" - the digital twin's
    // reading, kept separate from hw_status (which is always hardware's
    // reading) so the two can be compared for divergence.
    std::string digital_twin_status = "/digital_twin_status";
    // Operator/UI-facing output: current joint state plus its Cartesian
    // pose (computed via forward kinematics), published by
    // robot_node for anything that wants to display live robot
    // state without redoing FK itself.
    std::string motion_status = "/motion_status";
};

// Low-level, per-robot description: everything that depends on which
// physical robot this is (arm, humanoid, ...). kinematics_params is left as
// raw YAML because its shape depends on which kinematics_method the system
// config selects (a DH table, PoE screw axes, a URDF path for KDL/Pinocchio,
// ...) - only the chosen kinematics factory knows how to interpret it, so
// this struct stays agnostic to that.
struct RobotDescription {
    std::string name;
    int num_joints = 0;
    int dof = 0;
    std::vector<JointLimits> joint_limits;
    std::vector<PidGains> joint_pid_gains;
    YAML::Node kinematics_params;
    // Reported by the digital-twin/dummy-hardware backends before any
    // command is sent, and what robot_ui's "Send Home" button targets.
    // Defaults to all-zero (resized in loadSystemConfig) if the robot yaml
    // does not set it, but every all-zero pose on this project's
    // fusybots_six_dof[_spherical_wrist] robots sits on a shoulder/wrist
    // singularity - set this explicitly to a non-singular pose instead.
    std::vector<double> home_joint_position;
};

// High-level, robot-agnostic system config: which strategy to use at each
// stage of the pipeline, which control mode to command, and which
// RobotDescription file to load.
struct SystemConfig {
    std::string robot_description_file;
    std::string kinematics_method;
    std::string ik_method;
    std::string path_planner_method;
    std::string path_interpolator_method;
    // Used only for Cartesian targets (interpolates packed poses - lerp
    // position, slerp orientation - rather than joint vectors).
    std::string cartesian_path_interpolator_method;
    // trajectory_planner does not synthesize positions of its own - it
    // only picks already-existing points from the interpolators' dense
    // output at the right time - so these must be dense enough to cover
    // the whole eventual trajectory, or motion visibly stair-steps with an
    // inconsistent velocity signal. See path_interpolator_factory.hpp.
    int path_interpolator_points_per_segment = 1000;
    int cartesian_path_interpolator_points_per_segment = 1000;
    std::string trajectory_planner_method;
    ControlMode control_mode = ControlMode::kPosition;
    TopicNames topics;

    // Which robot backend(s) to run and their method-specific params (e.g.
    // "arduino" needs a serial_port; "dummy" needs nothing). Read by
    // hw_interface_node, not by robot_node.
    BackendMode backend_mode = BackendMode::kDigitalTwinOnly;
    std::string hardware_method;
    YAML::Node hardware_params;
    std::string digital_twin_method;
    YAML::Node digital_twin_params;

    // Read by hw_interface_node: how often it polls the backend and
    // publishes hw_status/digital_twin_status. Independent from
    // trajectory_playback_rate_hz because this one also drives how many ROS
    // callbacks/sec any Python subscriber (e.g. robot_ui) has to service -
    // too high a rate here visibly competes with a Tkinter GUI's event loop
    // for the GIL. 100 Hz is far more than a human needs to see move
    // smoothly; lower this if a UI feels laggy.
    double hw_status_publish_rate_hz = 30.0;

    // Read by robot_node: its /hw_command publish rate. Also used (as
    // 1/rate) as the trajectory generator's sample time_step - these two
    // must always match, or real-world playback speed drifts from the
    // commanded max_velocity/max_acceleration. See system_config.yaml's
    // timing: block comment.
    double trajectory_playback_rate_hz = 100.0;
    // Read by robot_node: how often it checks hw_status for staleness.
    double state_machine_poll_rate_hz = 10.0;

    // Read by robot_node's RobotStateMachine.
    double hw_status_timeout_sec = 0.5;
    double divergence_warn_threshold = 0.05;

    RobotDescription robot;
};

// Loads system_config_path, then resolves and loads the robot description
// file it points to (relative paths are resolved relative to
// system_config_path's directory). Throws std::runtime_error on any
// missing/malformed field.
SystemConfig loadSystemConfig(const std::string& system_config_path);

}  // namespace robot_config
