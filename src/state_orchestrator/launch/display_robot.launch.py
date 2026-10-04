"""Bring up the full live robot pipeline, visualized in RViz, with the
Tkinter teleop UI to drive it.

Which robot to display is config-driven, same as everything else in this
workspace: the `robot_xacro` launch argument's default is read straight
from system_config.yaml's robot_description_file -> that robot's own
kinematics.urdf_path, the exact same chain hw_interface_node/
robot_node already follow to pick their kinematics URDF - so
"just launch normally" shows the same robot the rest of the pipeline is
actually running, with no separate manual step. Override it for one run
without editing the config (`... robot_xacro:=/path/to/other.urdf.xacro`).

Pipeline: robot_ui (or anything else) -> /user_command -> robot_node
-> /hw_command -> hw_interface_node -> /hw_status -> joint_state_bridge_node
-> /joint_states -> robot_state_publisher -> TF -> RViz. All topic names
are the same ones used everywhere else in the stack - nothing is renamed
for visualization's sake.

Which backend produces /hw_status is a config-driven choice, same as
everything else in this workspace: system_config.yaml's existing
backend.digital_twin_method already names "mujoco" as a recognized value
(alongside "dummy") - set it there and this launch file starts
mujoco_digital_twin_node (real MuJoCo physics, see that package) instead
of hw_interface_node (the C++ kinematic-only dummy backend), with zero
other change to the pipeline: robot_node/robot_ui only know about
/hw_command and /hw_status, never which process is on the other end. The
`backend` launch argument overrides the config file's value for one run
without editing it (`... backend:=mujoco`); its own default is read from
system_config.yaml so "just launch normally" keeps doing whatever the
config file already says.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_system_config(robot_config_share):
    system_config_path = os.path.join(robot_config_share, "config", "system_config.yaml")
    with open(system_config_path) as f:
        return yaml.safe_load(f), system_config_path


def _configured_digital_twin_method(robot_config_share):
    root, _ = _load_system_config(robot_config_share)
    return root.get("backend", {}).get("digital_twin_method", "dummy")


def _resolve_package_path(path):
    """Mirrors kinematics_factory.cpp's resolvePackagePath: "package://
    <pkg>/<rel>" -> that package's installed share dir + "/" + rel; any
    other string (a plain filesystem path) is returned unchanged."""
    prefix = "package://"
    if not path.startswith(prefix):
        return path
    package_name, _, relative_path = path[len(prefix):].partition("/")
    return os.path.join(get_package_share_directory(package_name), relative_path)


def _configured_urdf_path(robot_config_share, fallback):
    """The same robot_description_file -> kinematics.urdf_path chain
    hw_interface_node/robot_node already follow (see
    robot_config.cpp's loadSystemConfig/loadRobotDescription) - falls back
    to `fallback` if system_config.yaml or the robot yaml it points to
    doesn't have a urdf_path (e.g. a robot only meant for the analytical
    strategy's link_lengths, not kdl) rather than failing the whole launch.
    """
    try:
        root, system_config_path = _load_system_config(robot_config_share)
        robot_description_file = root["robot_description_file"]
        robot_path = robot_description_file
        if not os.path.isabs(robot_path):
            robot_path = os.path.join(os.path.dirname(system_config_path), robot_path)
        with open(robot_path) as f:
            robot_root = yaml.safe_load(f)
        urdf_path = robot_root["kinematics"]["urdf_path"]
        return _resolve_package_path(urdf_path)
    except (KeyError, FileNotFoundError, TypeError):
        return fallback


def generate_launch_description():
    robot_description_share = get_package_share_directory("robot_description")
    state_orchestrator_share = get_package_share_directory("state_orchestrator")
    robot_config_share = get_package_share_directory("robot_config")

    robot_xacro_arg = DeclareLaunchArgument(
        "robot_xacro",
        default_value=_configured_urdf_path(
            robot_config_share, fallback=os.path.join(robot_description_share, "urdf", "fusybots_six_dof.urdf.xacro")),
        description="Path to the robot's urdf.xacro file. Defaults to whatever system_config.yaml's "
                     "robot_description_file -> kinematics.urdf_path already says.",
    )
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value=_configured_digital_twin_method(robot_config_share),
        description="Which backend publishes /hw_status: 'dummy' (hw_interface_node, kinematic-only) "
                     "or 'mujoco' (mujoco_digital_twin_node, real simulated physics). Defaults to "
                     "whatever system_config.yaml's backend.digital_twin_method already says.",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("robot_xacro")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    hw_interface_node = Node(
        package="hw_interface", executable="hw_interface_node",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("backend"), "dummy")),
    )
    mujoco_digital_twin_node = Node(
        package="mujoco_digital_twin", executable="mujoco_digital_twin_node",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("backend"), "mujoco")),
    )
    robot_node = Node(package="state_orchestrator", executable="robot_node")
    joint_state_bridge_node = Node(package="state_orchestrator", executable="joint_state_bridge_node")
    robot_ui_node = Node(package="robot_ui", executable="robot_ui_node")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(state_orchestrator_share, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        robot_xacro_arg,
        backend_arg,
        robot_state_publisher_node,
        hw_interface_node,
        mujoco_digital_twin_node,
        robot_node,
        joint_state_bridge_node,
        robot_ui_node,
        rviz_node,
    ])
