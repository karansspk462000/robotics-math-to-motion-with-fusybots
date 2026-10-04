"""Reads the same system_config.yaml (and the robot description it points
to) that the C++ nodes read, via robot_config's installed share directory -
so the UI is never out of sync with the actual running robot: point
system_config.yaml at a different robot and the UI's joint count, limits,
and topic names all follow automatically.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory


class Topics:
    def __init__(self, data):
        data = data or {}
        self.hw_status = data.get("hw_status", "/hw_status")
        self.user_command = data.get("user_command", "/user_command")
        self.hw_command = data.get("hw_command", "/hw_command")
        self.digital_twin_status = data.get("digital_twin_status", "/digital_twin_status")
        self.motion_status = data.get("motion_status", "/motion_status")


class Joint:
    def __init__(self, data):
        self.position_min = float(data.get("position_min", -3.14159))
        self.position_max = float(data.get("position_max", 3.14159))
        self.velocity_max = float(data.get("velocity_max", 1.0))
        self.effort_max = float(data.get("effort_max", 0.0))


class RobotConfig:
    def __init__(self, system_config_path):
        with open(system_config_path) as f:
            root = yaml.safe_load(f)

        self.topics = Topics(root.get("topics"))

        backend = root.get("backend", {})
        self.backend_mode = backend.get("mode", "digital_twin")

        motion_defaults = root.get("motion_defaults", {})
        self.default_velocity = float(motion_defaults.get("default_velocity", 1.0))
        self.default_acceleration = float(motion_defaults.get("default_acceleration", 1.0))

        robot_description_file = root["robot_description_file"]
        robot_path = robot_description_file
        if not os.path.isabs(robot_path):
            robot_path = os.path.join(os.path.dirname(system_config_path), robot_path)
        with open(robot_path) as f:
            robot_root = yaml.safe_load(f)

        self.robot_name = robot_root.get("name", "robot")
        self.num_joints = int(robot_root["num_joints"])
        self.joints = [Joint(j) for j in robot_root.get("joints", [])]
        self.home_joint_position = [float(v) for v in robot_root.get("home_joint_position",
                                                                       [0.0] * self.num_joints)]


def load_default_robot_config():
    share_dir = get_package_share_directory("robot_config")
    system_config_path = os.path.join(share_dir, "config", "system_config.yaml")
    return RobotConfig(system_config_path)
