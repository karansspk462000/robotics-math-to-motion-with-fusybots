"""Reads the same system_config.yaml (and the robot description it points
to) that the C++ nodes and robot_ui read - independently re-implementing
the same minimal schema rather than depending on either of those packages,
matching this workspace's existing precedent (robot_ui.config does the
same thing independently of robot_config.cpp; see that file's docstring).
Only the handful of fields this node actually needs are parsed.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory


class Topics:
    def __init__(self, data):
        data = data or {}
        self.hw_status = data.get("hw_status", "/hw_status")
        self.hw_command = data.get("hw_command", "/hw_command")


class MujocoTwinConfig:
    def __init__(self, system_config_path):
        with open(system_config_path) as f:
            root = yaml.safe_load(f)

        self.topics = Topics(root.get("topics"))

        timing = root.get("timing", {})
        self.hw_status_publish_rate_hz = float(timing.get("hw_status_publish_rate_hz", 30.0))
        self.mujoco_physics_timestep_sec = float(timing.get("mujoco_physics_timestep_sec", 0.002))

        robot_description_file = root["robot_description_file"]
        robot_path = robot_description_file
        if not os.path.isabs(robot_path):
            robot_path = os.path.join(os.path.dirname(system_config_path), robot_path)
        with open(robot_path) as f:
            robot_root = yaml.safe_load(f)

        self.robot_name = robot_root.get("name", "robot")
        self.num_joints = int(robot_root["num_joints"])
        self.home_joint_position = [float(v) for v in robot_root.get("home_joint_position",
                                                                       [0.0] * self.num_joints)]


def load_default_config():
    share_dir = get_package_share_directory("robot_config")
    system_config_path = os.path.join(share_dir, "config", "system_config.yaml")
    return MujocoTwinConfig(system_config_path)
