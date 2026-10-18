"""Entry point: load robot_config + the MuJoCo model, spin the ROS node on
a background thread, run MuJoCo's physics-step/viewer loop on the main
thread (see node.py's docstring for why the threads are split this way).

Usage: run via the installed console script (see setup.py):
    ros2 run mujoco_digital_twin mujoco_digital_twin_node
Requires hw_interface_node to NOT also be running for the same
/hw_command and /hw_status topics (this node is an alternative to it, not
an addition alongside it) - launch this instead of hw_interface_node,
then robot_node/robot_ui on top exactly as usual; they only know
they're talking to /hw_command and /hw_status, not which process is on
the other end.

If no display is available (or MUJOCO_DIGITAL_TWIN_HEADLESS=1 is set),
falls back to a headless physics loop with no viewer window - the ROS
topics behave identically either way, only the 3D visualization differs.
"""
import os
import tempfile
import threading
import time

import mujoco
import rclpy
from ament_index_python.packages import get_package_share_directory

from mujoco_digital_twin.config import load_default_config
from mujoco_digital_twin.node import MujocoTwinNode, spin_in_background

MESH_DIR_PLACEHOLDER = "MESH_DIR_PLACEHOLDER"


def _model_path(robot_name: str) -> str:
    """Returns a real filesystem path to a loadable MJCF for `robot_name`.

    The shipped models/<robot>.xml has <compiler meshdir="MESH_DIR_PLACEHOLDER">
    (see that file's header comment) because MuJoCo only understands real
    filesystem paths, not ROS package:// URIs, and robot_description's
    install location isn't known until ament_index resolves it here at
    runtime. Substitutes the real path into a temp copy rather than
    editing the installed model file in place.
    """
    mujoco_twin_share = get_package_share_directory("mujoco_digital_twin")
    template_path = os.path.join(mujoco_twin_share, "models", f"{robot_name}.xml")
    if not os.path.isfile(template_path):
        raise FileNotFoundError(
            f"No MuJoCo model for robot '{robot_name}' at {template_path} - this package "
            f"doesn't ship a MuJoCo model for any robot yet, so one must be added under "
            f"models/{robot_name}.xml (with the same <compiler meshdir=\"{MESH_DIR_PLACEHOLDER}\"> "
            f"placeholder this function substitutes at runtime) before this backend can be used.")

    with open(template_path) as f:
        xml = f.read()

    if MESH_DIR_PLACEHOLDER in xml:
        robot_description_share = get_package_share_directory("robot_description")
        mesh_dir = os.path.join(robot_description_share, "meshes", robot_name)
        xml = xml.replace(MESH_DIR_PLACEHOLDER, mesh_dir)
        resolved_fd, resolved_path = tempfile.mkstemp(prefix=f"{robot_name}_", suffix=".xml")
        with os.fdopen(resolved_fd, "w") as f:
            f.write(xml)
        return resolved_path

    return template_path


def _run_headless(model, data, lock):
    period = model.opt.timestep
    next_step = time.monotonic()
    while rclpy.ok():
        with lock:
            mujoco.mj_step(model, data)
        next_step += period
        sleep_for = next_step - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)


def _run_with_viewer(model, data, lock):
    import mujoco.viewer

    with mujoco.viewer.launch_passive(model, data) as viewer:
        period = model.opt.timestep
        next_step = time.monotonic()
        while viewer.is_running() and rclpy.ok():
            with lock:
                mujoco.mj_step(model, data)
            viewer.sync()
            next_step += period
            sleep_for = next_step - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)


def main():
    config = load_default_config()
    model_path = _model_path(config.robot_name)

    model = mujoco.MjModel.from_xml_path(model_path)
    model.opt.timestep = config.mujoco_physics_timestep_sec
    data = mujoco.MjData(model)

    if len(config.home_joint_position) == model.nq:
        data.qpos[:] = config.home_joint_position
        data.ctrl[:] = config.home_joint_position
        mujoco.mj_forward(model, data)

    lock = threading.Lock()

    rclpy.init()
    node = MujocoTwinNode(model, data, config, lock)
    spin_in_background(node)

    try:
        if os.environ.get("MUJOCO_DIGITAL_TWIN_HEADLESS") == "1":
            _run_headless(model, data, lock)
        else:
            try:
                _run_with_viewer(model, data, lock)
            except Exception as exc:  # noqa: BLE001 - no display, GL failure, etc: fall back, don't crash
                node.get_logger().warning(f"MuJoCo viewer unavailable ({exc}) - running headless instead")
                _run_headless(model, data, lock)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
