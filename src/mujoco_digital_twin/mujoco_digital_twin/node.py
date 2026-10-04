"""rclpy node for the MuJoCo digital twin: a drop-in replacement for
hw_interface_node's ROS contract (same /hw_command subscription, same
/hw_status publisher, same QoS and publish-rate config field) when you
want real simulated physics instead of hw_interface's kinematic-only
dummy backends. It is a separate process, not a C++ IRobotInterface
plugged into hw_interface_node - see the package README for why (MuJoCo's
best-supported, most complete API is Python; this workspace's other
backends are native C++ classes in-process with hw_interface_node).

Threading model mirrors robot_ui's (see robot_ui/ros_bridge.py): this
node's callbacks (hwCommandCallback-equivalent, and the status-publish
timer) run wherever rclpy.spin() is called from - main.py spins this node
on a background thread and runs MuJoCo's physics-step + viewer loop on the
main thread (MuJoCo's own examples always pair viewer.sync() with the
physics step in one loop, so that loop owns the main thread here, exactly
the reverse of robot_ui where Tkinter's mainloop owns the main thread and
ROS spins in the background - same split, mirrored). `state_lock` is the
one lock shared between both threads, guarding `data.ctrl` (written here
from a ROS callback, read by the physics loop before every mj_step) and
`data.qpos`/`data.qvel` (written by the physics loop's mj_step, read here
when publishing HwStatus) - exactly the kind of genuinely-required lock
described for robot_ui's bridge, not the C++ side's defensive one, because
these two threads really do run concurrently.
"""
import threading

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from custom_robot_msgs.msg import HwCommand, HwStatus


class MujocoTwinNode(Node):
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config, state_lock: threading.Lock):
        super().__init__("mujoco_digital_twin_node")
        self._model = model
        self._data = data
        self._lock = state_lock
        self._num_joints = config.num_joints
        self._warned_unsupported_mode = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(HwCommand, config.topics.hw_command, self._on_hw_command, qos)
        self._hw_status_pub = self.create_publisher(HwStatus, config.topics.hw_status, qos)

        period_sec = 1.0 / config.hw_status_publish_rate_hz
        self.create_timer(period_sec, self._publish_status)

        self.get_logger().info(
            f"mujoco_digital_twin_node: {config.robot_name} ({config.num_joints} DOF), "
            f"publishing {config.topics.hw_status} at {config.hw_status_publish_rate_hz} Hz, "
            f"subscribed to {config.topics.hw_command}")

    def _on_hw_command(self, msg: HwCommand):
        if msg.control_mode != "position":
            if not self._warned_unsupported_mode:
                self.get_logger().warning(
                    f"mujoco_digital_twin_node only implements control_mode 'position' - ignoring a "
                    f"'{msg.control_mode}' command (this warning logs once)")
                self._warned_unsupported_mode = True
            return
        if len(msg.joint_position) != self._num_joints:
            self.get_logger().error(
                f"hw_command had {len(msg.joint_position)} joint_position values, expected "
                f"{self._num_joints} - ignoring")
            return

        with self._lock:
            self._data.ctrl[:] = msg.joint_position

    def _publish_status(self):
        status = HwStatus()
        with self._lock:
            status.joint_positions = [float(v) for v in self._data.qpos]
            status.joint_velocity = [float(v) for v in self._data.qvel]
        # MuJoCo's <position> actuators don't model motor current/torque sensing for
        # this simple twin - left as zero, same as the dummy hw/twin backends do for
        # whatever they don't model either.
        status.joint_torque = [0.0] * self._num_joints
        status.joint_current = [0.0] * self._num_joints
        self._hw_status_pub.publish(status)


def spin_in_background(node):
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return thread
