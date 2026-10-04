"""rclpy node the GUI talks to. Runs its own executor in a background
thread; the GUI polls its plain attributes (guarded by a lock) from a
Tkinter `after()` callback on the main thread instead of touching widgets
from ROS callbacks directly - Tkinter itself is not thread-safe, only the
data hand-off needs to be.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from custom_robot_msgs.msg import HwStatus, MotionStatus, UserCommand


class RobotUiBridge(Node):
    def __init__(self, topics):
        super().__init__("robot_ui_node")
        self._topics = topics
        self._lock = threading.Lock()

        self.hw_status = None
        self.hw_status_time = 0.0
        self.motion_status = None
        self.digital_twin_status = None
        self.digital_twin_status_time = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self._user_command_pub = self.create_publisher(UserCommand, topics.user_command, qos)
        self.create_subscription(HwStatus, topics.hw_status, self._on_hw_status, qos)
        self.create_subscription(MotionStatus, topics.motion_status, self._on_motion_status, qos)
        self.create_subscription(HwStatus, topics.digital_twin_status, self._on_digital_twin_status, qos)

    def _on_hw_status(self, msg):
        with self._lock:
            self.hw_status = msg
            self.hw_status_time = time.monotonic()

    def _on_motion_status(self, msg):
        with self._lock:
            self.motion_status = msg

    def _on_digital_twin_status(self, msg):
        with self._lock:
            self.digital_twin_status = msg
            self.digital_twin_status_time = time.monotonic()

    def snapshot(self):
        """Thread-safe copy of the latest received state, for the GUI to read."""
        with self._lock:
            return {
                "hw_status": self.hw_status,
                "hw_status_age": time.monotonic() - self.hw_status_time if self.hw_status_time else None,
                "motion_status": self.motion_status,
                "digital_twin_status": self.digital_twin_status,
                "digital_twin_status_age": (
                    time.monotonic() - self.digital_twin_status_time if self.digital_twin_status_time else None
                ),
            }

    def send_user_command(self, planning_space, target_joint_position, target_cartesian_pose,
                           max_velocity, max_acceleration):
        msg = UserCommand()
        msg.planning_space = planning_space
        msg.target_joint_position = [float(v) for v in target_joint_position]
        msg.target_cartesian_pose = [float(v) for v in target_cartesian_pose]
        msg.max_velocity = float(max_velocity)
        msg.max_acceleration = float(max_acceleration)
        self._user_command_pub.publish(msg)


def spin_in_background(node):
    """Starts rclpy.spin(node) on a daemon thread; returns the thread."""
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return thread
