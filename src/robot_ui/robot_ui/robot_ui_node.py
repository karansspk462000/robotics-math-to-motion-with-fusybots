"""Entry point: rclpy.init, load config, spin the ROS bridge on a
background thread, run the Tkinter GUI on the main thread.
"""
import rclpy

from robot_ui.config import load_default_robot_config
from robot_ui.gui import RobotUiApp
from robot_ui.ros_bridge import RobotUiBridge, spin_in_background


def main():
    config = load_default_robot_config()

    rclpy.init()
    bridge = RobotUiBridge(config.topics)
    spin_in_background(bridge)

    app = RobotUiApp(config, bridge)
    try:
        app.run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
