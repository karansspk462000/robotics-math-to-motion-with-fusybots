from setuptools import find_packages, setup

package_name = "robot_ui"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="karan",
    maintainer_email="karansspk@gmail.com",
    description=(
        "Tkinter (ttkbootstrap) teleop/monitor UI for the robot stack: sends UserCommand targets "
        "and displays live hw_status/motion_status, auto-configured from system_config.yaml."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_ui_node = robot_ui.robot_ui_node:main",
        ],
    },
)
