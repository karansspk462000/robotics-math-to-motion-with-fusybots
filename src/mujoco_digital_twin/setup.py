import glob

from setuptools import find_packages, setup

package_name = "mujoco_digital_twin"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", glob.glob("mujoco_digital_twin/models/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="karan",
    maintainer_email="karansspk@gmail.com",
    description=(
        "MuJoCo-simulated digital twin: subscribes /hw_command, publishes /hw_status with real "
        "physics, as a drop-in replacement for hw_interface_node when you want simulated dynamics."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_digital_twin_node = mujoco_digital_twin.main:main",
        ],
    },
)
