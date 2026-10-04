from setuptools import find_packages, setup

package_name = "mesh_align_tool"

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
        "One-off interactive RViz tool for hand-tuning vla_manipulator's per-link mesh <origin> "
        "values by dragging a 6-DOF gizmo, instead of guessing offsets blind."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mesh_align_tool_node = mesh_align_tool.mesh_align_tool_node:main",
        ],
    },
)
