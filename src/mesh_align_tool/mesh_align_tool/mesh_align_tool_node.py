"""Interactive drag-to-align tool for vla_manipulator.urdf.xacro's per-link
mesh visuals - the RViz-native equivalent of nudging an object's transform
by hand in Blender.

For every link that has a real STL mesh visual, this publishes one
InteractiveMarker (RViz's native 3D drag gizmo: translate arrows + rotate
rings) parented to that link's own TF frame, showing the actual mesh, and
starting exactly at that mesh's current <origin> from the xacro. Drag it
in RViz; on mouse-up, this node prints a ready-to-paste
`<origin xyz="..." rpy="...">` line for that link straight to the
terminal - copy it into vla_manipulator.urdf.xacro once it looks right.

Because the marker's header.frame_id IS the link's own TF frame, the
feedback pose RViz reports is already exactly what URDF's <origin> wants
(a transform from the link frame to the mesh) - no extra math needed
beyond quaternion -> rpy.

Usage:
    ros2 launch state_orchestrator display_robot.launch.py \\
        robot_xacro:=<path-to>/vla_manipulator.urdf.xacro
    ros2 run mesh_align_tool mesh_align_tool_node
Then in RViz: Add -> By display type -> InteractiveMarkers, set its
Update Topic to /mesh_align_tool/update, and drag away.
"""
import math

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from scipy.spatial.transform import Rotation

# (link_name, mesh_filename, initial_xyz, initial_rpy) - copied verbatim
# from vla_manipulator.urdf.xacro's current <visual><origin> per link.
# Keep this in sync if the xacro changes independently of this tool.
LINKS = [
    ("l0", "u_type_robot_waist_bracket.stl", (0.0, 0.0, 0.04), (0.0, 0.0, 0.0)),
    ("l1", "mg996r_servo.stl", (0.0, 0.0, 0.03), (0.0, 0.0, 0.0)),
    ("l2", "long_u_type_servo_bracket.stl", (0.0525, 0.0, 0.0), (0.0, math.pi / 2, 0.0)),
    ("l3", "mg996r_servo.stl", (-0.0525, 0.0, 0.0), (0.0, math.pi / 2, 0.0)),
    ("l4", "multi_functional_servo_bracket.stl", (0.0, 0.0, -0.025), (0.0, 0.0, 0.0)),
    ("l5", "long_u_type_servo_bracket.stl", (-0.025, 0.0, 0.0), (0.0, math.pi / 2, 0.0)),
    ("l6", "mg996r_servo.stl", (0.0, 0.0, 0.03), (0.0, 0.0, 0.0)),
    ("le", "l_type_servo_bracket.stl", (0.0, 0.0, 0.03), (0.0, 0.0, 0.0)),
    ("le_1", "gripper_palm.stl", (0.0, 0.0, 0.025), (0.0, 0.0, 0.0)),
]

MESH_PACKAGE_PREFIX = "package://robot_description/meshes/vla_manipulator/"


def _quat_from_rpy(roll, pitch, yaw):
    x, y, z, w = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    return x, y, z, w


def _rpy_from_quat(x, y, z, w):
    return Rotation.from_quat([x, y, z, w]).as_euler("xyz")


def _make_axis_control(name, x, y, z, w, mode):
    control = InteractiveMarkerControl()
    control.name = name
    control.orientation.x = x
    control.orientation.y = y
    control.orientation.z = z
    control.orientation.w = w
    control.interaction_mode = mode
    return control


class MeshAlignToolNode(Node):
    def __init__(self):
        super().__init__("mesh_align_tool_node")
        self._server = InteractiveMarkerServer(self, "mesh_align_tool")

        for link_name, mesh_file, xyz, rpy in LINKS:
            self._add_marker(link_name, mesh_file, xyz, rpy)
        self._server.applyChanges()

        self.get_logger().info(
            f"mesh_align_tool_node: published {len(LINKS)} draggable mesh markers, one per link. "
            f"Add an 'InteractiveMarkers' display in RViz with update topic "
            f"/mesh_align_tool/update, drag a mesh, and watch this node's log for the resulting "
            f"<origin> line to paste into vla_manipulator.urdf.xacro.")

    def _add_marker(self, link_name, mesh_file, xyz, rpy):
        marker = InteractiveMarker()
        marker.header.frame_id = link_name
        marker.name = link_name
        marker.description = f"{link_name}: {mesh_file}"
        marker.scale = 0.08

        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        qx, qy, qz, qw = _quat_from_rpy(*rpy)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        # The visible, draggable mesh itself.
        mesh_marker = Marker()
        mesh_marker.type = Marker.MESH_RESOURCE
        mesh_marker.mesh_resource = MESH_PACKAGE_PREFIX + mesh_file
        mesh_marker.mesh_use_embedded_materials = False
        mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = 0.001
        mesh_marker.color.r, mesh_marker.color.g, mesh_marker.color.b, mesh_marker.color.a = (1.0, 0.65, 0.0, 0.9)

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(mesh_marker)
        marker.controls.append(visual_control)

        # Standard 6-DOF drag gizmo: 3 translate arrows + 3 rotate rings.
        marker.controls.append(_make_axis_control("move_x", 1.0, 0.0, 0.0, 1.0,
                                                     InteractiveMarkerControl.MOVE_AXIS))
        marker.controls.append(_make_axis_control("rotate_x", 1.0, 0.0, 0.0, 1.0,
                                                     InteractiveMarkerControl.ROTATE_AXIS))
        marker.controls.append(_make_axis_control("move_y", 0.0, 0.0, 1.0, 1.0,
                                                     InteractiveMarkerControl.MOVE_AXIS))
        marker.controls.append(_make_axis_control("rotate_y", 0.0, 0.0, 1.0, 1.0,
                                                     InteractiveMarkerControl.ROTATE_AXIS))
        marker.controls.append(_make_axis_control("move_z", 0.0, 1.0, 0.0, 1.0,
                                                     InteractiveMarkerControl.MOVE_AXIS))
        marker.controls.append(_make_axis_control("rotate_z", 0.0, 1.0, 0.0, 1.0,
                                                     InteractiveMarkerControl.ROTATE_AXIS))

        self._server.insert(marker, feedback_callback=self._on_feedback)

    def _on_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        p = feedback.pose.position
        o = feedback.pose.orientation
        roll, pitch, yaw = _rpy_from_quat(o.x, o.y, o.z, o.w)
        self.get_logger().info(
            f"[{feedback.marker_name}] new origin -> "
            f'<origin xyz="{p.x:.4f} {p.y:.4f} {p.z:.4f}" rpy="{roll:.4f} {pitch:.4f} {yaw:.4f}"/>')


def main():
    rclpy.init()
    node = MeshAlignToolNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
