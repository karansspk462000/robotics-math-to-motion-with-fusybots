"""Plain-Python pose -> flat 4x4 (row-major, list[16]) helper for
UserCommand's target_cartesian_pose (still a full homogeneous transform -
a planning target legitimately needs exact orientation, unlike
MotionStatus's cartesian_pose, which is [x,y,z,roll,pitch,yaw] straight
from math_utils::packPoseToVector6Rpy, so no matrix->rpy conversion is
needed on this side for that one anymore). No Eigen/numpy dependency
needed for this - it's just a few trig functions.
"""
import math


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def pose_to_flat_matrix(x, y, z, roll, pitch, yaw):
    """Returns a row-major flattened list[16] homogeneous transform."""
    r = rpy_to_matrix(roll, pitch, yaw)
    return [
        r[0][0], r[0][1], r[0][2], x,
        r[1][0], r[1][1], r[1][2], y,
        r[2][0], r[2][1], r[2][2], z,
        0.0, 0.0, 0.0, 1.0,
    ]
