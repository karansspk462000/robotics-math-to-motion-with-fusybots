#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "math_utils/pose/quaternion_conversions.hpp"

namespace math_utils {

// Eigen 4x4 homogeneous transform -> [x, y, z, qx, qy, qz, qw]. Lets a
// Cartesian pose travel through the generic Eigen::VectorXd-based
// IPathPlanner/IPathInterpolator interfaces (built for joint vectors, but
// just as usable for poses as long as whatever interpolates them knows to
// slerp the quaternion block instead of lerping it - see
// path_interpolator::CartesianLinearPathInterpolator).
inline Eigen::VectorXd packPoseToVector7(const Eigen::Matrix4d& pose) {
    Eigen::Isometry3d isometry;
    isometry.matrix() = pose;
    const Eigen::Quaterniond quat = toNormalizedQuaternion(isometry.rotation());

    Eigen::VectorXd v(7);
    v.head<3>() = isometry.translation();
    v[3] = quat.x();
    v[4] = quat.y();
    v[5] = quat.z();
    v[6] = quat.w();
    return v;
}

// Inverse of packPoseToVector7.
inline Eigen::Matrix4d unpackVector7ToPose(const Eigen::VectorXd& v) {
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.translation() = v.head<3>();
    Eigen::Quaterniond quat(v[6], v[3], v[4], v[5]);  // (w, x, y, z)
    quat.normalize();
    isometry.linear() = quat.toRotationMatrix();
    return isometry.matrix();
}

// Eigen 4x4 homogeneous transform -> [x, y, z, roll, pitch, yaw] (6
// values, not the 16 homogeneous-matrix elements) - for reporting a
// Cartesian pose to something that wants a human/UI-facing pose, not a
// generic interpolatable vector (see packPoseToVector7 above for that
// case). Uses the same R = Rz(yaw)*Ry(pitch)*Rx(roll) Tait-Bryan
// convention as robot_ui/geometry.py's pose_to_flat_matrix/
// flat_matrix_to_pose - keep both in sync, or the UI's Cartesian fields
// and this would silently mean different angles for the same rotation.
inline Eigen::VectorXd packPoseToVector6Rpy(const Eigen::Matrix4d& pose) {
    const Eigen::Matrix3d r = pose.block<3, 3>(0, 0);

    const double pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
    double roll = 0.0;
    double yaw = 0.0;
    if (std::abs(std::cos(pitch)) > 1e-6) {
        roll = std::atan2(r(2, 1), r(2, 2));
        yaw = std::atan2(r(1, 0), r(0, 0));
    } else {
        // Gimbal lock (pitch = +-90deg): roll/yaw aren't independently
        // recoverable, only their sum/difference is - same convention
        // robot_ui/geometry.py's flat_matrix_to_pose falls back to
        // (yaw = 0, solve roll alone) in this case.
        roll = std::atan2(-r(1, 2), r(1, 1));
    }

    Eigen::VectorXd v(6);
    v[0] = pose(0, 3);
    v[1] = pose(1, 3);
    v[2] = pose(2, 3);
    v[3] = roll;
    v[4] = pitch;
    v[5] = yaw;
    return v;
}

}  // namespace math_utils
