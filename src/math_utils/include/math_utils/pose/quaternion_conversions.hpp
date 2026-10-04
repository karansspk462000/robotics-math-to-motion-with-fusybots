#pragma once

#include <cmath>

#include <Eigen/Geometry>

namespace math_utils {

// A rotation matrix that has drifted slightly off SO(3) (accumulated
// floating-point error) produces a quaternion with norm != 1; several
// downstream consumers (KDL, ROS messages) assume unit quaternions and
// silently misbehave otherwise. Renormalize defensively before handing a
// quaternion across such a boundary.
inline Eigen::Quaterniond toNormalizedQuaternion(const Eigen::Matrix3d& rotation) {
    Eigen::Quaterniond quat(rotation);
    if (std::abs(quat.norm() - 1.0) > 1e-3) {
        quat.normalize();
    }
    return quat;
}

}  // namespace math_utils
