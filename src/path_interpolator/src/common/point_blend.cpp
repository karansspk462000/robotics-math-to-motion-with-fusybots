#include "path_interpolator/common/point_blend.hpp"

#include <stdexcept>

#include <Eigen/Geometry>

namespace path_interpolator {

Eigen::VectorXd blendPoint(const Eigen::VectorXd& a, const Eigen::VectorXd& b, double t, PoseSpace space) {
    if (space == PoseSpace::kJoint) {
        return a + t * (b - a);
    }

    if (a.size() != 7 || b.size() != 7) {
        throw std::invalid_argument(
            "blendPoint: PoseSpace::kCartesian expects 7-element packed poses (x, y, z, qx, qy, qz, qw)");
    }

    const Eigen::Vector3d position = a.head<3>() + t * (b.head<3>() - a.head<3>());
    Eigen::Quaterniond quat_a(a[6], a[3], a[4], a[5]);
    Eigen::Quaterniond quat_b(b[6], b[3], b[4], b[5]);
    quat_a.normalize();
    quat_b.normalize();
    const Eigen::Quaterniond quat = quat_a.slerp(t, quat_b);

    Eigen::VectorXd point(7);
    point.head<3>() = position;
    point[3] = quat.x();
    point[4] = quat.y();
    point[5] = quat.z();
    point[6] = quat.w();
    return point;
}

}  // namespace path_interpolator
