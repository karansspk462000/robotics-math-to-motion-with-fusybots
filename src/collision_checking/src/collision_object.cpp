#include "collision_checking/common/collision_object.hpp"

#include <cmath>

namespace collision_checking {

bool isPointInsideObject(const Eigen::Vector3d& point, const CollisionObject& object) {
    if (object.shape == CollisionShape::kSphere) {
        return (point - object.center).norm() <= object.half_extents.x();
    }
    // Box: transform into the box's own local frame (orientation is the
    // box's world-frame rotation, so its transpose/inverse rotates the
    // point back into local axes), then a per-axis half-extent check.
    const Eigen::Vector3d local_point = object.orientation.transpose() * (point - object.center);
    return std::abs(local_point.x()) <= object.half_extents.x() &&
           std::abs(local_point.y()) <= object.half_extents.y() &&
           std::abs(local_point.z()) <= object.half_extents.z();
}

}  // namespace collision_checking
