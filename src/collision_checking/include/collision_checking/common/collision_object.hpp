#pragma once

#include <Eigen/Dense>

namespace collision_checking {

// Phase 1: the only two primitive shapes needed to represent a static
// obstacle - a box (axis-aligned in its own local frame, arbitrarily
// posed in the world via `center` + `orientation`) or a sphere (radius
// only, `half_extents.x()`, orientation irrelevant). Add more shapes here
// only when something concretely needs one - meshes in particular are a
// much bigger lift (real geometry, not a closed-form point-inside test)
// and nothing in this workspace needs one yet.
enum class CollisionShape { kBox, kSphere };

struct CollisionObject {
    CollisionShape shape = CollisionShape::kSphere;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    // Box: half-extent along each local axis. Sphere: only x() is used, as
    // the radius - y()/z() are ignored (kept as one struct instead of a
    // variant/union for simplicity; this is Phase 1 scope, not a general
    // geometry library).
    Eigen::Vector3d half_extents = Eigen::Vector3d::Constant(0.1);
    // Box only: world-frame rotation of the box's local axes. Ignored for
    // spheres (rotating a sphere about its own center is a no-op).
    Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
};

// True if `point` (world frame) lies inside `object`, inclusive of the
// boundary. Box test: transform the point into the box's local frame
// (undo orientation/center), then a simple per-axis half-extent bound
// check - this is exactly why `orientation` is stored as a 3x3 rotation
// rather than e.g. Euler angles, so this transform is a single matrix
// multiply.
bool isPointInsideObject(const Eigen::Vector3d& point, const CollisionObject& object);

}  // namespace collision_checking
