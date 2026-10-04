#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "collision_checking/common/collision_object.hpp"
#include "collision_checking/interface/collision_checker_interface.hpp"
#include "kinematics/context/kinematics.hpp"

namespace collision_checking {

// Context: the live, queryable model of "what's in the world" - the
// shared_ptr + runtime-mutable situation that justifies a Context in this
// codebase (see kinematics::ForwardKinematics's own doc comment): multiple
// independent owners observe the *same* scene (path_planner's RRT* today;
// motion_planner's start/goal pre-check; a runtime safety monitor and a
// vision-fed updater later), and it's expected to change at runtime once
// something is actually updating it (nothing does yet - see below).
//
// Phase 1 (now): static box/sphere obstacles (see CollisionObject),
// checked against only the end-effector position via the injected
// ForwardKinematics Context. Deliberately NOT doing full-arm swept-volume
// checking or self-collision yet - both need real per-link collision
// geometry, which does not exist anywhere in this workspace's URDFs today
// (see the README's future-scope list for what a real version needs:
// self-collision pairs, per-link primitives/meshes, a vision-fed live
// scene update). This class's job is to be the seam those slot into later
// without needing path_planner/motion_planner to change at all - they
// only ever see a bool through IsStateValidFn.
//
// Guarded by a mutex, not just documented, because the whole point of a
// PlanningScene is that something other than the thread calling
// isStateValid will eventually call addObject/removeObject (a vision
// node's callback, most likely) - see this project's own concurrency
// rule (docs/ARCHITECTURE_LECTURE_NOTES.md §5.1): a lock's presence only
// matters once a second real caller exists, and here one is explicitly
// planned, not speculative.
class PlanningScene : public ICollisionChecker {
public:
    explicit PlanningScene(std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics);

    // False iff the end-effector position at this joint configuration
    // (via forward_kinematics_->solve()) lands inside any registered
    // CollisionObject. FK exceptions (e.g. a malformed joint vector) are
    // not caught here - they propagate, since "FK failed" is a caller
    // bug, not a collision result.
    bool isStateValid(const Eigen::VectorXd& joint_positions) const override;

    void addObject(const CollisionObject& object);
    void removeObject(std::size_t index);
    void clearObjects();
    std::size_t numObjects() const;

private:
    std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics_;
    mutable std::mutex mutex_;
    std::vector<CollisionObject> objects_;
};

}  // namespace collision_checking
