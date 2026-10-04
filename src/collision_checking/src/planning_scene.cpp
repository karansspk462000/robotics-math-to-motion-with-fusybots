#include "collision_checking/planning_scene.hpp"

#include <stdexcept>

namespace collision_checking {

PlanningScene::PlanningScene(std::shared_ptr<kinematics::ForwardKinematics> forward_kinematics)
    : forward_kinematics_(std::move(forward_kinematics)) {
    if (!forward_kinematics_) {
        throw std::invalid_argument("PlanningScene: forward_kinematics must not be null");
    }
}

bool PlanningScene::isStateValid(const Eigen::VectorXd& joint_positions) const {
    const Eigen::Vector3d end_effector_position =
        forward_kinematics_->solve(joint_positions).cart_pose.block<3, 1>(0, 3);

    std::lock_guard<std::mutex> lock(mutex_);
    for (const CollisionObject& object : objects_) {
        if (isPointInsideObject(end_effector_position, object)) {
            return false;
        }
    }
    return true;
}

void PlanningScene::addObject(const CollisionObject& object) {
    std::lock_guard<std::mutex> lock(mutex_);
    objects_.push_back(object);
}

void PlanningScene::removeObject(std::size_t index) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index >= objects_.size()) {
        throw std::out_of_range("PlanningScene::removeObject: index out of range");
    }
    objects_.erase(objects_.begin() + static_cast<long>(index));
}

void PlanningScene::clearObjects() {
    std::lock_guard<std::mutex> lock(mutex_);
    objects_.clear();
}

std::size_t PlanningScene::numObjects() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return objects_.size();
}

}  // namespace collision_checking
