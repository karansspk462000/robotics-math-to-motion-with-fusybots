// Unit tests for collision_checking::PlanningScene in isolation - real
// kinematics (KDL, this robot's actual URDF) so isStateValid's end-effector
// position checks are against real FK, not a stub.
#include <gtest/gtest.h>

#include "collision_checking/planning_scene.hpp"
#include "kinematics/context/kinematics.hpp"
#include "kinematics/factory/kinematics_factory.hpp"
#include "robot_params.hpp"

using collision_checking::CollisionObject;
using collision_checking::CollisionShape;
using collision_checking::PlanningScene;

namespace {

std::shared_ptr<kinematics::ForwardKinematics> buildForwardKinematics() {
    auto fk_strategy = kinematics::createFKStrategy("kdl", test_helpers::kdlParams());
    return std::make_shared<kinematics::ForwardKinematics>(fk_strategy);
}

}  // namespace

TEST(PlanningScene, EmptySceneIsAlwaysValid) {
    PlanningScene scene(buildForwardKinematics());
    EXPECT_EQ(scene.numObjects(), 0u);

    Eigen::VectorXd q(6);
    q << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    EXPECT_TRUE(scene.isStateValid(q));
}

TEST(PlanningScene, RejectsStateWithEndEffectorInsideASphereObstacle) {
    auto fk = buildForwardKinematics();
    PlanningScene scene(fk);

    Eigen::VectorXd home(6);
    home << 0.0, 1.3, 0.8, 0.0, -1.2, 0.0;  // this robot's real home pose
    const Eigen::Vector3d ee_position = fk->solve(home).cart_pose.block<3, 1>(0, 3);

    // Sanity: home is valid before any obstacle exists.
    ASSERT_TRUE(scene.isStateValid(home));

    CollisionObject obstacle;
    obstacle.shape = CollisionShape::kSphere;
    obstacle.center = ee_position;  // put the obstacle exactly where the end-effector is
    obstacle.half_extents = Eigen::Vector3d::Constant(0.05);  // 5cm radius
    scene.addObject(obstacle);
    EXPECT_EQ(scene.numObjects(), 1u);

    EXPECT_FALSE(scene.isStateValid(home)) << "end-effector is exactly at the obstacle's center";
}

TEST(PlanningScene, RejectsStateWithEndEffectorInsideABoxObstacle) {
    auto fk = buildForwardKinematics();
    PlanningScene scene(fk);

    Eigen::VectorXd home(6);
    home << 0.0, 1.3, 0.8, 0.0, -1.2, 0.0;
    const Eigen::Vector3d ee_position = fk->solve(home).cart_pose.block<3, 1>(0, 3);

    CollisionObject box;
    box.shape = CollisionShape::kBox;
    box.center = ee_position;
    box.half_extents = Eigen::Vector3d::Constant(0.05);
    box.orientation = Eigen::Matrix3d::Identity();
    scene.addObject(box);

    EXPECT_FALSE(scene.isStateValid(home));
}

TEST(PlanningScene, RemoveObjectRestoresValidity) {
    auto fk = buildForwardKinematics();
    PlanningScene scene(fk);

    Eigen::VectorXd home(6);
    home << 0.0, 1.3, 0.8, 0.0, -1.2, 0.0;
    const Eigen::Vector3d ee_position = fk->solve(home).cart_pose.block<3, 1>(0, 3);

    CollisionObject obstacle;
    obstacle.shape = CollisionShape::kSphere;
    obstacle.center = ee_position;
    obstacle.half_extents = Eigen::Vector3d::Constant(0.05);
    scene.addObject(obstacle);
    ASSERT_FALSE(scene.isStateValid(home));

    scene.removeObject(0);
    EXPECT_EQ(scene.numObjects(), 0u);
    EXPECT_TRUE(scene.isStateValid(home));
}

TEST(PlanningScene, ObstacleFarFromEndEffectorDoesNotAffectValidity) {
    auto fk = buildForwardKinematics();
    PlanningScene scene(fk);

    Eigen::VectorXd home(6);
    home << 0.0, 1.3, 0.8, 0.0, -1.2, 0.0;

    CollisionObject far_away;
    far_away.shape = CollisionShape::kSphere;
    far_away.center = Eigen::Vector3d(100.0, 100.0, 100.0);
    far_away.half_extents = Eigen::Vector3d::Constant(0.5);
    scene.addObject(far_away);

    EXPECT_TRUE(scene.isStateValid(home));
}

TEST(PlanningScene, ConstructorRejectsNullForwardKinematics) {
    EXPECT_THROW(PlanningScene scene(nullptr), std::invalid_argument);
}
