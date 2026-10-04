#pragma once

#include <Eigen/Dense>

namespace kinematics {

// Pose of the end-effector expressed as a homogeneous transform.
struct CartesianPosition {
    Eigen::Matrix4d cart_pose = Eigen::Matrix4d::Identity();
};

// Strategy interface for forward kinematics: joint space -> Cartesian pose.
// Concrete strategies (analytical, PoE, KDL, Pinocchio, ...) implement this
// so that ForwardKinematics can use any of them interchangeably.
class IFKStrategy {
public:
    virtual ~IFKStrategy() = default;
    virtual CartesianPosition computeCartesianPosition(
        const Eigen::VectorXd& joint_positions) const = 0;
    virtual int numJoints() const = 0;
};

// Strategy interface for inverse kinematics: Cartesian pose -> joint space.
class IIKStrategy {
public:
    virtual ~IIKStrategy() = default;
    virtual Eigen::VectorXd computeJointPositions(
        const CartesianPosition& target_pose,
        const Eigen::VectorXd& initial_guess) const = 0;

    // 6xN geometric Jacobian (3 linear + 3 angular rows) at
    // `joint_positions`, in the same twist convention as
    // kinematics::poseError (rows 0-2: linear velocity; rows 3-5: angular
    // velocity, both in the base frame). Each concrete strategy computes
    // this its own way - KDL uses KDL::ChainJntToJacSolver (its own
    // analytical Jacobian), AnalyticalSixAxisInverseKinematics derives it
    // in closed form from the same DH geometry its IK solves against -
    // rather than one generic finite-difference implementation shared by
    // everyone, so callers get the real Jacobian for whichever method is
    // actually configured.
    virtual Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& joint_positions) const = 0;
};

}  // namespace kinematics
