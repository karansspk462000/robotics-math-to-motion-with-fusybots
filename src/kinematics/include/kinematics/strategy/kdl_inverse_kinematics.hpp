#pragma once

#include <memory>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include "kinematics/strategy/kdl_forward_kinematics.hpp"

namespace kinematics {

// IIKStrategy wrapping KDL's Newton-Raphson position IK solver
// (KDL::ChainIkSolverPos_NR), ported from Kinematics::calc_ik in
// urdf_tutorial/src/Kinematics.cpp. Needs the same KDL::Chain as an
// existing KdlForwardKinematics, so it's constructed from one rather than
// from a generic IFKStrategy.
class KdlInverseKinematics : public IIKStrategy {
public:
    explicit KdlInverseKinematics(std::shared_ptr<KdlForwardKinematics> fk, int max_iterations = 100,
                                   double tolerance = 1e-6);

    Eigen::VectorXd computeJointPositions(const CartesianPosition& target_pose,
                                           const Eigen::VectorXd& initial_guess) const override;

    // KDL's own analytical Jacobian (KDL::ChainJntToJacSolver), not a
    // finite-difference approximation - exact given the chain's DH
    // parameters, same [linear; angular] row convention as poseError.
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& joint_positions) const override;

private:
    std::shared_ptr<KdlForwardKinematics> fk_;
    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainIkSolverVel_pinv ik_vel_solver_;
    mutable KDL::ChainIkSolverPos_NR solver_;
    mutable KDL::ChainJntToJacSolver jac_solver_;
};

}  // namespace kinematics
