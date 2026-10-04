#pragma once

#include <Eigen/Dense>

#include "kinematics/interface/kinematics_interface.hpp"

namespace kinematics {

// 6x1 twist error between two poses: head<3>() is the position delta
// (target - current), tail<3>() is the axis-angle rotation that takes
// current's orientation to target's (angle * axis, so its norm is the
// rotation angle in radians). This is the twist convention every
// IIKStrategy::computeJacobian implementation is expected to match (KDL's
// via KDL::Jacobian's own [linear; angular] row order, the analytical
// strategy's via its closed-form derivation) - see
// CrossValidation.KdlAndAnalyticalJacobiansAgree in integration_tests,
// which would catch it if any of them ever drifted from this convention.
Eigen::VectorXd poseError(const Eigen::Matrix4d& current, const Eigen::Matrix4d& target);

// Finite-difference geometric Jacobian (6xN: 3 linear + 3 angular rows,
// one column per joint) at `joint_positions`, generic over any
// IFKStrategy. NOT used by any production code path anymore - each
// IIKStrategy computes its own real Jacobian now (KDL::ChainJntToJacSolver
// for kdl, a closed-form derivation for analytical - see
// IIKStrategy::computeJacobian). This exists purely as an independent,
// method-agnostic ground truth for tests: comparing two analytical
// Jacobians against each other proves they agree with each other, but
// comparing both against this finite-difference version (which shares no
// code with either) is a stronger check that they also agree with the
// actual geometry.
Eigen::MatrixXd computeNumericalJacobian(const IFKStrategy& fk_strategy, const Eigen::VectorXd& joint_positions,
                                          double step = 1e-6);

}  // namespace kinematics
