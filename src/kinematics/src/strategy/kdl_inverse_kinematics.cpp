#include "kinematics/strategy/kdl_inverse_kinematics.hpp"

#include <stdexcept>

#include "math_utils/kdl/kdl_conversions.hpp"

namespace {

// Used directly in the constructor's initializer list, ahead of
// fk_solver_/ik_vel_solver_/solver_, so a null fk_ is rejected before any
// solver is constructed - without this, a `cond ? fk_->chain() :
// KDL::Chain()` ternary would be needed instead, and that ternary's
// mismatched-value-category branches (a reference vs a temporary) force
// the compiler to materialize a *copy* of the chain for every solver.
// That copy is enough to make KDL::ChainIkSolverPos_NR stop converging
// (verified: identical setup, only this ternary added, took CartToJnt
// from returning 0/success to -3/max-iterations-exceeded on every input,
// including the exact target position as the initial guess).
const KDL::Chain& chainOrThrow(const std::shared_ptr<kinematics::KdlForwardKinematics>& fk) {
    if (!fk) {
        throw std::invalid_argument("KdlInverseKinematics: fk must not be null");
    }
    return fk->chain();
}

}  // namespace

namespace kinematics {

KdlInverseKinematics::KdlInverseKinematics(std::shared_ptr<KdlForwardKinematics> fk, int max_iterations,
                                            double tolerance)
    : fk_(std::move(fk)),
      fk_solver_(chainOrThrow(fk_)),
      ik_vel_solver_(fk_->chain()),
      solver_(fk_->chain(), fk_solver_, ik_vel_solver_, max_iterations, tolerance),
      jac_solver_(fk_->chain()) {}

Eigen::VectorXd KdlInverseKinematics::computeJointPositions(const CartesianPosition& target_pose,
                                                             const Eigen::VectorXd& initial_guess) const {
    const unsigned int num_joints = fk_->chain().getNrOfJoints();
    if (static_cast<unsigned int>(initial_guess.size()) != num_joints) {
        throw std::invalid_argument(
            "KdlInverseKinematics: initial_guess size does not match the chain's joint count");
    }

    const KDL::JntArray q_init = math_utils::toKdlJntArray(initial_guess);
    const KDL::Frame target_frame = math_utils::toKdlFrame(target_pose.cart_pose);

    KDL::JntArray q_out(num_joints);
    if (solver_.CartToJnt(q_init, target_frame, q_out) < 0) {
        throw std::runtime_error("KdlInverseKinematics: CartToJnt failed to converge");
    }

    return math_utils::toEigenVector(q_out);
}

Eigen::MatrixXd KdlInverseKinematics::computeJacobian(const Eigen::VectorXd& joint_positions) const {
    const unsigned int num_joints = fk_->chain().getNrOfJoints();
    if (static_cast<unsigned int>(joint_positions.size()) != num_joints) {
        throw std::invalid_argument(
            "KdlInverseKinematics: joint_positions size does not match the chain's joint count");
    }

    const KDL::JntArray q = math_utils::toKdlJntArray(joint_positions);
    KDL::Jacobian jacobian(num_joints);
    if (jac_solver_.JntToJac(q, jacobian) < 0) {
        throw std::runtime_error("KdlInverseKinematics: JntToJac failed");
    }
    return jacobian.data;
}

}  // namespace kinematics
