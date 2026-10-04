#pragma once

#include <memory>

#include "kinematics/strategy/analytical_six_axis_forward_kinematics.hpp"

namespace kinematics {

// Closed-form inverse kinematics for the same 6R spherical-wrist arm as
// AnalyticalSixAxisForwardKinematics, ported from
// urdf_tutorial/src/inverse_kinematics_logic.cpp. Enumerates all 8
// shoulder/elbow/wrist sign-branch solutions and returns the one closest to
// initial_guess (matching PathPlanner::choose_best_sol from
// urdf_tutorial/src/path_planner_logic.cpp).
//
// Requires a true spherical wrist (SixAxisLinkLengths::l7 == 0, i.e. axes
// 4/5/6 intersect at one point - see fusybots_six_dof_spherical_wrist.yaml). Verified via
// FK/IK round-trip against kdl on that config: ~1e-16 (machine precision)
// pose error across multiple joint configurations, including q=0. Do not
// use this with a robot description that has l7 != 0 (e.g. fusybots_six_dof.yaml)
// - the shoulder_offset term this formula relies on assumes the wrist
// offset is zero; verified round-trip error was ~0.3 (unusably large) on
// that non-spherical config.
class AnalyticalSixAxisInverseKinematics : public IIKStrategy {
public:
    explicit AnalyticalSixAxisInverseKinematics(std::shared_ptr<AnalyticalSixAxisForwardKinematics> fk);

    Eigen::VectorXd computeJointPositions(const CartesianPosition& target_pose,
                                           const Eigen::VectorXd& initial_guess) const override;

    // Closed-form geometric Jacobian, derived (not finite-differenced)
    // from the same DH chain fk_ evaluates: for revolute joint i, column
    // i is [z_(i-1) x (p_e - p_(i-1)); z_(i-1)], the standard result for
    // a revolute joint's instantaneous linear/angular velocity
    // contribution, where z_(i-1)/p_(i-1) are frame (i-1)'s z-axis/origin
    // in the base frame (obtained via fk_->computeFk on partial-length
    // joint vectors) and p_e is the end-effector position. The DH rows'
    // fixed theta offsets (see analytical_six_axis_forward_kinematics.cpp)
    // are additive constants w.r.t. the raw joint variable, so they don't
    // change this derivative.
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& joint_positions) const override;

private:
    std::shared_ptr<AnalyticalSixAxisForwardKinematics> fk_;
};

}  // namespace kinematics
