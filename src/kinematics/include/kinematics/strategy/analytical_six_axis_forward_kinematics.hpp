#pragma once

#include <vector>

#include "kinematics/interface/kinematics_interface.hpp"
#include "kinematics/common/six_axis_link_lengths.hpp"

namespace kinematics {

// Closed-form (analytical DH) forward kinematics for the specific 6R
// spherical-wrist arm defined in fusybots_six_dof.urdf.xacro, ported from
// urdf_tutorial/src/forward_kinematics_logic.cpp. This is not a generic
// n-DOF DH evaluator (that's DHForwardKinematics) - it hardcodes this
// robot's link topology so that AnalyticalSixAxisInverseKinematics can pair
// it with a matching closed-form solution.
class AnalyticalSixAxisForwardKinematics : public IFKStrategy {
public:
    explicit AnalyticalSixAxisForwardKinematics(SixAxisLinkLengths lengths);

    CartesianPosition computeCartesianPosition(const Eigen::VectorXd& joint_positions) const override;
    int numJoints() const override;

    // Ported from the original calculate_fk(n, theta): evaluates the DH
    // chain for min(theta.size(), 7) rows, appending the fixed tool row
    // (theta = -pi) when n > 6. Exposed (not just computeCartesianPosition)
    // because AnalyticalSixAxisInverseKinematics needs the same partial-
    // chain evaluation internally.
    //
    // Only ever verified/used at theta.size() == 3 (T03, for the IK's
    // shoulder/elbow solve) or == 6/7 (full chain) - do NOT call this with
    // other partial sizes: the fixed theta-offset adjustments below index
    // fixed positions (0, 1, 3, 5) unconditionally once size > 3, so a
    // size of 1, 4, or 5 either indexes out of bounds outright or silently
    // applies an offset meant for a different joint. Use
    // computeJointFrames() instead if you need every intermediate frame.
    Eigen::Matrix4d computeFk(double n, const Eigen::VectorXd& theta) const;

    // Returns T_(0,i) for i = 0..6: T_(0,0) is the base frame (identity),
    // T_(0,i) for i = 1..6 is the transform after applying joint i's own
    // DH row only (never computeFk's n>6 fixed tool row). Safe for any
    // caller that needs every per-joint frame - unlike computeFk, there
    // is no partial-chain ambiguity: `joint_positions` must be the full
    // 6-vector, the theta-offset adjustments are computed once from it,
    // then applied per-row while accumulating. This is exactly what a
    // geometric Jacobian needs (see
    // AnalyticalSixAxisInverseKinematics::computeJacobian) - each column
    // i's rotation axis/origin is frame (i-1)'s z-axis/position.
    std::vector<Eigen::Matrix4d> computeJointFrames(const Eigen::VectorXd& joint_positions) const;

    const SixAxisLinkLengths& linkLengths() const { return lengths_; }

private:
    SixAxisLinkLengths lengths_;
};

}  // namespace kinematics
