// Regression tests scoped to the kinematics package alone: FK/IK
// round-tripping, Jacobian correctness, and the robot's "home" pose staying
// clear of singularities. No motion_planner/path_planner/path_interpolator/
// trajectory_planner dependency - see test_motion_planner.cpp for tests that
// exercise the full pipeline.
#include <gtest/gtest.h>

#include "kinematics/common/jacobian.hpp"
#include "kinematics/context/kinematics.hpp"
#include "kinematics/factory/kinematics_factory.hpp"
#include "robot_params.hpp"

using test_helpers::analyticalParams;
using test_helpers::kdlParams;

// Regression for: KdlInverseKinematics's constructor used a `cond ?
// fk_->chain() : KDL::Chain()` ternary to guard against a null fk_, whose
// mismatched-value-category branches forced the compiler to copy the
// chain for every solver - silently breaking NR convergence for every
// target, including the exact solution as its own seed.
TEST(KdlKinematics, IkConvergesFromMultipleSeedsIncludingZero) {
    auto fk = kinematics::createFKStrategy("kdl", kdlParams());
    auto ik = kinematics::createIKStrategy("kdl", fk);

    Eigen::VectorXd q_target(6);
    q_target << 0.4, 0.3, -0.3, 0.2, 0.5, -0.2;
    const auto target_pose = fk->computeCartesianPosition(q_target);

    for (double seed_value : {0.0, 0.1, -0.2, 0.39}) {
        Eigen::VectorXd seed = Eigen::VectorXd::Constant(6, seed_value);
        Eigen::VectorXd solution;
        ASSERT_NO_THROW(solution = ik->computeJointPositions(target_pose, seed)) << "seed=" << seed_value;
        const auto check_pose = fk->computeCartesianPosition(solution);
        const double error = (target_pose.cart_pose - check_pose.cart_pose).norm();
        EXPECT_LT(error, 1e-4) << "seed=" << seed_value << " converged to a wrong pose, error=" << error;
    }
}

// Regression for: the closed-form analytical IK's th1/th3 equations were
// missing a shoulder_offset = l6 + d2 term, so cos_th3 didn't recover the
// true th3 even given the exact wrist position - fixed by re-deriving with
// shoulder_offset, requires a true spherical wrist (l7 = 0).
TEST(AnalyticalKinematics, IkRoundTripsToMachinePrecisionOnSphericalWrist) {
    auto fk = kinematics::createFKStrategy("analytical", analyticalParams());
    auto ik = kinematics::createIKStrategy("analytical", fk);

    const std::vector<Eigen::VectorXd> test_configs = {
        (Eigen::VectorXd(6) << 0.3, -0.2, 0.5, 0.1, 0.4, -0.1).finished(),
        (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
        (Eigen::VectorXd(6) << 1.0, 0.5, -0.5, 0.3, 0.7, -0.4).finished(),
        (Eigen::VectorXd(6) << -0.6, 0.3, 0.2, -0.5, 0.2, 0.9).finished(),
    };

    for (const auto& q : test_configs) {
        const auto target_pose = fk->computeCartesianPosition(q);
        const auto solution = ik->computeJointPositions(target_pose, Eigen::VectorXd::Zero(6));
        const auto check_pose = fk->computeCartesianPosition(solution);
        const double error = (target_pose.cart_pose - check_pose.cart_pose).norm();
        EXPECT_LT(error, 1e-9) << "q=" << q.transpose() << " round-trip error=" << error;
    }
}

// Regression for: the robot's "home" pose (see home_joint_position in
// robots/fusybots_six_dof_spherical_wrist.yaml - MUST be kept identical to the vector
// below) must not itself sit on a kinematic singularity. All-zero joints
// (the previous default) put j3 at exactly 0 - elbow fully straight, a
// textbook singularity - which was discovered by a grid search after
// naively guessing that bending j2/j5 away from zero would be enough (it
// wasn't: j2/j5 barely mattered, only j3 did). This test locks in that a
// small move in every Cartesian direction, and several teleop-scale (2cm
// to 15cm) moves reaching up (+z) and out (+x) with y near zero converge,
// exactly the move pattern a "go home, then jog the arm" workflow uses.
TEST(HomePosition, IsNonSingularForTypicalCartesianMoves) {
    auto fk = kinematics::createFKStrategy("kdl", kdlParams());
    auto ik_strategy = kinematics::createIKStrategy("kdl", fk);
    auto ik = std::make_shared<kinematics::InverseKinematics>(std::move(ik_strategy));

    Eigen::VectorXd home(6);
    home << 0.0, 1.3, 0.8, 0.0, -1.2, 0.0;
    const auto home_pose = fk->computeCartesianPosition(home);

    const std::vector<Eigen::Vector3d> tiny_dirs = {
        {0.00005, 0, 0}, {-0.00005, 0, 0}, {0, 0.00005, 0}, {0, -0.00005, 0}, {0, 0, 0.00005}, {0, 0, -0.00005},
    };
    for (const auto& d : tiny_dirs) {
        kinematics::CartesianPosition target;
        target.cart_pose = home_pose.cart_pose;
        target.cart_pose.block<3, 1>(0, 3) += d;
        EXPECT_NO_THROW(ik->solve(target, home)) << "tiny move delta=" << d.transpose()
                                                    << " did not converge - home is on/near a singularity";
    }

    const std::vector<Eigen::Vector3d> teleop_dirs = {
        {0.02, 0, 0.02}, {0.05, 0, 0.05}, {0.08, 0.01, 0.08}, {0.1, 0, 0.1}, {0.05, 0, 0.1},
        {0.1, 0, 0.03}, {0.03, -0.01, 0.1}, {0.12, 0.02, 0.05}, {0.06, 0, 0.15},
    };
    for (const auto& d : teleop_dirs) {
        Eigen::VectorXd seed = home;
        kinematics::CartesianPosition target;
        target.cart_pose = home_pose.cart_pose;
        target.cart_pose.block<3, 1>(0, 3) += d;
        EXPECT_NO_THROW(seed = ik->solve(target, seed))
            << "teleop-scale move delta=" << d.transpose() << " did not converge from home";
    }
}

// Regression for motion_status.cartesian_velocity (robot_node's
// hwStatusCallback): cartesian_velocity = J(q) * joint_velocity must
// actually match real end-effector motion, not just be dimensionally
// plausible. Verified against an independent ground truth: finite-
// differencing FK itself along q(t) = q0 + t*qdot for a tiny t, using the
// same poseError() twist convention the Jacobian is built from - if this
// test and computeNumericalJacobian ever used different or inconsistent
// twist conventions, this comparison would catch it even though each
// might look internally self-consistent on its own.
TEST(CartesianVelocity, JacobianTimesJointVelocityMatchesFiniteDifference) {
    auto fk = kinematics::createFKStrategy("kdl", kdlParams());

    Eigen::VectorXd q(6);
    q << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    Eigen::VectorXd qdot(6);
    qdot << 0.2, -0.3, 0.15, -0.1, 0.25, -0.2;

    const Eigen::MatrixXd jacobian = kinematics::computeNumericalJacobian(*fk, q);
    const Eigen::VectorXd cartesian_velocity = jacobian * qdot;

    const double dt = 1e-6;
    const Eigen::Matrix4d pose_now = fk->computeCartesianPosition(q).cart_pose;
    const Eigen::Matrix4d pose_next = fk->computeCartesianPosition(q + dt * qdot).cart_pose;
    const Eigen::VectorXd finite_difference_twist = kinematics::poseError(pose_now, pose_next) / dt;

    const double error = (cartesian_velocity - finite_difference_twist).norm();
    EXPECT_LT(error, 1e-4) << "J(q)*qdot=" << cartesian_velocity.transpose()
                            << " vs finite-difference=" << finite_difference_twist.transpose();
}

// Regression/cross-validation for IIKStrategy::computeJacobian: KDL's
// implementation (KDL::ChainJntToJacSolver, its own internal analytical
// Jacobian) and AnalyticalSixAxisInverseKinematics's implementation (our
// own closed-form derivation - see analytical_six_axis_inverse_kinematics
// .cpp's computeJacobian) are two completely independent code paths, on
// the same physical robot (fusybots_six_dof_spherical_wrist - both kdlParams() and
// analyticalParams() describe it). If they agree at many configurations,
// that's strong evidence both are actually correct, not just internally
// self-consistent - two independent bugs would have to happen to agree.
TEST(CartesianVelocity, KdlAndAnalyticalJacobiansAgree) {
    auto kdl_fk = kinematics::createFKStrategy("kdl", kdlParams());
    auto kdl_ik = kinematics::createIKStrategy("kdl", kdl_fk);

    auto analytical_fk = kinematics::createFKStrategy("analytical", analyticalParams());
    auto analytical_ik = kinematics::createIKStrategy("analytical", analytical_fk);

    const std::vector<Eigen::VectorXd> test_configs = {
        (Eigen::VectorXd(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished(),
        (Eigen::VectorXd(6) << 0.3, -0.2, 0.5, 0.1, 0.4, -0.1).finished(),
        (Eigen::VectorXd(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
        (Eigen::VectorXd(6) << 1.0, 0.5, -0.5, 0.3, 0.7, -0.4).finished(),
        (Eigen::VectorXd(6) << -0.6, 0.3, 0.2, -0.5, 0.2, 0.9).finished(),
    };

    for (const auto& q : test_configs) {
        const Eigen::MatrixXd kdl_jacobian = kdl_ik->computeJacobian(q);
        const Eigen::MatrixXd analytical_jacobian = analytical_ik->computeJacobian(q);

        ASSERT_EQ(kdl_jacobian.rows(), 6);
        ASSERT_EQ(kdl_jacobian.cols(), 6);
        ASSERT_EQ(analytical_jacobian.rows(), 6);
        ASSERT_EQ(analytical_jacobian.cols(), 6);

        const double error = (kdl_jacobian - analytical_jacobian).norm();
        EXPECT_LT(error, 1e-6) << "q=" << q.transpose() << "\nkdl J=\n"
                                << kdl_jacobian << "\nanalytical J=\n"
                                << analytical_jacobian;
    }
}
