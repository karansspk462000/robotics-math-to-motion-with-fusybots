#include "kinematics/strategy/analytical_six_axis_inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace kinematics {

AnalyticalSixAxisInverseKinematics::AnalyticalSixAxisInverseKinematics(
    std::shared_ptr<AnalyticalSixAxisForwardKinematics> fk)
    : fk_(std::move(fk)) {
    if (!fk_) {
        throw std::invalid_argument("AnalyticalSixAxisInverseKinematics: fk must not be null");
    }
}

// Re-derived for a true spherical wrist (l7 = 0, i.e. axes 4/5/6 intersect
// at one point - see robot_fundamentals_ws fusybots_six_dof_spherical_wrist.yaml and
// urdf_tutorial's updated inverse_kinematics_logic.cpp). The previous
// version of this formula assumed l7 != 0 without a matching shoulder-
// offset term and did not correctly invert the forward chain (confirmed
// by feeding it the exact wrist position and finding cos_th3 didn't
// recover the true theta3 at all). This version adds the shoulder offset
// (l6 + d2, the net radial offset between the base axis and the arm
// plane) to th1's and th3's equations, which is the missing piece.
Eigen::VectorXd AnalyticalSixAxisInverseKinematics::computeJointPositions(
    const CartesianPosition& target_pose, const Eigen::VectorXd& initial_guess) const {
    const SixAxisLinkLengths& lengths = fk_->linkLengths();
    const double l1 = lengths.l1;
    const double l3 = lengths.l3;
    const double l5 = lengths.l5;
    const double l6 = lengths.l6;
    const double l7 = lengths.l7;
    const double le = lengths.le;
    const double shoulder_offset = l6 + lengths.d2;  // l6 + (l2 - l4)

    const Eigen::Matrix4d& HTM = target_pose.cart_pose;
    const Eigen::Vector3d position_end_effector = HTM.block<3, 1>(0, 3);
    const Eigen::Matrix3d rotation_matrix = HTM.block<3, 3>(0, 0);
    const Eigen::Vector3d d_end_effector(-l7, 0, le);
    const Eigen::Vector3d p_vec_wrist_wrt_base = position_end_effector - rotation_matrix * d_end_effector;

    const double x_wrist = p_vec_wrist_wrt_base(0);
    const double y_wrist = p_vec_wrist_wrt_base(1);
    const double z_wrist = p_vec_wrist_wrt_base(2);
    const double rho_sq = x_wrist * x_wrist + y_wrist * y_wrist;

    std::vector<Eigen::VectorXd> solutions;
    for (int i = 0; i < 8; ++i) {
        const int s2 = (i % 2 == 1) ? 1 : -1;
        const int s3 = ((i / 2) % 2 == 1) ? 1 : -1;
        const int s5 = ((i / 4) % 2 == 1) ? 1 : -1;

        // Clamp against floating-point overshoot at exact singularities
        // (e.g. wrist center on the shoulder-offset axis), where the
        // argument can land a few ULPs on the wrong side of 0.
        const double reach_sq = std::max(0.0, rho_sq - shoulder_offset * shoulder_offset);
        const double planar_reach = s2 * std::sqrt(reach_sq);
        const double th1 = std::atan2(y_wrist, x_wrist) + std::atan2(planar_reach, shoulder_offset);

        double cos_th3 = (planar_reach * planar_reach + (z_wrist - l1) * (z_wrist - l1) - l3 * l3 - l5 * l5) /
                          (2 * l3 * l5);
        cos_th3 = std::clamp(cos_th3, -1.0, 1.0);
        const double sin_th3 = std::sqrt(1 - cos_th3 * cos_th3);
        const double th3 = std::atan2(s3 * sin_th3, cos_th3);

        const double a = l3 + l5 * std::cos(th3);
        const double b = l5 * std::sin(th3);
        const double th2 = std::atan2(planar_reach, z_wrist - l1) - std::atan2(b, a);

        Eigen::VectorXd theta_03(3);
        theta_03 << th1, th2, th3;
        const Eigen::Matrix4d T03 = fk_->computeFk(3, theta_03);
        const Eigen::Matrix4d T03_inv = T03.inverse();

        Eigen::Matrix4d T_end_effector_to_wrist;
        T_end_effector_to_wrist << -1, 0, 0, l7,
                                    0, -1, 0, 0,
                                    0, 0, 1, le,
                                    0, 0, 0, 1;

        const Eigen::Matrix4d T36 = T03_inv * (HTM * T_end_effector_to_wrist.inverse());

        const double cos_th5 = std::clamp(T36(2, 2), -1.0, 1.0);
        const double sin_th5 = std::sqrt(1 - cos_th5 * cos_th5);
        const double th5 = std::atan2(s5 * sin_th5, cos_th5);

        const double th6 = std::atan2(T36(2, 0) * std::sin(th5), T36(2, 1) * std::sin(th5));
        const double th4 = std::atan2(T36(0, 2) * (-std::sin(th5)), T36(1, 2) * std::sin(th5));

        Eigen::VectorXd solution(6);
        solution << th1, th2, th3, th4, th5, th6;
        solutions.push_back(solution);
    }

    if (solutions.empty()) {
        throw std::runtime_error("AnalyticalSixAxisInverseKinematics: target pose is unreachable");
    }

    // PathPlanner::choose_best_sol: closest solution to the current joints.
    double min_norm = std::numeric_limits<double>::max();
    Eigen::VectorXd best_solution = solutions.front();
    for (const auto& solution : solutions) {
        const double norm = (solution - initial_guess).norm();
        if (norm < min_norm) {
            min_norm = norm;
            best_solution = solution;
        }
    }
    return best_solution;
}

Eigen::MatrixXd AnalyticalSixAxisInverseKinematics::computeJacobian(const Eigen::VectorXd& joint_positions) const {
    if (joint_positions.size() != 6) {
        throw std::invalid_argument("AnalyticalSixAxisInverseKinematics: expects exactly 6 joint positions");
    }

    // frames[i] = T_(0,i) for the 6 rotating joints only; frames.back() is
    // the wrist/joint-6 frame, NOT the actual tool tip - computeFk(7, ...)
    // still needs to be the source of end_effector_position, since it
    // additionally applies the fixed tool-offset row (d=le) that
    // computeJointFrames deliberately excludes (that row isn't a joint,
    // nothing rotates about it, so it has no Jacobian column - but the
    // linear-velocity formula below still needs the true tip position).
    const std::vector<Eigen::Matrix4d> frames = fk_->computeJointFrames(joint_positions);
    const Eigen::Matrix4d full_pose = fk_->computeFk(7, joint_positions);
    const Eigen::Vector3d end_effector_position = full_pose.block<3, 1>(0, 3);

    // Row i's own alpha, exactly as used inside computeFk/computeJointFrames
    // (must stay in sync with those - see six_axis_forward_kinematics.cpp).
    Eigen::VectorXd alpha(6);
    alpha << 0, M_PI_2, 0, 0, -M_PI_2, M_PI_2;

    Eigen::MatrixXd jacobian(6, 6);
    for (int i = 0; i < 6; ++i) {
        // This DH table is Craig's ("modified") convention - each row is
        // Rx(alpha_i) * Tx(a_i) * Rz(theta_i) * Tz(d_i), alpha_i applied
        // BEFORE that same row's own theta_i. So joint i's true rotation
        // axis is not frames[i]'s raw z-axis (that's frame i's z AFTER
        // the full row, i.e. after theta_i too) - it's frames[i]'s
        // rotation with row i's own alpha_i pre-applied, the frame
        // theta_i actually rotates about. Verified numerically against
        // KDL's independently-computed Jacobian (see
        // CartesianVelocity.KdlAndAnalyticalJacobiansAgree) - using the
        // raw z-axis here instead reproduces a spurious degenerate
        // Jacobian (two adjacent columns identical whenever alpha_i == 0).
        const Eigen::Vector3d local_axis(0.0, -std::sin(alpha[i]), std::cos(alpha[i]));
        const Eigen::Vector3d axis = frames[i].block<3, 3>(0, 0) * local_axis;
        // Craig's convention places frame i's origin on axis (i+1), not
        // axis i - so the point on THIS column's axis (axis i, 0-indexed)
        // is frames[i+1]'s origin, not frames[i]'s. Verified against
        // finite-differencing the already-trusted full-chain computeFk:
        // using frames[i] here instead silently duplicates adjacent
        // columns' linear part whenever two consecutive joints have
        // parallel axes (alpha_i == 0) - exactly what this robot's
        // shoulder/elbow/wrist-pitch triple does.
        const Eigen::Vector3d origin = frames[i + 1].block<3, 1>(0, 3);

        jacobian.block<3, 1>(0, i) = axis.cross(end_effector_position - origin);
        jacobian.block<3, 1>(3, i) = axis;
    }
    return jacobian;
}

}  // namespace kinematics
