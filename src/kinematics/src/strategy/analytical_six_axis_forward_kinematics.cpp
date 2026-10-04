#include "kinematics/strategy/analytical_six_axis_forward_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kinematics {

AnalyticalSixAxisForwardKinematics::AnalyticalSixAxisForwardKinematics(SixAxisLinkLengths lengths)
    : lengths_(lengths) {}

Eigen::Matrix4d AnalyticalSixAxisForwardKinematics::computeFk(double n, const Eigen::VectorXd& theta) const {
    Eigen::VectorXd alpha(7);
    alpha << 0, M_PI_2, 0, 0, -M_PI_2, M_PI_2, 0;

    Eigen::VectorXd a(7);
    a << 0, 0, -lengths_.l3, -lengths_.l5, 0, 0, 0;

    Eigen::VectorXd d(7);
    d << lengths_.l1, 0, lengths_.d2, lengths_.l6, lengths_.l7, 0, lengths_.le;

    Eigen::VectorXd adjusted_theta = theta;
    adjusted_theta[0] += M_PI_2;
    adjusted_theta[1] -= M_PI_2;
    if (theta.size() > 3) {
        adjusted_theta[3] += M_PI_2;
        adjusted_theta[5] += M_PI_2;
    }

    Eigen::Matrix4d Ttemp = Eigen::Matrix4d::Identity();
    if (n > 6) {
        adjusted_theta.conservativeResize(adjusted_theta.size() + 1);
        adjusted_theta(adjusted_theta.size() - 1) = -M_PI;
    }

    const int num_rows = std::min(static_cast<int>(adjusted_theta.size()), 7);
    for (int i = 0; i < num_rows; ++i) {
        Eigen::Matrix4d T;
        T << cos(adjusted_theta[i]), -sin(adjusted_theta[i]), 0, a[i],
             sin(adjusted_theta[i]) * cos(alpha[i]), cos(adjusted_theta[i]) * cos(alpha[i]), -sin(alpha[i]), -d[i] * sin(alpha[i]),
             sin(adjusted_theta[i]) * sin(alpha[i]), cos(adjusted_theta[i]) * sin(alpha[i]), cos(alpha[i]), d[i] * cos(alpha[i]),
             0, 0, 0, 1;

        Ttemp = Ttemp * T;
    }
    return Ttemp;
}

std::vector<Eigen::Matrix4d> AnalyticalSixAxisForwardKinematics::computeJointFrames(
    const Eigen::VectorXd& joint_positions) const {
    if (joint_positions.size() != 6) {
        throw std::invalid_argument("AnalyticalSixAxisForwardKinematics: expects exactly 6 joint positions");
    }

    Eigen::VectorXd alpha(6);
    alpha << 0, M_PI_2, 0, 0, -M_PI_2, M_PI_2;

    Eigen::VectorXd a(6);
    a << 0, 0, -lengths_.l3, -lengths_.l5, 0, 0;

    Eigen::VectorXd d(6);
    d << lengths_.l1, 0, lengths_.d2, lengths_.l6, lengths_.l7, 0;

    Eigen::VectorXd adjusted_theta = joint_positions;
    adjusted_theta[0] += M_PI_2;
    adjusted_theta[1] -= M_PI_2;
    adjusted_theta[3] += M_PI_2;
    adjusted_theta[5] += M_PI_2;

    std::vector<Eigen::Matrix4d> frames;
    frames.reserve(7);
    frames.push_back(Eigen::Matrix4d::Identity());

    for (int i = 0; i < 6; ++i) {
        Eigen::Matrix4d T;
        T << cos(adjusted_theta[i]), -sin(adjusted_theta[i]), 0, a[i],
             sin(adjusted_theta[i]) * cos(alpha[i]), cos(adjusted_theta[i]) * cos(alpha[i]), -sin(alpha[i]),
                 -d[i] * sin(alpha[i]),
             sin(adjusted_theta[i]) * sin(alpha[i]), cos(adjusted_theta[i]) * sin(alpha[i]), cos(alpha[i]),
                 d[i] * cos(alpha[i]),
             0, 0, 0, 1;
        frames.push_back(frames.back() * T);
    }
    return frames;
}

CartesianPosition AnalyticalSixAxisForwardKinematics::computeCartesianPosition(
    const Eigen::VectorXd& joint_positions) const {
    if (joint_positions.size() != 6) {
        throw std::invalid_argument(
            "AnalyticalSixAxisForwardKinematics: expects exactly 6 joint positions");
    }
    CartesianPosition result;
    result.cart_pose = computeFk(7, joint_positions);
    return result;
}

int AnalyticalSixAxisForwardKinematics::numJoints() const {
    return 6;
}

}  // namespace kinematics
