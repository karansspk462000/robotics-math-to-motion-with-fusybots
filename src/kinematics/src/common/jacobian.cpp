#include "kinematics/common/jacobian.hpp"

namespace kinematics {

Eigen::VectorXd poseError(const Eigen::Matrix4d& current, const Eigen::Matrix4d& target) {
    Eigen::VectorXd error(6);
    error.head<3>() = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);

    const Eigen::Matrix3d rotation_error = target.block<3, 3>(0, 0) * current.block<3, 3>(0, 0).transpose();
    const Eigen::AngleAxisd angle_axis(rotation_error);
    error.tail<3>() = angle_axis.angle() * angle_axis.axis();
    return error;
}

Eigen::MatrixXd computeNumericalJacobian(const IFKStrategy& fk_strategy, const Eigen::VectorXd& joint_positions,
                                          double step) {
    const int n = static_cast<int>(joint_positions.size());
    Eigen::MatrixXd jacobian(6, n);

    const Eigen::Matrix4d base_pose = fk_strategy.computeCartesianPosition(joint_positions).cart_pose;

    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd perturbed = joint_positions;
        perturbed[i] += step;
        const Eigen::Matrix4d perturbed_pose = fk_strategy.computeCartesianPosition(perturbed).cart_pose;
        jacobian.col(i) = poseError(base_pose, perturbed_pose) / step;
    }
    return jacobian;
}

}  // namespace kinematics
