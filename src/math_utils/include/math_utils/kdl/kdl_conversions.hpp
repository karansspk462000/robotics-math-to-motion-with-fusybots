#pragma once

#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "math_utils/pose/quaternion_conversions.hpp"

namespace math_utils {

// Eigen 4x4 homogeneous transform -> KDL::Frame.
inline KDL::Frame toKdlFrame(const Eigen::Matrix4d& pose) {
    Eigen::Isometry3d isometry;
    isometry.matrix() = pose;
    const Eigen::Quaterniond quat = toNormalizedQuaternion(isometry.rotation());

    const KDL::Rotation rotation = KDL::Rotation::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
    const Eigen::Vector3d translation = isometry.translation();
    const KDL::Vector kdl_translation(translation.x(), translation.y(), translation.z());
    return KDL::Frame(rotation, kdl_translation);
}

// KDL::Frame -> Eigen 4x4 homogeneous transform.
inline Eigen::Matrix4d toEigenMatrix(const KDL::Frame& frame) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            pose(row, col) = frame.M(row, col);
        }
        pose(row, 3) = frame.p(row);
    }
    return pose;
}

// Eigen::VectorXd -> KDL::JntArray.
inline KDL::JntArray toKdlJntArray(const Eigen::VectorXd& joint_positions) {
    KDL::JntArray joint_array(joint_positions.size());
    for (int i = 0; i < joint_positions.size(); ++i) {
        joint_array(i) = joint_positions(i);
    }
    return joint_array;
}

// KDL::JntArray -> Eigen::VectorXd.
inline Eigen::VectorXd toEigenVector(const KDL::JntArray& joint_array) {
    Eigen::VectorXd joint_positions(joint_array.rows());
    for (unsigned int i = 0; i < joint_array.rows(); ++i) {
        joint_positions(i) = joint_array(i);
    }
    return joint_positions;
}

}  // namespace math_utils
