#pragma once

#include <vector>

#include <Eigen/Dense>

namespace math_utils {

// std::vector<T> -> Eigen::VectorXd, widening T (e.g. ROS message fields
// are frequently float32/float64, Eigen math is done in double).
template <typename T>
inline Eigen::VectorXd toEigenVector(const std::vector<T>& values) {
    Eigen::VectorXd vector(values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
        vector[static_cast<int>(i)] = static_cast<double>(values[i]);
    }
    return vector;
}

// Eigen::VectorXd -> std::vector<T> (T defaults to double; pass T=float
// for a ROS message's float32[] field, e.g. custom_robot_msgs::HwStatus).
template <typename T = double>
inline std::vector<T> toStdVector(const Eigen::VectorXd& vector) {
    std::vector<T> result(vector.size());
    for (int i = 0; i < vector.size(); ++i) {
        result[i] = static_cast<T>(vector[i]);
    }
    return result;
}

}  // namespace math_utils
