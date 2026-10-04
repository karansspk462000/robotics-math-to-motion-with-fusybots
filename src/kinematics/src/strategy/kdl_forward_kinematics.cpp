#include "kinematics/strategy/kdl_forward_kinematics.hpp"

#include <stdexcept>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "math_utils/kdl/kdl_conversions.hpp"

namespace kinematics {

KdlForwardKinematics::KdlForwardKinematics(const std::string& urdf_xml, const std::string& start_link,
                                           const std::string& end_link) {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        throw std::runtime_error("KdlForwardKinematics: failed to parse URDF into a KDL tree");
    }
    if (!tree.getChain(start_link, end_link, chain_)) {
        throw std::runtime_error("KdlForwardKinematics: failed to extract chain from '" + start_link +
                                  "' to '" + end_link + "'");
    }
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
}

CartesianPosition KdlForwardKinematics::computeCartesianPosition(
    const Eigen::VectorXd& joint_positions) const {
    if (static_cast<unsigned int>(joint_positions.size()) != chain_.getNrOfJoints()) {
        throw std::invalid_argument(
            "KdlForwardKinematics: joint_positions size does not match the chain's joint count");
    }

    const KDL::JntArray joint_angles_kdl = math_utils::toKdlJntArray(joint_positions);

    KDL::Frame ee_pose;
    if (fk_solver_->JntToCart(joint_angles_kdl, ee_pose) < 0) {
        throw std::runtime_error("KdlForwardKinematics: JntToCart failed");
    }

    CartesianPosition result;
    result.cart_pose = math_utils::toEigenMatrix(ee_pose);
    return result;
}

int KdlForwardKinematics::numJoints() const {
    return static_cast<int>(chain_.getNrOfJoints());
}

}  // namespace kinematics
