#pragma once

#include <memory>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "kinematics/interface/kinematics_interface.hpp"

namespace kinematics {

// IFKStrategy built directly from a URDF via kdl_parser - no manual DH
// derivation, works for any chain topology (including the non-axis-aligned
// fixed offsets in fusybots_six_dof.urdf.xacro). Ported from
// urdf_tutorial/src/Kinematics.cpp (Kinematics::initKinematics /
// calculate_fk), split into a dedicated FK strategy so it composes with the
// rest of the kinematics package the same way DHForwardKinematics does.
class KdlForwardKinematics : public IFKStrategy {
public:
    // urdf_xml is the robot's URDF as a string (already xacro-processed);
    // start_link/end_link name the chain to extract from the parsed tree.
    KdlForwardKinematics(const std::string& urdf_xml, const std::string& start_link,
                         const std::string& end_link);

    CartesianPosition computeCartesianPosition(const Eigen::VectorXd& joint_positions) const override;
    int numJoints() const override;

    const KDL::Chain& chain() const { return chain_; }

private:
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

}  // namespace kinematics
