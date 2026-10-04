#pragma once

#include <memory>

#include "kinematics/interface/kinematics_interface.hpp"

namespace kinematics {

class Kinematics {
public:
    Kinematics() = default;
    virtual ~Kinematics() = default; //Abstract Class has virtual destructor because it has virtual functions. 
    // This ensures that the derived class's destructor is called when deleting an object through a base class pointer.
};


// Holds shared_ptr, not unique_ptr, deliberately: createFKStrategy()
// returns shared_ptr<IFKStrategy> because the SAME strategy object also
// has to be handed to createIKStrategy() (which dynamic_pointer_casts it
// to a concrete type - only possible on a shared_ptr, not a unique_ptr).
// A Context wrapping unique_ptr couldn't hold the output of that factory
// at all.
class ForwardKinematics : public Kinematics {
public:
    explicit ForwardKinematics(std::shared_ptr<IFKStrategy> strategy);
    ~ForwardKinematics() override = default;

    CartesianPosition solve(const Eigen::VectorXd& joint_positions) const;
    void setStrategy(std::shared_ptr<IFKStrategy> strategy);

private:
    std::shared_ptr<IFKStrategy> strategy_;
};

class InverseKinematics : public Kinematics {
public:
    explicit InverseKinematics(std::unique_ptr<IIKStrategy> strategy);
    ~InverseKinematics() override = default;

    Eigen::VectorXd solve(const CartesianPosition& target_pose,
                           const Eigen::VectorXd& initial_guess) const;
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& joint_positions) const;
    void setStrategy(std::unique_ptr<IIKStrategy> strategy);

private:
    std::unique_ptr<IIKStrategy> strategy_;
};

}  // namespace kinematics
