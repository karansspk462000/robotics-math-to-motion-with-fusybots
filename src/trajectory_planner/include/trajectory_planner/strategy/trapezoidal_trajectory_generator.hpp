#pragma once

#include "trajectory_planner/interface/trajectory_generator_interface.hpp"

namespace trajectory_planner {

// Default ITrajectoryGenerator: time-parameterizes a joint-space path with a
// single synchronized trapezoidal velocity profile along the path's arc
// length, respecting MotionConstraints::max_velocity/max_acceleration.
class TrapezoidalTrajectoryGenerator : public ITrajectoryGenerator {
public:
    explicit TrapezoidalTrajectoryGenerator(double time_step = 0.01);

    Trajectory generateTrajectory(const std::vector<Eigen::VectorXd>& path,
                                   const MotionConstraints& constraints) override;

private:
    double time_step_;
};

}  // namespace trajectory_planner
