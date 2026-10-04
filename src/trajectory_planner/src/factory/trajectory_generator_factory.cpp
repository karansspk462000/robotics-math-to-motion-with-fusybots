#include "trajectory_planner/factory/trajectory_generator_factory.hpp"

#include <stdexcept>

#include "trajectory_planner/strategy/trapezoidal_trajectory_generator.hpp"

namespace trajectory_planner {

std::unique_ptr<ITrajectoryGenerator> createTrajectoryGenerator(const std::string& method, double time_step) {
    if (method == "trapezoidal") {
        return std::make_unique<TrapezoidalTrajectoryGenerator>(time_step);
    }
    throw std::runtime_error("trajectory_generator_factory: unknown trajectory_planner_method '" +
                              method + "'");
}

}  // namespace trajectory_planner
