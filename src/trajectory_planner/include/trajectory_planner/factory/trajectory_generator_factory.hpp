#pragma once

#include <memory>
#include <string>

#include "trajectory_planner/interface/trajectory_generator_interface.hpp"

namespace trajectory_planner {

// Builds the ITrajectoryGenerator named by `method` (e.g. "trapezoidal").
// time_step is the generator's sample spacing (seconds) - callers driving a
// real playback timer must pass 1/playback_rate_hz so the two stay in sync
// (a mismatch causes real-world playback speed to drift from the commanded
// max_velocity/max_acceleration).
std::unique_ptr<ITrajectoryGenerator> createTrajectoryGenerator(const std::string& method,
                                                                  double time_step = 0.01);

}  // namespace trajectory_planner
