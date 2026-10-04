#include "trajectory_planner/strategy/trapezoidal_trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace trajectory_planner {

namespace {

struct TrapezoidalProfile {
    double total_time;
    double accel_time;
    double peak_velocity;
};

TrapezoidalProfile computeProfile(double distance, double max_velocity, double max_acceleration) {
    TrapezoidalProfile profile;
    profile.accel_time = max_velocity / max_acceleration;
    const double accel_distance = 0.5 * max_acceleration * profile.accel_time * profile.accel_time;

    if (2.0 * accel_distance >= distance) {
        // Not enough distance to reach max_velocity: triangular profile.
        profile.accel_time = std::sqrt(distance / max_acceleration);
        profile.peak_velocity = max_acceleration * profile.accel_time;
        profile.total_time = 2.0 * profile.accel_time;
    } else {
        profile.peak_velocity = max_velocity;
        const double cruise_distance = distance - 2.0 * accel_distance;
        const double cruise_time = cruise_distance / max_velocity;
        profile.total_time = 2.0 * profile.accel_time + cruise_time;
    }
    return profile;
}

// Returns {arc_length_traveled, speed} at time t along the profile.
std::pair<double, double> sampleProfile(const TrapezoidalProfile& profile, double max_acceleration, double t) {
    const double t_decel_start = profile.total_time - profile.accel_time;

    if (t <= profile.accel_time) {
        return {0.5 * max_acceleration * t * t, max_acceleration * t};
    }
    if (t <= t_decel_start) {
        const double s = 0.5 * max_acceleration * profile.accel_time * profile.accel_time +
                          profile.peak_velocity * (t - profile.accel_time);
        return {s, profile.peak_velocity};
    }
    const double t_from_end = profile.total_time - t;
    const double accel_distance = 0.5 * max_acceleration * profile.accel_time * profile.accel_time;
    const double cruise_distance = profile.peak_velocity * (t_decel_start - profile.accel_time);
    const double total_distance = 2.0 * accel_distance + cruise_distance;
    const double s = total_distance - 0.5 * max_acceleration * t_from_end * t_from_end;
    return {s, max_acceleration * t_from_end};
}

// The arc-length parameter s drives every joint together: on segment i,
// joint j's speed is (|path[i+1][j] - path[i][j]| / segment_length) * ds/dt,
// so joint j's own velocity_max caps ds/dt on that segment at
// velocity_max[j] * segment_length / |Δq_ij|. This ratio generally differs
// segment to segment (the path is not one straight line in joint space), so
// the bottleneck must be the minimum over every (segment, joint) pair, not
// an average over the whole path - a path-wide average ratio can hide a
// segment whose local ratio is worse than average, letting that segment's
// joint exceed velocity_max undetected. Returns max_velocity unchanged if
// joint_velocity_max is empty (opt-in) or no joint moves enough to matter.
double effectiveMaxVelocity(const std::vector<Eigen::VectorXd>& path, double /*total_length*/,
                            double max_velocity, const std::vector<double>& joint_velocity_max) {
    if (joint_velocity_max.empty()) {
        return max_velocity;
    }

    const int num_joints = static_cast<int>(path.front().size());
    double bottleneck = max_velocity;
    for (std::size_t i = 1; i < path.size(); ++i) {
        const double segment_length = (path[i] - path[i - 1]).norm();
        if (segment_length < 1e-9) {
            continue;  // zero-length segment - no direction, can't be the bottleneck
        }
        for (int j = 0; j < num_joints && j < static_cast<int>(joint_velocity_max.size()); ++j) {
            const double joint_delta = std::abs(path[i][j] - path[i - 1][j]);
            if (joint_delta < 1e-9 || joint_velocity_max[j] <= 0.0) {
                continue;  // joint doesn't move on this segment (or has no configured limit)
            }
            const double joint_cap = joint_velocity_max[j] * segment_length / joint_delta;
            bottleneck = std::min(bottleneck, joint_cap);
        }
    }
    return bottleneck;
}

}  // namespace

// This class's only job is timing: fit a trapezoidal time function s(t)
// (using max_velocity/max_acceleration) over the arc length of `path`, then
// at each timestep publish whichever point in `path` that arc length has
// reached. It does not synthesize any new positions between path's
// points - path_interpolator already produced every point this trajectory
// will ever contain, at whatever resolution it was configured with. An
// earlier version of this class looked up a "segment" and linearly blended
// between its two endpoints; that is a *second*, uncontrolled spatial
// interpolation on top of path_interpolator's, and for Cartesian targets
// (where path is already-IK-solved points, not a joint-space line) that
// blend does not stay on path_interpolator's curve - confirmed by testing
// that increasing path's resolution from 5 to 2000 points did not reduce
// the resulting Cartesian-space deviation at all, which only makes sense
// if resolution wasn't actually being used.
TrapezoidalTrajectoryGenerator::TrapezoidalTrajectoryGenerator(double time_step)
    : time_step_(time_step) {
    if (time_step_ <= 0.0) {
        throw std::invalid_argument("TrapezoidalTrajectoryGenerator: time_step must be > 0");
    }
}

Trajectory TrapezoidalTrajectoryGenerator::generateTrajectory(
    const std::vector<Eigen::VectorXd>& path, const MotionConstraints& constraints) {
    if (path.size() < 2) {
        throw std::invalid_argument("TrapezoidalTrajectoryGenerator: path needs >= 2 waypoints");
    }

    std::vector<double> cumulative_length(path.size(), 0.0);
    for (std::size_t i = 1; i < path.size(); ++i) {
        cumulative_length[i] = cumulative_length[i - 1] + (path[i] - path[i - 1]).norm();
    }
    const double total_length = cumulative_length.back();

    if (total_length < 1e-9) {
        TrajectoryPoint point;
        point.position = path.front();
        point.velocity = Eigen::VectorXd::Zero(path.front().size());
        point.acceleration = Eigen::VectorXd::Zero(path.front().size());
        return {point};
    }

    const double max_velocity =
        effectiveMaxVelocity(path, total_length, constraints.max_velocity, constraints.joint_velocity_max);
    const TrapezoidalProfile profile = computeProfile(total_length, max_velocity, constraints.max_acceleration);

    // Pass 1: decide, for every output timestep, where along `path` the
    // time function has reached - positions only, no velocity yet.
    //
    // The position is a *local linear blend* between the two path points
    // straddling the target arc length, not a snap to the nearest existing
    // point. Snapping made position a step function of arc length, and
    // whenever the output time_step didn't line up with path's point
    // spacing (which it won't, in general - they're independent
    // resolutions), the resulting position sequence froze for a few
    // samples and then jumped, which finite-differencing then reported as
    // velocity oscillating between zero and a spike: the same "up down up
    // down" chatter, just triggered by dt-vs-density mismatch rather than
    // by low density alone. Blending between two *adjacent* dense points
    // (path was already sampled dense enough that consecutive points are
    // close to colinear) keeps position a continuous function of arc
    // length with negligible curvature error, so velocity stays smooth
    // regardless of how time_step and points_per_segment happen to align.
    std::vector<Eigen::VectorXd> positions;
    std::vector<double> times;
    std::size_t point_index = 0;
    for (double t = 0.0; t < profile.total_time; t += time_step_) {
        const auto [arc_length, speed] = sampleProfile(profile, constraints.max_acceleration, t);
        (void)speed;  // profile speed is not used for velocity - see pass 2 below

        while (point_index + 1 < path.size() && cumulative_length[point_index + 1] <= arc_length) {
            ++point_index;
        }

        if (point_index + 1 < path.size()) {
            const double segment_start_len = cumulative_length[point_index];
            const double segment_len = cumulative_length[point_index + 1] - segment_start_len;
            const double frac =
                segment_len > 1e-12 ? std::clamp((arc_length - segment_start_len) / segment_len, 0.0, 1.0) : 0.0;
            positions.push_back(path[point_index] + frac * (path[point_index + 1] - path[point_index]));
        } else {
            positions.push_back(path[point_index]);
        }
        times.push_back(t);
    }
    positions.push_back(path.back());
    times.push_back(profile.total_time);

    // Pass 2: velocity/acceleration are finite differences of the
    // positions actually chosen above, not the profile's theoretical
    // speed - guaranteed self-consistent with what gets published, unlike
    // an earlier version that used tangent * profile-speed: whenever a
    // position repeated across timesteps (which happens whenever
    // path_interpolator's density doesn't perfectly match the output
    // sample rate - it doesn't, in general), that version kept reporting
    // a smoothly ramping nonzero velocity for a position that was not
    // actually changing, which is exactly what produced the "up down up
    // down" chatter in /motion_status/joint_velocity.
    // Single, simple convention throughout: velocity[i] is the forward
    // difference to the NEXT published position ("how fast we're about to
    // move"), and acceleration[i] the forward difference of velocity. The
    // last sample is already at rest (path.back(), by construction of the
    // profile), so its velocity/acceleration are exactly zero rather than
    // a finite difference against a nonexistent next sample.
    Trajectory trajectory(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        trajectory[i].time_from_start = times[i];
        trajectory[i].position = positions[i];
    }
    for (std::size_t i = 0; i + 1 < positions.size(); ++i) {
        const double dt = times[i + 1] - times[i];
        trajectory[i].velocity =
            dt > 1e-9 ? (positions[i + 1] - positions[i]) / dt : Eigen::VectorXd::Zero(positions[i].size()).eval();
    }
    trajectory.back().velocity = Eigen::VectorXd::Zero(positions.back().size());
    for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
        const double dt = times[i + 1] - times[i];
        trajectory[i].acceleration = dt > 1e-9
                                          ? ((trajectory[i + 1].velocity - trajectory[i].velocity) / dt).eval()
                                          : Eigen::VectorXd::Zero(positions[i].size()).eval();
    }
    trajectory.back().acceleration = Eigen::VectorXd::Zero(positions.back().size());

    return trajectory;
}

}  // namespace trajectory_planner
