#include "path_interpolator/strategy/linear_path_interpolator.hpp"

#include <stdexcept>

#include "path_interpolator/common/point_blend.hpp"

namespace path_interpolator {

LinearPathInterpolator::LinearPathInterpolator(PoseSpace space, int points_per_segment)
    : space_(space), points_per_segment_(points_per_segment) {
    if (points_per_segment_ < 2) {
        throw std::invalid_argument("LinearPathInterpolator: points_per_segment must be >= 2");
    }
}

std::vector<Eigen::VectorXd> LinearPathInterpolator::interpolatePath(
    const std::vector<Eigen::VectorXd>& path) {
    if (path.size() < 2) {
        throw std::invalid_argument("LinearPathInterpolator: path needs >= 2 waypoints");
    }

    std::vector<Eigen::VectorXd> dense_path;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const Eigen::VectorXd& segment_start = path[i];
        const Eigen::VectorXd& segment_end = path[i + 1];
        const int points_to_add = (i + 2 == path.size()) ? points_per_segment_ : points_per_segment_ - 1;

        for (int j = 0; j < points_to_add; ++j) {
            const double t = static_cast<double>(j) / static_cast<double>(points_per_segment_ - 1);
            dense_path.push_back(blendPoint(segment_start, segment_end, t, space_));
        }
    }
    return dense_path;
}

}  // namespace path_interpolator
