#include "path_interpolator/factory/path_interpolator_factory.hpp"

#include <stdexcept>

#include "path_interpolator/strategy/linear_path_interpolator.hpp"

namespace path_interpolator {

std::unique_ptr<IPathInterpolator> createPathInterpolator(const std::string& method,
                                                            int points_per_segment) {
    if (method == "linear") {
        return std::make_unique<LinearPathInterpolator>(PoseSpace::kJoint, points_per_segment);
    }
    if (method == "linear_cartesian") {
        return std::make_unique<LinearPathInterpolator>(PoseSpace::kCartesian, points_per_segment);
    }
    throw std::runtime_error("path_interpolator_factory: unknown path_interpolator_method '" +
                              method + "'");
}

}  // namespace path_interpolator
