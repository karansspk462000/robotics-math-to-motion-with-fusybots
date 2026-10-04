#pragma once

#include "path_interpolator/interface/path_interpolator_interface.hpp"
#include "path_interpolator/common/pose_space.hpp"

namespace path_interpolator {

// Default IPathInterpolator: subdivides each segment of the input path into
// straight-line sub-waypoints, via blendPoint() (see point_blend.hpp) so it
// works identically for joint vectors and packed Cartesian poses - only
// space needs to change between the two, not the class.
class LinearPathInterpolator : public IPathInterpolator {
public:
    explicit LinearPathInterpolator(PoseSpace space = PoseSpace::kJoint, int points_per_segment = 25);

    std::vector<Eigen::VectorXd> interpolatePath(const std::vector<Eigen::VectorXd>& path) override;

private:
    PoseSpace space_;
    int points_per_segment_;
};

}  // namespace path_interpolator
