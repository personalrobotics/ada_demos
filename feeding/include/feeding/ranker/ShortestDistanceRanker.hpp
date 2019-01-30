#ifndef FEEDING_SHORTESTDISTANCERANKER_HPP_

#include "feeding/ranker/TargetFoodRanker.hpp"

namespace feeding{

/// Ranks items based on their distance to the endeffector
class ShortestDistanceRanker : public TargetFoodRanker
{
public:
    // Documention inherited.
    std::vector<const TargetFoodItem> sort(
        const std::vector<const TargetFoodItem>& items,
        const Eigen::Isometry3d& forqueTransform) const = 0;
}

} // namespace feeding

#endif