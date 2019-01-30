#ifndef FEEDING_SUCCESSRATERANKER_HPP_

#include "feeding/ranker/TargetFoodRanker.hpp"

namespace feeding{

/// Ranks items based on their predicted success rate
class SuccessRateRanker : public TargetFoodRanker
{
public:
    // Documentation inherited.
    std::vector<const TargetFoodItem> sort(
        const std::vector<const TargetFoodItem>& items,
        const Eigen::Isometry3d& forqueTransform) const = 0;
}

} // namespace feeding

#endif