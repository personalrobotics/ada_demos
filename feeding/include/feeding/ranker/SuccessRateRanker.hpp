#ifndef FEEDING_SUCCESSRATERANKER_HPP_

#include "feeding/ranker/TargetFoodRanker.hpp"

namespace feeding{

/// Ranks items based on their predicted success rate
class SuccessRateRanker : public TargetFoodRanker
{
public:
    // Documentation inherited.
    std::vector<FoodItem> sort(
        const std::vector<FoodItem>& items,
        const Eigen::Isometry3d& forqueTransform) const = 0;

    FoodItemWithActionScorePtr createFoodItemWithActionScore(
        const aikido::perception::DetectedObject& item) const = 0;
}

} // namespace feeding

#endif
