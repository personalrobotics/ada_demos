#ifndef FEEDING_SHORTESTDISTANCERANKER_HPP_

#include "feeding/ranker/TargetFoodRanker.hpp"
#include "feeding/FoodItem.hpp"

namespace feeding{

/// Ranks items based on their distance to the endeffector
class ShortestDistanceRanker : public TargetFoodRanker
{
public:
    // Documention inherited.
    std::vector<FoodItem> sort(
        const std::vector<FoodItem>& items,
        const Eigen::Isometry3d& forqueTransform) const override;

    std::vector<FoodItemWithActionScorePtr> sort(
        const std::vector<FoodItemWithActionScorePtr>& items,
        const Eigen::Isometry3d& forqueTransform) const override;

    FoodItemWithActionScorePtr createFoodItemWithActionScore(
        const aikido::perception::DetectedObject& item) const override;
};

} // namespace feeding

#endif
