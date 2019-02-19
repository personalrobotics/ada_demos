#ifndef FEEDING_SUCCESSRATERANKER_HPP_

#include "feeding/ranker/TargetFoodRanker.hpp"

namespace feeding{

/// Ranks items based on their predicted success rate
class SuccessRateRanker : public TargetFoodRanker
{
public:
    /// Returns a sorted list of items.
    /// \param[in] items List of food items.
    /// \param[out] items List of food items.
    void sort(
        std::vector<std::unique_ptr<FoodItem>>& items) const override;

    std::unique_ptr<FoodItem> createFoodItem(
        const aikido::perception::DetectedObject& item,
        const Eigen::Isometry3d& forqueTransform) const override;
}

} // namespace feeding

#endif
