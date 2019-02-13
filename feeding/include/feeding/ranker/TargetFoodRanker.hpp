#ifndef FEEDING_TARGETFOODRANKER_HPP_
#define FEEDING_TARGETFOODRANKER_HPP_

#include <vector>
#include <Eigen/Core>
#include "feeding/FoodItem.hpp"

namespace feeding {

// Base class for ranking target food items.
class TargetFoodRanker
{
public:

    /// Returns a sorted list of items.
    /// \param[in] items List of food items.
    /// \param[in] forqueTransform Pose of the forque tine.
    virtual std::vector<FoodItem> sort(
        const std::vector<FoodItem>& items,
        const Eigen::Isometry3d& forqueTransform) const = 0;

    virtual std::vector<FoodItemWithActionScorePtr> sort(
        const std::vector<FoodItemWithActionScorePtr>& items,
        const Eigen::Isometry3d& forqueTransform) const = 0;
};

} // namespace feeding

#endif
