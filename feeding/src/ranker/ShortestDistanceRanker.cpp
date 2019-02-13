#include "feeding/ranker/ShortestDistanceRanker.hpp"
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
std::vector<FoodItem> ShortestDistanceRanker::sort(
        const std::vector<FoodItem>& items,
        const Eigen::Isometry3d& forqueTransform) const
{
    std::vector<FoodItem> rankedFoodItems;

    std::vector<double> distancesToForque;
    for(const auto& item: items)
    {
        distancesToForque.emplace_back(
            getDistance(item.pose, forqueTransform));
    }

    // Sort based on the distance

    // TODO
    for(std::size_t i = 0; i < items.size(); ++i)
        rankedFoodItems.emplace_back(items[i]);

    return rankedFoodItems;
}

//==============================================================================
std::vector<FoodItemWithActionScorePtr> ShortestDistanceRanker::sort(
        const std::vector<FoodItemWithActionScorePtr>& items,
        const Eigen::Isometry3d& forqueTransform) const
{
  // TODO
  std::vector<FoodItemWithActionScorePtr> outItems;
  return outItems;
}

} // namespace feeding
