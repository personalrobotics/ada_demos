#include "feeding/ranker/ShortestDistanceRanker.hpp"
#include "feeding/AcquisitionAction.hpp"
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
          getDistance(item.getPose(), forqueTransform));
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
  std::vector<FoodItemWithActionScorePtr> rankedFoodItems;

  std::vector<double> distancesToForque;
  for(const auto& item: items)
  {
      distancesToForque.emplace_back(
          getDistance(item->getItem()->getPose(), forqueTransform));
  }

  // TODO: Sort items based on the distance

  for(std::size_t i = 0; i < items.size(); ++i)
      rankedFoodItems.emplace_back(items[i]);

  return rankedFoodItems;
}

//==============================================================================
FoodItemWithActionScorePtr
ShortestDistanceRanker::createFoodItemWithActionScore(
    const aikido::perception::DetectedObject& item) const
{
    FoodItem foodItem(item.getName(), item.getUid(), item.getMetaSkeleton());

    // TODO: change this based on item class
    AcquisitionAction action(
        TiltStyle::NONE, 0.0, 0.0, Eigen::Vector3d(-1, 0, 0));

    return std::make_shared<FoodItemWithActionScore>(
      foodItem, action, 1.0);
}

} // namespace feeding
