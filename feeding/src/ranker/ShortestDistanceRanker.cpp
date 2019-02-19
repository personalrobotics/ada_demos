#include "feeding/ranker/ShortestDistanceRanker.hpp"

#include <dart/common/StlHelpers.hpp>
#include "feeding/AcquisitionAction.hpp"
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
void ShortestDistanceRanker::sort(
    std::vector<std::unique_ptr<FoodItem>>& items) const
{
  // Ascending since score is the distance.
  TargetFoodRanker::sort(items, SORT_ORDER::ASCENDING);
}


//==============================================================================
std::unique_ptr<FoodItem> ShortestDistanceRanker::createFoodItem(
    const aikido::perception::DetectedObject& item,
    const Eigen::Isometry3d& forqueTransform) const
{
    // TODO: change this based on item class
    AcquisitionAction action(
        TiltStyle::NONE, 0.0, 0.0, Eigen::Vector3d(-1, 0, 0));

    auto itemPose = item.getMetaSkeleton()->getBodyNode(0)->getWorldTransform();
    double distance = getDistance(itemPose, forqueTransform);

    return dart::common::make_unique<FoodItem>(
      item.getName(),
      item.getUid(),
      item.getMetaSkeleton(),
      action,
      distance);
}


} // namespace feeding
