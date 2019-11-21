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
  TiltStyle tiltStyle(TiltStyle::NONE);

  // TODO: have a map of item -> strategy
  // if (item.getName() == "strawberry")
  // {
  //   tiltStyle = TiltStyle::VERTICAL;
  // }
  // if (item.getName() == "banana")
  // {
  //   tiltStyle = TiltStyle::ANGLED;
  // }

  // TODO: check if rotation and tilt angle should change
  AcquisitionAction action(tiltStyle, 0.0, 0.0, Eigen::Vector3d(0, 0, -1));

  auto itemPose = item.getMetaSkeleton()->getBodyNode(0)->getWorldTransform();
  double distance = getDistance(itemPose, forqueTransform);

  return std::unique_ptr<FoodItem>(new FoodItem(
      item.getName(), item.getUid(), item.getMetaSkeleton(), action, distance));
}

} // namespace feeding
