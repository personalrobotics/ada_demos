#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/MoveAboveFood.hpp"


namespace feeding{
namespace action{

FoodItemWithActionScorePtr detectAndMoveAboveFood(
  const std::shared_ptr<ada::Ada>& ada,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  const std::shared_ptr<Perception>& perception,
  const std::string& foodName,
  double heightAboveFood,
  double horizontalTolerance,
  double verticalTolerance,
  double rotationTolerance,
  double tiltTolerance,
  double planningTimeout,
  int maxNumTrials,
  std::vector<double> velocityLimits)
{
  auto candidateItems = perception->perceiveFood(foodName);

  FoodItemWithActionScorePtr targetItemWithScore;

  if (candidateItems.size() == 0)
    throw std::runtime_error("Failed to detect any food.");

  bool moveAboveSuccessful = false;
  for(const auto& itemWithScore: candidateItems)
  {
    auto action = itemWithScore->action;
    auto item = itemWithScore->item;

    if (!moveAboveFood(
      ada,
      collisionFree,
      item.name,
      item.pose,
      action.rotationAngle,
      action.tiltStyle,
      heightAboveFood,
      horizontalTolerance,
      verticalTolerance,
      rotationTolerance,
      tiltTolerance,
      planningTimeout,
      maxNumTrials,
      velocityLimits))
    {
      ROS_INFO_STREAM("Failed to move above " << item.name);
      continue;
    }
    moveAboveSuccessful = true;
    targetItemWithScore = itemWithScore;
    break;
  }

  if (!moveAboveSuccessful)
  {
    ROS_ERROR("Failed to move above any food.");
    throw std::runtime_error("Failed to move above any food.");
  }
  perception->setFoodItemToTrack(targetItemWithScore->item);
  return targetItemWithScore;
}

}
}