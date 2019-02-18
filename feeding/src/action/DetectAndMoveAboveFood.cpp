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
  // Perception returns the list of good candidates, any one of them is good.
  // Multiple candidates are preferrable since planning may fail.
  auto candidateItems = perception->perceiveFood(foodName);

  FoodItemWithActionScorePtr targetItemWithScore;

  if (candidateItems.size() == 0)
    throw std::runtime_error("Failed to detect any food.");

  ROS_INFO_STREAM("Detected " << candidateItems.size() << " " << foodName);

  bool moveAboveSuccessful = false;
  for(const auto& itemWithScore: candidateItems)
  {
    auto action = itemWithScore->getAction();
    auto item = itemWithScore->getItem();

    std::cout << "Tilt style " << action->getTiltStyle() << std::endl;
    if (!moveAboveFood(
      ada,
      collisionFree,
      item->getName(),
      item->getPose(),
      action->getRotationAngle(),
      action->getTiltStyle(),
      heightAboveFood,
      horizontalTolerance,
      verticalTolerance,
      rotationTolerance,
      tiltTolerance,
      planningTimeout,
      maxNumTrials,
      velocityLimits))
    {
      ROS_INFO_STREAM("Failed to move above " << item->getName());
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
  perception->setFoodItemToTrack(targetItemWithScore->getItem());
  return targetItemWithScore;
}

}
}