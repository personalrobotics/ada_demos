#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include <chrono>
#include <thread>
#include "feeding/util.hpp"

#include "feeding/action/MoveAboveFood.hpp"

namespace feeding {
namespace action {

std::unique_ptr<FoodItem> detectAndMoveAboveFood(
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
    std::vector<double> velocityLimits,
    FeedingDemo* feedingDemo)
{
  std::cout << "DetectAndMoveAboveFood" << std::endl;
  std::vector<std::unique_ptr<FoodItem>> candidateItems;
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(2));

    ROS_INFO_STREAM("Perceive");
    // Perception returns the list of good candidates, any one of them is good.
    // Multiple candidates are preferrable since planning may fail.
    candidateItems = perception->perceiveFood(foodName);

    if (candidateItems.size() == 0) {
      talk("I can't find that food. Try putting it on the plate.");
      ROS_WARN_STREAM("Failed to detect any food. Please place food on the plate.");
    }
    else
      break;
  }

  ROS_INFO_STREAM("Detected " << candidateItems.size() << " " << foodName);

  bool moveAboveSuccessful = false;

  auto& item = candidateItems[0];

  //for (auto& item : candidateItems)
  // {
    auto action = item->getAction();

    ROS_INFO_STREAM("Tilt style " << TiltStyleToString.at(action->getTiltStyle()) << " angle " << action->getRotationAngle() * 180 / 3.14 );
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
            velocityLimits,
            feedingDemo))
    {
      ROS_INFO_STREAM("Failed to move above " << item->getName());
      // ROS_INFO_STREAM("Try again with 180 degrees added in rotationa angle");

      // if (!moveAboveFood(
      //       ada,
      //       collisionFree,
      //       item->getName(),
      //       item->getPose(),
      //       action->getRotationAngle() + 3.141592,
      //       action->getTiltStyle(),
      //       heightAboveFood,
      //       horizontalTolerance,
      //       verticalTolerance,
      //       rotationTolerance,
      //       tiltTolerance,
      //       planningTimeout,
      //       maxNumTrials,
      //       velocityLimits,
      //       feedingDemo))
      // {
      //   ROS_INFO_STREAM("Failed to move above " << item->getName());
      // }
    }

    moveAboveSuccessful = true;
    perception->setFoodItemToTrack(item.get());
    std::cout << "set FoodItemToTrack succesful" << std::endl;
    return std::move(item);
  //}

  if (!moveAboveSuccessful)
  {
    ROS_ERROR("Failed to move above any food.");
    return nullptr;
  }
}
}
}