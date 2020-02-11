// Acquire action to be made similar to Skewer. 

#include "feeding/action/Acquire.hpp"
#include "feeding/action/AcquireMoveAboveFood.hpp"
#include "feeding/action/DetectFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/AcquireMoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"
#include "feeding/FeedingDemo.hpp"

#include <libada/util.hpp>

static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

namespace feeding {
namespace action {

//==============================================================================
bool acquire(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Workspace>& workspace,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    const std::string& foodName,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    double heightAboveFood,
    double horizontalToleranceForFood,
    double verticalToleranceForFood,
    double rotationToleranceForFood,
    double tiltToleranceForFood,
    double moveOutofFoodLength,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    std::chrono::milliseconds waitTimeForFood,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits,
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper,
    std::vector<std::string> rotationFreeFoodNames,
    FeedingDemo* feedingDemo,
    const double incidentAngle,
    const double force,
    const double inFoodRotationAngle,
    const double exitAngle)
{
  ROS_INFO_STREAM("Move above plate");
  bool abovePlaceSuccess = moveAbovePlate(
      ada,
      collisionFree,
      plate,
      plateEndEffectorTransform,
      horizontalToleranceAbovePlate,
      verticalToleranceAbovePlate,
      rotationToleranceAbovePlate,
      planningTimeout,
      maxNumTrials,
      velocityLimits);

  if (!abovePlaceSuccess)
  {
    talk(
        "Sorry, I'm having a little trouble moving. Mind if I get a little "
        "help?");
    ROS_WARN_STREAM("Move above plate failed. Please restart");
    return false;
  }

  if (std::find(
          rotationFreeFoodNames.begin(), rotationFreeFoodNames.end(), foodName)
      != rotationFreeFoodNames.end())
  {
    rotationToleranceForFood = M_PI;
  }

  bool detectAndMoveAboveFoodSuccess = true;
  Eigen::Vector3d endEffectorDirection(0, sin(M_PI/2 - incidentAngle), -cos(M_PI/2 - incidentAngle)); // change this for incident angle

  for (std::size_t trialCount = 0; trialCount < 3; ++trialCount)
  {
    for (std::size_t i = 0; i < 2; ++i) 
    {
      if (i == 0)
      {
        talk(std::string("Planning to the ") + foodName, true);
      }
      if (i == 1)
      {
        if (getUserInputWithOptions(optionPrompts, "Did I succeed in moving over the food?") == 1)
        {
           break;
        }
          talk("Adjusting, hold tight!", true);
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }

      ROS_INFO_STREAM("Detect and Move above food");
      std::vector<std::unique_ptr<FoodItem>> candidateItems = detectFood(perception, foodName);

      // Move above food
      bool moveAboveSuccessful = false;
      std::unique_ptr<FoodItem> item;
      for (auto& foodItem : candidateItems)
      {
        auto action = foodItem->getAction();

        if (!acquireMoveAboveFood( // created new move above
                ada,
                collisionFree,
                foodItem->getName(),
                foodItem->getPose(),
                action->getRotationAngle(),
                action->getTiltStyle(),
                heightAboveFood,
                horizontalToleranceForFood,
                verticalToleranceForFood,
                rotationToleranceForFood,
                tiltToleranceForFood,
                planningTimeout,
                incidentAngle, // incident angle parameter
                maxNumTrials,
                velocityLimits,
                feedingDemo))
        {
          ROS_INFO_STREAM("Failed to move above " << foodItem->getName());
          talk("Sorry, I'm having a little trouble moving. Let's try again.");
          return false;
        }
        moveAboveSuccessful = true;

        perception->setFoodItemToTrack(item.get());
        item = std::move(foodItem);
      }

      if (!moveAboveSuccessful)
      {
        ROS_ERROR("Failed to move above any food.");
        talk("Failed, let me start from beginning");
        return false;
      }
      if (!item)
        detectAndMoveAboveFoodSuccess = false;
    }

    if (!detectAndMoveAboveFoodSuccess)
      return false;

    ROS_INFO_STREAM(
        "Getting " << foodName << " with " << force << "N force");

    double torqueThreshold = 2;
    if(force < 3 || force > 25) {
      ROS_INFO_STREAM("Force parameter is out of the range");
      return false;
    }

    if (ftThresholdHelper)
      ftThresholdHelper->setThresholds(
          force, torqueThreshold);

    // ===== INTO FOOD =====
    talk("Here we go!", true);
    auto moveIntoSuccess = acquireMoveInto(
        ada,
        perception,
        collisionFree,
        nodeHandle,
        TargetItem::FOOD,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance,
        endEffectorDirection,
        ftThresholdHelper);

    if (!moveIntoSuccess)
    {
      ROS_INFO_STREAM("Failed. Retry");
      talk("Sorry, I'm having a little trouble moving. Let me try again.");
      return false;
    }

    std::this_thread::sleep_for(waitTimeForFood);


    if(inFoodRotationAngle < 0 || inFoodRotationAngle > M_PI/4) {
      ROS_INFO_STREAM("in food rotation angle is invalid");
      return false;
    }
    // Rotate the fork in the food
    std::cout << "Rotating the fork in the food..." << std::endl;
    Eigen::Vector3d rotateForkDirection(0, 0, -1);
    ada->moveArmToEndEffectorOffset(
        rotateForkDirection,
        0.05,
        nullptr,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance);

    // ===== OUT OF FOOD =====
    if(exitAngle < M_PI/4 || exitAngle > M_PI/2) {
      ROS_INFO_STREAM("exit angle is invalid");
      return false;
    }
    Eigen::Vector3d direction(0, cos(exitAngle), sin(exitAngle)); // incorporate liftoff angle into direction vector
    std::cout << "Move out of: " << direction.transpose() << std::endl;
    
    moveOutOf( 
        ada,
        nullptr,
        TargetItem::FOOD,
        moveOutofFoodLength * 2.0,
        direction,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance,
        ftThresholdHelper);

    if (getUserInputWithOptions(optionPrompts, "Did I succeed?") == 1)
    {
      ROS_INFO_STREAM("Successful");
      return true;
    }

    ROS_INFO_STREAM("Failed.");
    talk("Oops, let me try again.");
  }
  return false;
}

} // namespace action
} // namespace feeding
