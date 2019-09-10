#include "feeding/action/Skewer.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"
#include "feeding/FeedingDemo.hpp"

#include <libada/util.hpp>

using ada::util::getRosParam;

static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

namespace feeding {
namespace action {

//==============================================================================
bool skewer(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Workspace>& workspace,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    const std::string& foodName,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    const std::unordered_map<std::string, double>& foodSkeweringForces,
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
    FeedingDemo* feedingDemo)
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

  // Pause a bit so camera can catch up
  if(velocityLimits[0] > 0.5) {
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

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

  int actionOverride = -1;

  for (std::size_t trialCount = 0; trialCount < 3; ++trialCount)
  {
    Eigen::Vector3d endEffectorDirection(0, 0, -1);
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

      if (!getRosParam<bool>("/humanStudy/autoAcquisition", feedingDemo->getNodeHandle()) && i == 0)
      {
          // Read Action from Topic
          talk("How should I pick up the food?", false);
          std::string actionName;
          std::string actionTopic;
          feedingDemo->getNodeHandle().param<std::string>("/humanStudy/actionTopic", actionTopic, "/study_action_msgs");
          actionName = getInputFromTopic(actionTopic, feedingDemo->getNodeHandle(), false, -1);
          talk("Alright, let me use " + actionName, true);

          if (actionName == "cross_skewer") {
            actionOverride = 1;
          } else if (actionName == "tilt") {
            actionOverride = 2;
          } else if (actionName == "cross_tilt") {
            actionOverride = 3;
          } else if (actionName == "angle") {
            actionOverride = 4;
          } else if (actionName == "cross_angle"){
            actionOverride = 5;
          } else {
            actionOverride = 0;
          }
      }

      auto item = detectAndMoveAboveFood(
          ada,
          collisionFree,
          perception,
          foodName,
          heightAboveFood,
          horizontalToleranceForFood,
          verticalToleranceForFood,
          rotationToleranceForFood,
          tiltToleranceForFood,
          planningTimeout,
          maxNumTrials,
          velocityLimits,
          feedingDemo,
          actionOverride);

      if (!item)
      {
        talk("Failed, let me start from the beginning");
        return false;
      }

      auto tiltStyle = item->getAction()->getTiltStyle();
      if (tiltStyle == TiltStyle::ANGLED)
      {
        endEffectorDirection = Eigen::Vector3d(0.1, 0, -0.18);
        endEffectorDirection.normalize();
      }
      if (!item) {
        detectAndMoveAboveFoodSuccess = false;
      }

      // Add error if autonomous
      if(getRosParam<bool>("/humanStudy/autoAcquisition", feedingDemo->getNodeHandle()) && // autonomous
        getRosParam<bool>("/humanStudy/createError", feedingDemo->getNodeHandle()) && // add error
        trialCount == 0) // First Trial
      { 
        ROS_WARN_STREAM("Error Requested for Acquisition!");
        endEffectorDirection(1) += 1.0;
        endEffectorDirection.normalize();
      }
    }

    if (!detectAndMoveAboveFoodSuccess)
      return false;

    ROS_INFO_STREAM(
        "Getting " << foodName << "with " << foodSkeweringForces.at(foodName)
                   << "N with angle mode ");

    double torqueThreshold = 2;
    if (ftThresholdHelper)
      ftThresholdHelper->setThresholds(
          foodSkeweringForces.at(foodName), torqueThreshold);

    // ===== INTO FOOD =====
    talk("Here we go!", true);
    auto moveIntoSuccess = moveInto(
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

    // ===== OUT OF FOOD =====
    Eigen::Vector3d direction(0, 0, 1);
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
