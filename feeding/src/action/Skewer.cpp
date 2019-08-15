#include "feeding/action/Skewer.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"

#include <libada/util.hpp>

static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

namespace feeding {
namespace action {


//==============================================================================
// For Summer 2019 experiment
// Control robot verbosity (last argument)
bool skewer(
    int verbosityLevel,
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

  if (!abovePlaceSuccess)
  {
    if (verbosityLevel == INTERMEDIATE_VERBOSITY) {
      talk(ERROR_STATUS_MSG);
    }
    if (verbosityLevel == HIGH_VERBOSITY) {
      talk(ERROR_STATUS_MSG);
      talk("Move above plate failed. Mind if I get a little help?");
    }
    ROS_WARN_STREAM("Move above plate failed. Please restart");
    return false;
  }

  if (std::find(rotationFreeFoodNames.begin(),
      rotationFreeFoodNames.end(), foodName) !=
      rotationFreeFoodNames.end())
  {
    rotationToleranceForFood = M_PI;
  }

  double torqueThreshold = 2;
  if (ftThresholdHelper)
    ftThresholdHelper->setThresholds(foodSkeweringForces.at(foodName), torqueThreshold);

  bool detectAndMoveAboveFoodSuccess = true;
  Eigen::Vector3d endEffectorDirection(0, 0, -1);

  for (std::size_t trialCount = 0; trialCount < 3; ++trialCount)
  {
    // Why a for loop here?
    for(std::size_t i = 0; i < 2; ++i)
    {
      if (verbosityLevel != BASIC_VERBOSITY) {
        if(i == 0) {
          talk(std::string("Planning to the ") + foodName, true);
        }
        if(i == 1) {
          talk("Adjusting, hold tight!", true);
        }
      }
      ROS_INFO_STREAM("Detect and Move above food");
      auto item = detectAndMoveAboveFood(
          verbosityLevel,
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
          feedingDemo);

      // returns nullptr if unsuccessful
      // TODO: test this out
      // maybe redundant: detectAndMoveAboveFood already handles error
      if (!item)
      {
        if (verbosityLevel == INTERMEDIATE_VERBOSITY) {
          talk(ERROR_STATUS_MSG);
        }
        if (verbosityLevel == HIGH_VERBOSITY) {
          talk(ERROR_STATUS_MSG);
          talk("Failed to detect food");
        }
        return false;
      }

      auto tiltStyle = item->getAction()->getTiltStyle();
      if (tiltStyle == TiltStyle::ANGLED)
      {
        endEffectorDirection = Eigen::Vector3d(0.1, 0, -0.18);
        endEffectorDirection.normalize();
      }
      if (!item)
        detectAndMoveAboveFoodSuccess = false;
    }

    if (!detectAndMoveAboveFoodSuccess)
      return false;

    ROS_INFO_STREAM(
          "Getting " << foodName << "with " << foodSkeweringForces.at(foodName)
                     << "N with angle mode ");

    // ===== INTO FOOD =====
    if (verbosityLevel != BASIC_VERBOSITY) {
      talk("Here we go!", true);
    }
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
      if (verbosityLevel == INTERMEDIATE_VERBOSITY) {
        talk(ERROR_STATUS_MSG);
      }
      if (verbosityLevel == HIGH_VERBOSITY) {
        talk(ERROR_STATUS_MSG);
        talk("Move into food failed. Let me try again");
      }
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
    if (verbosityLevel == INTERMEDIATE_VERBOSITY) {
      talk(ERROR_STATUS_MSG);
    }
    if (verbosityLevel == HIGH_VERBOSITY && trialCount == 2) {
      talk(ERROR_STATUS_MSG);
      talk("Move out of food failed.");
    } else if (verbosityLevel == HIGH_VERBOSITY) {
      talk(ERROR_STATUS_MSG);
      talk("Move out of food failed. Let me try again.");
    }
    
  }
  return false;
}

} // namespace action
} // namespace feeding
