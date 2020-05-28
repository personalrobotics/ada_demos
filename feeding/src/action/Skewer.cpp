#include "ada_demos/DetectAcquisition.h"
#include "feeding/action/Skewer.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/onFork.hpp"
#include <libada/util.hpp>

using ada::util::getRosParam;

static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};
static const std::vector<std::string> actionPrompts{"(1) skewer", "(3) tilt", "(5) angle"};

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

  if (!abovePlaceSuccess)
  {
    talk(
        "Sorry, I'm having a little trouble moving. Mind if I get a little "
        "help?");
    ROS_WARN_STREAM("Move above plate failed. Please restart");
    return false;
  }

  bool detectAndMoveAboveFoodSuccess = true;

  int actionOverride = -1;

  if (!getRosParam<bool>("/humanStudy/autoAcquisition", *(feedingDemo->getNodeHandle())))
      {
          // Read Action from Topic
          talk("How should I pick up the food?", true);
          ROS_INFO_STREAM("Waiting for action...");
          std::string actionName;
          std::string actionTopic;
          feedingDemo->getNodeHandle()->param<std::string>("/humanStudy/actionTopic", actionTopic, "/study_action_msgs");
          actionName = getInputFromTopic(actionTopic, *(feedingDemo->getNodeHandle()), false, -1);
          talk("Alright, let me use " + actionName, false);

          if (actionName == "skewer") {
            actionOverride = 1;
          } else if (actionName == "vertical") {
            actionOverride = 1;
          }else if (actionName == "cross_skewer") {
            actionOverride = 1;
          } else if (actionName == "tilt") {
            actionOverride = 3;
          } else if (actionName == "cross_tilt") {
            actionOverride = 3;
          } else if (actionName == "angle") {
            actionOverride = 5;
          } else if (actionName == "cross_angle"){
            actionOverride = 5;
          } else {
            actionOverride = getUserInputWithOptions(actionPrompts, "Didn't get valid action. Choose manually:");
            if (actionOverride > 5 || actionOverride < 0) {
              actionOverride = 1;
            }
          }
      }

    if (std::find(
          rotationFreeFoodNames.begin(), rotationFreeFoodNames.end(), foodName)
      != rotationFreeFoodNames.end())
  {
    rotationToleranceForFood = M_PI;
    if (actionOverride == 1) {
      actionOverride = 0;
    }
  }

    // Pause a bit so camera can catch up
    /*
    if(velocityLimits[0] > 0.5) {
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    */
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

  for (std::size_t trialCount = 0; trialCount < 3; ++trialCount)
  {

    Eigen::Vector3d endEffectorDirection(0, 0, -1);
    std::unique_ptr<FoodItem> item;
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (i == 0)
      {
        talk(std::string("Planning to the ") + foodName, true);
      }
      if (i == 1)
      {
          talk("Adjusting, hold tight!", true);
          std::this_thread::sleep_for(std::chrono::milliseconds(500));

          // Set Action Override
          auto action = item->getAction();
          int actionNum = 0;
          switch(action->getTiltStyle()) {
            case TiltStyle::ANGLED:
            actionNum = 4;
            break;
            case TiltStyle::VERTICAL:
            actionNum = 2;
            break;
            default:
            actionNum = 0;
          }
          if (action->getRotationAngle() > 0.01) {
            // Assume 90-degree action
            actionNum++;
          }
          // Call here so we don't overwrite features
          Eigen::Vector3d foodVec = item->getPose().rotation() * Eigen::Vector3d::UnitX();
          double baseRotateAngle = atan2(foodVec[1], foodVec[0]);
          detectAndMoveAboveFood(
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
            &baseRotateAngle,
            actionNum);
          auto tiltStyle = item->getAction()->getTiltStyle();
          if (tiltStyle == TiltStyle::ANGLED)
          {
            // Apply base rotation of food
            Eigen::Isometry3d eePose = ada->getHand()->getEndEffectorBodyNode()->getTransform();
            Eigen::Vector3d newEEDir = eePose.rotation() * Eigen::Vector3d::UnitZ();
            newEEDir[2] = sqrt(pow(newEEDir[0], 2.0) + pow(newEEDir[1], 2.0)) * (-0.2 / 0.1);
            endEffectorDirection = newEEDir;
            endEffectorDirection.normalize();

            Eigen::Vector3d forkXAxis = eePose.rotation() * Eigen::Vector3d::UnitX();
            forkXAxis[2] = 0.0;
            forkXAxis.normalize();
            endEffectorDirection *= heightAboveFood;
            endEffectorDirection += (0.01 * forkXAxis);
            endEffectorDirection.normalize();

          }
          else if (tiltStyle == TiltStyle::NONE)
          {
            // Apply base rotation of food
            Eigen::Isometry3d eePose = ada->getHand()->getEndEffectorBodyNode()->getTransform();
            Eigen::Vector3d forkYAxis = eePose.rotation() * Eigen::Vector3d::UnitY();
            forkYAxis[2] = 0.0;
            forkYAxis.normalize();
            Eigen::Vector3d forkXAxis = eePose.rotation() * Eigen::Vector3d::UnitX();
            forkXAxis[2] = 0.0;
            forkXAxis.normalize();
            endEffectorDirection *= heightAboveFood;
            endEffectorDirection += ((-0.02 * forkYAxis) + (-0.005 * forkXAxis));
            endEffectorDirection.normalize();
          }
          else if (tiltStyle == TiltStyle::VERTICAL)
          {
            // Apply base rotation of food
            Eigen::Isometry3d eePose = ada->getHand()->getEndEffectorBodyNode()->getTransform();
            Eigen::Vector3d forkYAxis = eePose.rotation() * Eigen::Vector3d::UnitY();
            Eigen::Vector3d forkXAxis = eePose.rotation() * Eigen::Vector3d::UnitX();
            forkXAxis[2] = 0.0;
            forkXAxis.normalize();
            forkYAxis[2] = 0.0;
            forkYAxis.normalize();
            endEffectorDirection *= heightAboveFood;
            endEffectorDirection += ((-0.025 * forkYAxis) + (-0.01 * forkXAxis));
            endEffectorDirection.normalize();
          }
          break;
      }

      ROS_INFO_STREAM("Detect and Move above food");
      item = detectAndMoveAboveFood(
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
          nullptr,
          actionOverride);

      if (!item)
      {
        talk("Failed, let me start from the beginning");
        return false;
      }

      if (!item) {
        detectAndMoveAboveFoodSuccess = false;
      }

      // Add error if autonomous
      if(getRosParam<bool>("/humanStudy/autoAcquisition", *(feedingDemo->getNodeHandle())) && // autonomous
        getRosParam<bool>("/humanStudy/createError", *(feedingDemo->getNodeHandle())) && // add error
        trialCount == 0) // First Trial
      {
        ROS_WARN_STREAM("Error Requested for Acquisition!");
        endEffectorDirection(1) -= 1.0;
        endEffectorDirection.normalize();
      }
    }

    if (!detectAndMoveAboveFoodSuccess)
      return false;

    ROS_INFO_STREAM(
        "Getting " << foodName << " with " << foodSkeweringForces.at(foodName)
                   << "N with angle mode ");

    double torqueThreshold = 2;
    if (ftThresholdHelper)
      ftThresholdHelper->setThresholds(
          foodSkeweringForces.at(foodName), torqueThreshold);

    // ===== COLLECT FORQUE DATA =====
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ROS_INFO_STREAM("Collecting forque data before action.");
    int numDataPts = 400;
    Eigen::Vector3d beforeForceAvg;
    Eigen::Vector3d beforeTorqueAvg;
    bool ftTimeout = false;

    if (ftThresholdHelper)
    {
      bool canCollect = ftThresholdHelper->startDataCollection(numDataPts);
      if (canCollect)
      {
        ros::Time start_time = ros::Time::now();
        ros::Duration timeout(10.0); // Timeout of 10 seconds
        while (!ftThresholdHelper->isDataCollectionFinished(beforeForceAvg,
              beforeTorqueAvg)) {
          ftTimeout = ros::Time::now() - start_time > timeout;
          if (ftTimeout)
            break;
        }
        if (ftTimeout) {
          ROS_INFO_STREAM("FT data collection failed. Use Vision only.");
        }
        else
        {
          ROS_INFO_STREAM("Done with FT data collection.");
          ROS_INFO_STREAM("Before average z force: " << beforeForceAvg.z());
        }
      }
    }

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
        (feedingDemo->isAdaReal()) ? ftThresholdHelper : nullptr);

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
        (feedingDemo->isAdaReal()) ? ftThresholdHelper : nullptr);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ===== COLLECT FORQUE DATA =====
    Eigen::Vector3d afterForceAvg;
    Eigen::Vector3d afterTorqueAvg;
    double zForceAvgDiff = 0.0;
    
    if (ftThresholdHelper && !ftTimeout)
    {
      bool canCollect = ftThresholdHelper->startDataCollection(numDataPts);
      if (canCollect)
      {
        ROS_INFO_STREAM("Collecting forque data after action.");
        while (!ftThresholdHelper->isDataCollectionFinished(afterForceAvg,
              afterTorqueAvg)) { }
        ROS_INFO_STREAM("Done with data collection.");
        ROS_INFO_STREAM("After average z force: " << afterForceAvg.z());
        // Use only z-force
        if (!feedingDemo->isAdaReal())
        {
        zForceAvgDiff = afterForceAvg.z();  // use in simulation
        }
        else
        {
        zForceAvgDiff = afterForceAvg.z() - beforeForceAvg.z();  // use in real
        }
        ROS_INFO_STREAM("Difference between average z forces: " << zForceAvgDiff);
      }
      else
      {
        ROS_INFO_STREAM("Failure in collecting FT data");
      }
    }

    // ===== CALL TO SERVICE ====
    ada_demos::DetectAcquisition srv;
    ROS_INFO_STREAM("Calling service...");
    int visualRes = -1;
    if (ros::service::call("acquisition_detector", srv))
    {
      ROS_INFO_STREAM("Success in calling Vision service.");
      visualRes = srv.response.success;
      ROS_INFO_STREAM("Visual system says: " << visualRes);
    }
    else
    {
      ROS_INFO_STREAM("Failure in calling Vision service.");
    }

    int combinedRes = isFoodOnFork(visualRes, zForceAvgDiff, numDataPts);
    if (combinedRes > 0)
    {
      ROS_INFO_STREAM("Successful in picking up food.");
      return true;
    }
    else if (combinedRes < 0)
    {
      ROS_INFO_STREAM("Failed to pick up food.");
      return false;
    }
    else
    {
      ROS_INFO_STREAM("Unable to determine");
      talk("Let me try again.");
    }
  }

  ROS_INFO_STREAM("Three trials failed. Asking for help.");
  talk("Please helper me determine whether I successfully picked up food.");

  if (getUserInputWithOptions(optionPrompts, "Did I succeed?") == 1)
  {
    ROS_INFO_STREAM("Successful");
    talk("Success");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Failed.");
    talk("Failed");
    return false;
  }
}

} // namespace action
} // namespace feeding
