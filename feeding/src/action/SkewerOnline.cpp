#include "feeding/action/Skewer.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/FoodItem.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"

#include "conban_spanet/PublishLoss.h"

#include <libada/util.hpp>

static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

namespace feeding {
namespace action {

//==============================================================================
bool skewerOnline(
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
  double torqueThreshold = 2;
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

  // Re-tare force-torque sensor
  if (ftThresholdHelper) {
      ftThresholdHelper->setThresholds(
          -2, torqueThreshold);
      ftThresholdHelper->setThresholds(STANDARD_FT_THRESHOLD);
    }


  // Pause a bit so camera can catch up
  if(velocityLimits[0] > 0.5) {
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (std::find(
          rotationFreeFoodNames.begin(), rotationFreeFoodNames.end(), foodName)
      != rotationFreeFoodNames.end())
  {
    rotationToleranceForFood = M_PI;
  }

  bool detectAndMoveAboveFoodSuccess = true;
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
          std::this_thread::sleep_for(std::chrono::milliseconds(4000));

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
            ROS_WARN_STREAM("WORKING");
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
            endEffectorDirection += ((-0.016 * forkYAxis) + (0.007 * forkXAxis));
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
            endEffectorDirection += ((-0.015 * forkYAxis) + (0.005 * forkXAxis));
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
          feedingDemo);

      if (!item)
      {
        talk("Failed, let me start from the beginning");
        return false;
      }

      if (!item)
        detectAndMoveAboveFoodSuccess = false;
    }

    if (!detectAndMoveAboveFoodSuccess)
      return false;

    ROS_INFO_STREAM(
        "Getting " << foodName << "with " << foodSkeweringForces.at(foodName)
                   << "N with angle mode ");

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
        heightAboveFood,
        direction,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance,
        ftThresholdHelper);

    // Wait for 5s for falling off
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // ===== In Front of Person =====
    /*
    talk("Testing move to person.", true);
    moveInFrontOfPerson(
          ada,
          collisionFree,
          workspace->getPersonPose(),
          feedingDemo->mPersonTSRParameters.at("distance"),
          feedingDemo->mPersonTSRParameters.at("horizontalTolerance"),
          feedingDemo->mPersonTSRParameters.at("verticalTolerance"),
          planningTimeout,
          maxNumTrials,
          velocityLimits);
    */


    int loss = 1;
    talk("Record success.", true);
    if (getUserInputWithOptions(optionPrompts, "Did I succeed?") == 1)
    {
      ROS_INFO_STREAM("Successful");
      loss = 0;
    } else {
      ROS_INFO_STREAM("Failed");
    }

    if(feedingDemo->mIsOnlineDemo) {
      // Publish Loss to algorithm
      YAML::Node node = item->getExtraInfo();
      std::vector<double> features = node["features"].as<std::vector<double>>();
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

      std::vector<double> p_t = item->mAnnotation;

      // Actually call service
      conban_spanet::PublishLoss srv;
      srv.request.features.insert(std::end(srv.request.features), std::begin(features), std::end(features));
      srv.request.p_t.insert(std::end(srv.request.p_t), std::begin(p_t), std::end(p_t));
      srv.request.a_t = actionNum;
      srv.request.loss = loss;

      if (ros::service::call("PublishLoss", srv))
      {
        ROS_INFO("Success! Error Message: %d", (int)srv.response.success);
      }
      else
      {
        ROS_ERROR("Failed to call service publish_loss");
        return false;
      }
      return srv.response.success;
    }
    return (loss == 0);
}

} // namespace action
} // namespace feeding
