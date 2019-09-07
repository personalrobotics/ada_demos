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
        if (getUserInputWithOptions(optionPrompts, "Did I succeed in moving over the food?") == 1)
        {
          break;
        }
          talk("Adjusting, hold tight!", true);
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

    // ===== In Front of Person =====
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


    int loss = 1;

    if (getUserInputWithOptions(optionPrompts, "Did I succeed?") == 1)
    {
      ROS_INFO_STREAM("Successful");
      loss = 0;
    } else {
      ROS_INFO_STREAM("Failed");
    }

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
    ros::ServiceClient client = feedingDemo->getNodeHandle().serviceClient<conban_spanet::PublishLoss>("/conban_spanet_server/PublishLoss");
    conban_spanet::PublishLoss srv;
    srv.request.features.insert(std::end(srv.request.features), std::begin(features), std::end(features));
    srv.request.p_t.insert(std::end(srv.request.p_t), std::begin(p_t), std::end(p_t));
    srv.request.a_t = actionNum;
    srv.request.loss = loss;

    if (client.call(srv))
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

} // namespace action
} // namespace feeding
