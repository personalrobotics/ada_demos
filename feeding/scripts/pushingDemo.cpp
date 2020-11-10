#include <ros/ros.h>
#include <libada/util.hpp>
#include <sstream>
#include "std_msgs/String.h"

#include "feeding/FeedingDemo.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveAboveFood.hpp"
#include "feeding/action/MoveAbove.hpp"

#include "feeding/util.hpp"
#include "feeding/FTThresholdHelper.hpp"

#include "feeding/action/PushOnFood.hpp"
#include "feeding/action/MoveAboveAndScoop.hpp"
#include "feeding/action/PolyScoop.hpp"

#include "feeding/FoodItem.hpp"

#include "aikido/perception/shape_conversions.hpp"
#include <yaml-cpp/exceptions.h>

using ada::util::waitForUser;
using ada::util::chooseMode;
using ada::util::getRosParam;

namespace feeding {

void pushingDemo(
  FeedingDemo& feedingDemo,
  std::shared_ptr<Perception>& perception,
  ros::NodeHandle nodeHandle)
{
  // TODO: positioning the hand above the plate
  ROS_INFO_STREAM("========== Pushing DEMO ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  std::shared_ptr<FTThresholdHelper> FTThresholdHelper = feedingDemo.getFTThresholdHelper();

  while (true)
  {
    waitForUser("Start?", ada);

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running Pushing demo");

    // ===== MOVE ABOVE PLATE =====
    ROS_INFO_STREAM("Move above plate");
    bool abovePlaceSuccess = action::moveAbovePlate(
        ada,
        collisionFree,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters["horizontalTolerance"],
        feedingDemo.mPlateTSRParameters["verticalTolerance"],
        feedingDemo.mPlateTSRParameters["rotationTolerance"],
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits);

    // if (!abovePlaceSuccess)
    // {
    //   ROS_WARN_STREAM("Move above plate failed. Please restart");
    //   exit(EXIT_FAILURE);
    // }
    // else
    // {
    //   std::cout << "Move above Place Success" << std::endl;
    // }

    std::vector<std::unique_ptr<FoodItem>> candidateItems;
    while (true)
    {
      // Perception returns the list of good candidates, any one of them is good.
      // Multiple candidates are preferrable since planning may fail.
      candidateItems = perception->perceiveFood("onebite_mpotato");

      if (candidateItems.size() == 0)
      {
        talk("I can't find that food. Try putting it on the plate.");
        ROS_WARN_STREAM(
            "Failed to detect any food. Please place food on the plate.");
      }
      else
        break;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto& item = candidateItems[0];
    perception->setFoodItemToTrack(item.get());
    auto foodTransform = item->getPose();
    auto score = item->getYamlNode()["score"];
    // push_direction = item->getYamlNode()["push_direction"].as<std::string>();
    // std::cout << item->getYamlNode()["push_direction"] << std::endl;
    // choose push or scoop?:
    std::cout << "score is :" << score << std::endl;
    char mode = chooseMode("choose?");
    if (mode == 's')
    {
      //--------------------------------------------------------------------------------------------------------
      // Move to the designate place for scooping
      ScoopHelper scoopHelper = ScoopHelper(nodeHandle, foodTransform);
      double height
        = getRosParam<double>("/scoopDemo/height", nodeHandle);
      double minima
        = getRosParam<double>("/scoopDemo/minima", nodeHandle);
      double alpha
        = getRosParam<double>("/scoopDemo/alpha", nodeHandle);
      double delta
        = getRosParam<double>("/scoopDemo/delta", nodeHandle);
      double beta
        = getRosParam<double>("/scoopDemo/beta", nodeHandle);
      double scoop_mode
        = getRosParam<int>("/scoopDemo/mode", nodeHandle);
      height *= alpha;
      double theta = scoopHelper.getTheta2();
      double direction = scoopHelper.getDirection2();

      bool aboveFoodSuccess = action::moveAbove(
          ada,
          feedingDemo.getCollisionConstraint(),
          scoopHelper.getTransform(),
          feedingDemo.getFoodEndEffectorTransform(scoop_mode, height, minima, theta, direction, delta),
          0.01,
          0.01,
          0.1,
          0.00,
          feedingDemo.mPlanningTimeout,
          feedingDemo.mMaxNumTrials,
          feedingDemo.mVelocityLimits);
      if (!aboveFoodSuccess)
          ROS_WARN_STREAM("Move above plate failed. Please restart");
      else
          std::cout <<"Move above Place Success"<<std::endl;

      //--------------------------------------------------------------------------------------------------------
      // Excecute Scooping
      action::PolyScoop(
      ada,
      feedingDemo.getCollisionConstraint(),
      height,
      theta,
      minima,
      direction,
      scoop_mode,
      feedingDemo.mPlateTSRParameters["horizontalTolerance"],
      feedingDemo.mPlateTSRParameters["verticalTolerance"],
      feedingDemo.mPlateTSRParameters["rotationTolerance"],
      feedingDemo.mEndEffectorOffsetPositionTolerance,
      feedingDemo.mEndEffectorOffsetAngularTolerance,
      feedingDemo.mPlanningTimeout,
      feedingDemo.mMaxNumTrials,
      feedingDemo.mVelocityLimits);
    }
    if (mode == 'p')
    {
      std::vector<std::unique_ptr<FoodItem>> pushedItems;
      while (true)
      {
        // Perception returns the list of good candidates, any one of them is good.
        // Multiple candidates are preferrable since planning may fail.
        pushedItems = perception->perceiveReconfiguredFood("onebite_mpotato");

        if (pushedItems.size() == 0)
        {
          talk("I can't find that food. Try putting it on the plate.");
          ROS_WARN_STREAM(
              "Failed to detect any food. Please place food on the plate.");
        }
        else
          break;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      auto& pusheditem = pushedItems[0];
      perception->setFoodItemToTrack(pusheditem.get());
      auto score = pusheditem->getYamlNode()["score"];
      std::string push_direction = "left_push";
      push_direction = "left_push";
      // TODO: check whether yamlNode work or not.
      // std::cout << item->getYamlNode()["push_direction"] << std::endl;
      std::cout << "push_direction = " << push_direction << std::endl;
      
      double length = 0.05;
      length = getRosParam<double>("/pushingDemo/length", nodeHandle);
      Eigen::Isometry3d food3d= ada->getHand()->getEndEffectorTransform("food").get();
      action::PushOnFood(
        ada,
        push_direction,
        length,
        foodTransform,
        food3d,
        feedingDemo.getCollisionConstraint(),
        feedingDemo.mFoodTSRParameters["horizontalTolerance"],
        feedingDemo.mFoodTSRParameters["verticalTolerance"],
        feedingDemo.mFoodTSRParameters["rotationTolerance"],
        feedingDemo.mFoodTSRParameters["tiltTolerance"],
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits);
    }
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}

}; // namespace feeding
