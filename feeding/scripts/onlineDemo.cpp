
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include <cstdlib>
#include <ctime>

#include <yaml-cpp/yaml.h>

#include "conban_spanet/GetAction.h"
#include "conban_spanet/PublishLoss.h"

using ada::util::getRosParam;
using ada::util::waitForUser;

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <algorithm> 
#include <cctype>
#include <locale>

namespace feeding {

void onlineDemo(
    FeedingDemo& feedingDemo,
    std::shared_ptr<Perception>& perception,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("==========  ONLINE DEMO ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  srand(time(NULL));

  nodeHandle.setParam("/spanetIncludeFeatures", true);

  while (true)
  {
    if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);
    
    // Get food name
    std::string foodName = "";
    while(feedingDemo.mFoodSkeweringForces.find(foodName) == feedingDemo.mFoodSkeweringForces.end()) {
        std::cout << "Food name: ";
        std::cin >> foodName;
        if (foodName == std::string("quit")) {
            // Note for the code purists out there:
            // This is the EXPLICIT EXCEPTION to the goto rule
            // https://github.com/isocpp/CppCoreGuidelines/blob/036324/CppCoreGuidelines.md#es76-avoid-goto
            goto endwhile;
        }
    }

    ROS_INFO_STREAM("Running online demo for " << foodName);

    // Move above plate
    ROS_INFO_STREAM("Move above plate");
    bool abovePlateSuccess = action::moveAbovePlate(
        ada,
        collisionFree,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters.at("horizontalTolerance"),
        feedingDemo.mPlateTSRParameters.at("verticalTolerance"),
        feedingDemo.mPlateTSRParameters.at("rotationTolerance"),
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits);
    if (!abovePlateSuccess) {
      ROS_WARN_STREAM("Error moving above plate");
      break;
    }

    // Get action from food item image
    int action = -1;
    std::vector<double> p_t;
    std::vector<std::unique_ptr<FoodItem>> candidateItems;
    while(candidateItems.size() == 0) {
        ROS_INFO_STREAM("Detecting " << foodName);
        candidateItems = perception->perceiveFood(foodName);
    }
    auto item = std::move(candidateItems[0]);

    YAML::Node node = item->getExtraInfo();
    if(node["features"].IsSequence()) {
      std::vector<double> features = node["features"].as<std::vector<double>>();

      // Send features to ROS Service
      conban_spanet::GetAction srv;
      srv.request.features.insert(std::end(srv.request.features), std::begin(features), std::end(features));
      if (ros::service::call("GetAction", srv))
      {
        // Set mAnnotation and overwrite action.
        p_t = srv.response.p_t;
        action = srv.response.a_t;
      }
      else
      {
        ROS_ERROR("Failed to call service get_action, try again");
        continue;
      }
    } else {
      ROS_WARN_STREAM("Warning: no feature vector, try again!");
      continue;
    }


    // Move above food item
    ROS_INFO_STREAM("Detect and move above food, action " << action);
    item = action::detectAndMoveAboveFood(
            ada,
            collisionFree,
            perception,
            foodName,
            feedingDemo.mFoodTSRParameters.at("height"),
            feedingDemo.mFoodTSRParameters.at("horizontalTolerance"),
            feedingDemo.mFoodTSRParameters.at("verticalTolerance"),
            feedingDemo.mFoodTSRParameters.at("rotationTolerance"),
            feedingDemo.mFoodTSRParameters.at("tiltTolerance"),
            feedingDemo.mPlanningTimeout,
            feedingDemo.mMaxNumTrials,
            feedingDemo.mVelocityLimits,
            &feedingDemo,
            nullptr,
            action);

    ROS_INFO_STREAM("Adjusting...");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    item = action::detectAndMoveAboveFood(
            ada,
            collisionFree,
            perception,
            foodName,
            feedingDemo.mFoodTSRParameters.at("height"),
            feedingDemo.mFoodTSRParameters.at("horizontalTolerance"),
            feedingDemo.mFoodTSRParameters.at("verticalTolerance"),
            feedingDemo.mFoodTSRParameters.at("rotationTolerance"),
            feedingDemo.mFoodTSRParameters.at("tiltTolerance"),
            feedingDemo.mPlanningTimeout,
            feedingDemo.mMaxNumTrials,
            feedingDemo.mVelocityLimits,
            &feedingDemo,
            nullptr,
            action);


    // Re-tare force, set to move-in threshold
    ROS_INFO_STREAM("Setting force thresholds and re-taring...");
    if (feedingDemo.getFTThresholdHelper()) {
        feedingDemo.getFTThresholdHelper()->setThresholds(feedingDemo.mFoodSkeweringForces[foodName], 2.0, true);
    }

    // Add fudge factor
    std::vector<double> offsetVector
      = getRosParam<std::vector<double>>("/acquisitionData/foodOffsetFork", nodeHandle);
    Eigen::Vector3d foodOffset(offsetVector[0], offsetVector[1], offsetVector[2]);

    Eigen::Isometry3d eePose
              = ada->getHand()->getEndEffectorBodyNode()->getTransform();
    foodOffset = eePose.rotation() * foodOffset;

    if(action > 3) {
      // No fork offset for angled skewering
      foodOffset = Eigen::Vector3d(0, 0, 0);
    }

    offsetVector
      = getRosParam<std::vector<double>>("/acquisitionData/foodOffsetWorld", nodeHandle);
    Eigen::Vector3d worldOffset(offsetVector[0], offsetVector[1], offsetVector[2]);
    
    foodOffset += worldOffset;

    // Move into food item
    ROS_INFO_STREAM("Skewering...");
    auto moveIntoSuccess = action::moveInto(
        ada,
        perception,
        &nodeHandle,
        feedingDemo.getFTThresholdHelper(),
        feedingDemo.mServoVelocity,
        "", // Just get the one food present
        foodOffset);

    if (!moveIntoSuccess)
    {
      ROS_ERROR_STREAM("Failed to skewer food.");
      break;
    }

    // Move out of food item
    ROS_INFO_STREAM("Move out of food item");
    Eigen::Vector3d direction(0, 0, 1);
    action::moveOutOf(
        ada,
        nullptr,
        TargetItem::FOOD,
        feedingDemo.mMoveOufOfFoodLength * 2.0,
        direction,
        feedingDemo.mPlanningTimeout,
        feedingDemo.mEndEffectorOffsetPositionTolerance,
        feedingDemo.mEndEffectorOffsetAngularTolerance,
        feedingDemo.getFTThresholdHelper(),
        feedingDemo.mVelocityLimits);

    // Record success / failure
    int loss = -1;
    ROS_INFO_STREAM("Wait before determining success...");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    bool success;
    std::cout << "Did I succeed (0, 1)? ";
    std::cin >> success;
    if (success)
    {
      ROS_INFO_STREAM("Recording success...");
      loss = 0;
    } else {
      ROS_INFO_STREAM("Recording failure...");
      loss = 1;
    }

    // Publish Loss to algorithm
    std::vector<double> features = node["features"].as<std::vector<double>>();

    // Actually call service
    conban_spanet::PublishLoss srv;
    srv.request.features.insert(std::end(srv.request.features), std::begin(features), std::end(features));
    srv.request.p_t.insert(std::end(srv.request.p_t), std::begin(p_t), std::end(p_t));
    srv.request.a_t = action;
    srv.request.loss = loss;

    if (ros::service::call("PublishLoss", srv))
    {
      ROS_INFO("Success! Error Message: %d", (int)srv.response.success);
    }
    else
    {
      ROS_ERROR("Failed to call service publish_loss");
    }


    ROS_INFO("Done! Replace food item...\n\n");

  } // end while
endwhile:

  // ===== DONE =====
  ROS_INFO("Data collection finished.");
}
};
