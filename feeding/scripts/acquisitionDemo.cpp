// This demo is a modified version of demo.cpp
// Calls the Acquire action

// For now this is a copy paste from demo.cpp


#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>


#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/Acquire.hpp"
#include "feeding/action/PickUpFork.hpp"
#include "feeding/action/PutDownFork.hpp"
#include "feeding/action/FeedFoodToPerson.hpp"
#include "feeding/action/Skewer.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include <cstdlib>
#include <ctime>

using ada::util::getRosParam;
using ada::util::waitForUser;

namespace feeding {

void acquisitionDemo(
    FeedingDemo& feedingDemo,
    std::shared_ptr<Perception>& perception,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("========= ACQUISITION DEMO ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  talk("Hello, my name is aid uh. It's my pleasure to serve you today!");

  srand(time(NULL));

  while (true)
  {
    if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);

    talk("What food would you like?");
    auto foodName = getUserInput(true, nodeHandle);
    if (foodName == std::string("quit")) {
        break;
    }

    nodeHandle.setParam("/deep_pose/forceFood", false);
    nodeHandle.setParam("/deep_pose/publish_spnet", (true));
    nodeHandle.setParam("/deep_pose/invertSPNetDirection", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running bite transfer study for " << foodName);



    talk(std::string("One ") + foodName + std::string(" coming right up!"), true);

    // ===== FORQUE PICKUP =====
    if (foodName == "pickupfork")
    {
      action::pickUpFork(
        ada,
        collisionFree,
        feedingDemo.mForkHolderAngle,
        feedingDemo.mForkHolderTranslation,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters.at("height"),
        feedingDemo.mPlateTSRParameters.at("horizontalTolerance"),
        feedingDemo.mPlateTSRParameters.at("verticalTolerance"),
        feedingDemo.mPlateTSRParameters.at("rotationTolerance"),
        feedingDemo.mEndEffectorOffsetPositionTolerance,
        feedingDemo.mEndEffectorOffsetAngularTolerance,
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits,
        feedingDemo.getFTThresholdHelper());
    }
    else if (foodName == "putdownfork")
    {
      action::putDownFork(
        ada,
        collisionFree,
        feedingDemo.mForkHolderAngle,
        feedingDemo.mForkHolderTranslation,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters.at("height"),
        feedingDemo.mPlateTSRParameters.at("horizontalTolerance"),
        feedingDemo.mPlateTSRParameters.at("verticalTolerance"),
        feedingDemo.mPlateTSRParameters.at("rotationTolerance"),
        feedingDemo.mEndEffectorOffsetPositionTolerance,
        feedingDemo.mEndEffectorOffsetAngularTolerance,
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits,
        feedingDemo.getFTThresholdHelper());
    }
    else
    {
      bool acquire = action::acquire(
        ada,
        workspace,
        collisionFree,
        perception,
        &nodeHandle,
        foodName,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters.at("horizontalTolerance"),
        feedingDemo.mPlateTSRParameters.at("verticalTolerance"),
        feedingDemo.mPlateTSRParameters.at("rotationTolerance"),
        feedingDemo.mFoodTSRParameters.at("height"),
        feedingDemo.mFoodTSRParameters.at("horizontalTolerance"),
        feedingDemo.mFoodTSRParameters.at("verticalTolerance"),
        feedingDemo.mFoodTSRParameters.at("rotationTolerance"),
        feedingDemo.mFoodTSRParameters.at("tiltTolerance"),
        feedingDemo.mMoveOufOfFoodLength,
        feedingDemo.mEndEffectorOffsetPositionTolerance,
        feedingDemo.mEndEffectorOffsetAngularTolerance,
        feedingDemo.mWaitTimeForFood,
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits,
        feedingDemo.getFTThresholdHelper(),
        feedingDemo.mRotationFreeFoodNames,
        &feedingDemo,
	      M_PI/4, // 0 <= incidentAngle <= pi/4
	      5, // 3N <= force <= 25N
	      0, // inFoodRotationAngle
	      M_PI/2); // pi/4 <= exitAngle <= pi/2

      if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);

      if (!acquire)
      {
        ROS_WARN_STREAM("Restart from the beginning");
        continue;
      }
    }
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
};


