
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/PickUpFork.hpp"
#include "feeding/action/PutDownFork.hpp"
#include "feeding/action/FeedFoodToPerson.hpp"
#include "feeding/action/Skewer.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include "std_msgs/String.h"
#include <cstdlib>
#include <ctime>

using ada::util::getRosParam;
using ada::util::waitForUser;

namespace feeding {

void demo(
    FeedingDemo& feedingDemo,
    std::shared_ptr<Perception>& perception,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("==========  DEMO ==========");

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

    // ===== Beginning of Web/Speech Interface =====
    // TODO: change it to passed in param
    int interface = 0;    // 0 - web
                          // 1 - alexa

    // get trial Type (input by test staff)
    ROS_INFO_STREAM("Select trial Type on Web Page");
    int trialType = getTrialTypeFromWebPage();
    ROS_INFO_STREAM("Selected trial Type: " << trialType);

    // get food name
    std::string foodName;
    if (interface) {
      talk("What do you want to eat?");
      ROS_INFO_STREAM("Get food item from Alexa");
      foodName = getFoodInputFromAlexa(nodeHandle);
    } else {
      ROS_INFO_STREAM("Select Food Item from Web Page");
      foodName = getFoodInputFromWebPage(nodeHandle);
    }
    ROS_INFO_STREAM("Selected Food: " << foodName);

    // TODO: needs Ethan to advise Action::Goal for web interface
    std::string action;
    if (trialType != AUTO && trialType != AC_AUTO) {
      if (interface) {
        ROS_INFO_STREAM("Get action from Alexa");
        talk("Please choose an action to pick up the food.");
        action = getActionInputFromAlexa(nodeHandle);
      } else {
        ROS_INFO_STREAM("Get action from Web Page");
        action = getActionInputFromWebPage(nodeHandle);
      }
      ROS_INFO_STREAM("Selected action: " << action);
    }

    nodeHandle.setParam("/deep_pose/forceFood", false);
    nodeHandle.setParam("/deep_pose/publish_spnet", (true));
    nodeHandle.setParam("/deep_pose/invertSPNetDirection", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running bite transfer study for " << foodName);

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
      bool skewer = action::skewer(
        ada,
        workspace,
        collisionFree,
        perception,
        &nodeHandle,
        foodName,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mFoodSkeweringForces,
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
        &feedingDemo);

      if (!skewer)
      {
        ROS_WARN_STREAM("Restart from the beginning");
        continue;
      }

      // Send message to web interface to indicate skweweing finished
      publishActionDoneToWeb();

      // ===== IN FRONT OF PERSON =====
      ROS_INFO_STREAM("Move forque in front of person");

      bool tilted = (foodName != "celery");

      // wait for user to let robot bring food
      // blocking function
      if (trialType != AUTO && trialType != TI_AUTO) {
        if (interface) {
          getTimingFromAlexa();
        } else {
          getTimingFromWebPage();
        }
      }
      // change:
      // if not autoTiming parameter

      action::feedFoodToPerson(
        trialType,
        interface,
        ada,
        workspace,
        collisionFree,
        feedingDemo.getCollisionConstraintWithWallFurtherBack(),
        perception,
        &nodeHandle,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        workspace->getPersonPose(),
        feedingDemo.mWaitTimeForPerson,
        feedingDemo.mPlateTSRParameters.at("height"),
        feedingDemo.mPlateTSRParameters.at("horizontalTolerance"),
        feedingDemo.mPlateTSRParameters.at("verticalTolerance"),
        feedingDemo.mPlateTSRParameters.at("rotationTolerance"),
        feedingDemo.mPersonTSRParameters.at("distance"),
        feedingDemo.mPersonTSRParameters.at("horizontalTolerance"),
        feedingDemo.mPersonTSRParameters.at("verticalTolerance"),
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mEndEffectorOffsetPositionTolerance,
        feedingDemo.mEndEffectorOffsetAngularTolerance,
        feedingDemo.mVelocityLimits,
        tilted ? &feedingDemo.mTiltOffset : nullptr
        );
    }
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
  talk("Thank you, I hope I was helpful!");
}
};
