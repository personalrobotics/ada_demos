
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
#include <cstdlib>
#include <ctime>

using ada::util::getRosParam;

namespace feeding {

void humanStudyDemo(
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

  while (true)
  {
    if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);

    talk("What food would you like?");
    auto foodName = getUserFoodInput(true, nodeHandle);
    if (foodName == std::string("quit")) {
        break;
    }

    nodeHandle.setParam("/deep_pose/forceFood", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running human study for " << foodName);

    talk(std::string("One ") + foodName + std::string(" coming right up!"), true);
    
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

      // Send message to web interface to indicate skewer finished
      publishActionDoneToWeb();

      // ===== IN FRONT OF PERSON =====
      ROS_INFO_STREAM("Move forque in front of person");

      // TODO: Set tilted explcitly for long food items:
      bool tilted = (foodName == "celery" || foodName == "carrot" || foodName == "bell_pepper" || foodName == "apple");

      // Check autoTiming, and if false, wait for topic
      if (!getRosParam<bool>("/humanStudy/autoTiming", feedingDemo.getNodeHandle())) {
        talk("Let me know when you are ready.", false);
        std::string done = "";
        while (done != "continue") {
            std::string actionTopic;
            nodeHandle.param<std::string>("/humanStudy/actionTopic", actionTopic, "/study_action_msgs");
            done = getInputFromTopic(actionTopic, nodeHandle, true, -1);
        }
      }

      action::feedFoodToPerson(
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

  // ===== DONE =====
  ROS_INFO("Demo finished.");
  talk("Thank you, I hope I was helpful!");
}
};
