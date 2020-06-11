
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/PickUpFork.hpp"
#include "feeding/action/PutDownFork.hpp"
#include "feeding/action/FeedFoodToPerson.hpp"
#include "feeding/action/SkewerOnline.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include <cstdlib>
#include <ctime>

using ada::util::getRosParam;
using ada::util::waitForUser;

namespace feeding {

void onlineDemo(
    FeedingDemo& feedingDemo,
    std::shared_ptr<Perception>& perception,
    ros::NodeHandle& nodeHandle)
{

  ROS_INFO_STREAM("==========  DEMO ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  initTopics(&nodeHandle);
  //talk("Hello, my name is aid uh. It's my pleasure to serve you today!");

  srand(time(NULL));

  while (true)
  {
    if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);

    talk("What food would you like?", true);
    auto foodName = getUserFoodInput(true, nodeHandle, false, 0.1);
    if (foodName == std::string("quit")) {
        break;
    }

    nodeHandle.setParam("/deep_pose/forceFood", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running bite transfer study for " << foodName);

    talk(std::string("One ") + foodName + std::string(" coming right up!"), true);

      bool skewer = action::skewerOnline(
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

      if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);

      if (!skewer)
      {
        ROS_WARN_STREAM("Action failed! Algorithm should not be updated.");
        continue;
      }
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
  talk("Thank you, I hope I was helpful!");
}
};
