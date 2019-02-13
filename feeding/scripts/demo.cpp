
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
// #include "feeding/action/PickUpFork.hpp"
// #include "feeding/action/PutDownFork.hpp"
#include "feeding/action/FeedFoodToPerson.hpp"
#include "feeding/action/Skewer.hpp"

using ada::util::getRosParam;
using ada::util::waitForUser;
using aikido::rviz::WorldInteractiveMarkerViewerPtr;

namespace feeding {


const int MAX_TRIAL_PER_ITEM = 3;
const bool TERMINATE_AT_USER_PROMPT = true;

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

  while (true)
  {
    waitForUser("next step?", ada);

    auto foodName = getUserInput(false, nodeHandle);

    nodeHandle.setParam("/deep_pose/forceFood", false);
    nodeHandle.setParam("/deep_pose/publish_spnet", (true));
    nodeHandle.setParam("/deep_pose/invertSPNetDirection", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running bite transfer study for " << foodName);

    // ===== FORQUE PICKUP =====
    if (foodName == "pickupfork")
    {
      // action::pickUpFork(ada, collisionFree);
    }
    else if (foodName == "putdownfork")
    {
      // action::putDownFork(ada);
    }
    else
    {
      action::skewer(
        ada,
        workspace,
        collisionFree,
        perception,
        foodName,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mFoodSkeweringForces,
        feedingDemo.mPlateTSRParameters["height"],
        feedingDemo.mPlateTSRParameters["horizontalTolerance"],
        feedingDemo.mPlateTSRParameters["verticalTolerance"],
        feedingDemo.mPlateTSRParameters["rotationTolerance"],
        feedingDemo.mFoodTSRParameters["height"],
        feedingDemo.mFoodTSRParameters["horizontalTolerance"],
        feedingDemo.mFoodTSRParameters["verticalTolerance"],
        feedingDemo.mFoodTSRParameters["rotationTolerance"],
        feedingDemo.mFoodTSRParameters["tiltTolerance"],
        feedingDemo.mMoveOufOfFoodLength,
        feedingDemo.mEndEffectorOffsetPositionTolerance,
        feedingDemo.mEndEffectorOffsetAngularTolerance,
        feedingDemo.mWaitTimeForFood,
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits,
        feedingDemo.getFTThresholdHelper());

      // ===== IN FRONT OF PERSON =====
      waitForUser("Move forque in front of person", ada);

      bool tilted = (foodName != "celery");
      action::feedFoodToPerson(
        ada,
        workspace,
        collisionFree,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mWaitTimeForPerson,
        tilted,
        feedingDemo.mPlateTSRParameters["height"],
        feedingDemo.mPlateTSRParameters["horizontalTolerance"],
        feedingDemo.mPlateTSRParameters["verticalTolerance"],
        feedingDemo.mPlateTSRParameters["rotationTolerance"],
        feedingDemo.mPersonTSRParameters["distance"],
        feedingDemo.mPersonTSRParameters["horizontalTolerance"],
        feedingDemo.mPersonTSRParameters["verticalTolerance"],
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits
        );
    }

    workspace.reset();
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
};
