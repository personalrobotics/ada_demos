
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/Reset.hpp"

using ada::util::waitForUser;

namespace feeding {

void resetDemo(
    FeedingDemo& feedingDemo,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("========== Reset DEMO ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  while (true)
  {
    waitForUser("next step?", ada);

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running Reset demo");

    action::reset(
    ada,
    collisionFree,
    plate,
    feedingDemo.getPlateEndEffectorTransform(),
    feedingDemo.mPlateTSRParameters["horizontalTolerance"],
    feedingDemo.mPlateTSRParameters["verticalTolerance"],
    feedingDemo.mPlateTSRParameters["rotationTolerance"],
    feedingDemo.mPlanningTimeout,
    feedingDemo.mEndEffectorOffsetPositionTolerance,
    feedingDemo.mEndEffectorOffsetAngularTolerance,
    feedingDemo.mMaxNumTrials,
    feedingDemo.mVelocityLimits,
    feedingDemo.getFTThresholdHelper());

    workspace.reset();
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
};
