
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/Pushing.hpp"

using ada::util::waitForUser;

namespace feeding {

void pushingDemo(
    FeedingDemo& feedingDemo,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("========== Pushing DEMO ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  while (true)
  {
    waitForUser("next step?", ada);

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running Pushing demo");

    action::pushing();
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
};
