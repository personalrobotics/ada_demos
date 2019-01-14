
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"

using ada::util::getRosParam;
using ada::util::waitForUser;
using aikido::rviz::WorldInteractiveMarkerViewerPtr;

namespace feeding {


const int MAX_TRIAL_PER_ITEM = 3;
const bool TERMINATE_AT_USER_PROMPT = true;

void demo(
    FeedingDemo& feedingDemo,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("==========  DEMO ==========");

  while (true)
  {
    feedingDemo.waitForUser("next step?");

    auto foodName = getUserInput(false, nodeHandle);

    nodeHandle.setParam("/deep_pose/forceFood", false);
    nodeHandle.setParam("/deep_pose/publish_spnet", (true));
    nodeHandle.setParam("/deep_pose/invertSPNetDirection", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running bite transfer study for " << foodName);

    // ===== FORQUE PICKUP =====
    if (foodName == "pickupfork")
    {
      feedingDemo.pickUpFork();
    }
    else if (foodName == "putdownfork")
    {
      feedingDemo.putDownFork();
    }
    else
    {
      feedingDemo.moveAbovePlate();

      if (!getRosParam<bool>("/study/skipSkewering", nodeHandle))
      {
        feedingDemo.skewer(foodName, nodeHandle,
          MAX_TRIAL_PER_ITEM);
      }

      // ===== IN FRONT OF PERSON =====
      feedingDemo.waitForUser("Move forque in front of person");

      bool tilted = true;
      feedingDemo.feedFoodToPerson(nodeHandle, tilted);
    }

    feedingDemo.reset();
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
};
