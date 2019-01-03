
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"

using ada::util::getRosParam;
using ada::util::waitForUser;
using aikido::rviz::WorldInteractiveMarkerViewerPtr;

namespace feeding {


const int MAX_TRIAL_PER_ITEM = 3;
const bool TERMINATE_AT_USER_PROMPT = true;

void demo(
    FeedingDemo& feedingDemo,
    FTThresholdHelper& ftThresholdHelper,
    Perception& perception,
    ros::NodeHandle nodeHandle,
    bool autoContinueDemo,
    bool adaReal)
{
  ftThresholdHelper.init();

  aikido::rviz::WorldInteractiveMarkerViewerPtr viewer
      = feedingDemo.getViewer();

  ROS_INFO_STREAM("==========  DEMO ==========");

  while (waitForUser("next step?", TERMINATE_AT_USER_PROMPT))
  {
    auto foodName = feedingDemo.getUserInput(false, nodeHandle);

    nodeHandle.setParam("/deep_pose/forceFood", false);
    nodeHandle.setParam("/deep_pose/publish_spnet", (true));
    nodeHandle.setParam("/deep_pose/invertSPNetDirection", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << std::endl
              << "\033[1;32mRunning bite transfer study for " << foodName
              << ".\033[0m" << std::endl;

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
        feedingDemo.skewer(foodName, ftThresholdHelper, perception, nodeHandle,
          autoContinueDemo, adaReal, MAX_TRIAL_PER_ITEM);
      }

      // ===== IN FRONT OF PERSON =====
      if (!autoContinueDemo)
        waitForUser("Move forque in front of person", TERMINATE_AT_USER_PROMPT);

      bool tilted = true;
      feedingDemo.feedFoodToPerson(perception, nodeHandle,
        autoContinueDemo, tilted);
    }
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
};
