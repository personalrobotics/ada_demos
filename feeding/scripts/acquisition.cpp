
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"

#include "feeding/perception/Perception.hpp"

using ada::util::getRosParam;
using ada::util::waitForUser;

bool TERMINATE_AT_USER_PROMPT = true;

namespace feeding {

void acquisition(
    FeedingDemo& feedingDemo,
    FTThresholdHelper& ftThresholdHelper,
    Perception& perception,
    ros::NodeHandle nodeHandle,
    bool autoContinueDemo,
    bool adaReal)
{

  aikido::rviz::WorldInteractiveMarkerViewerPtr viewer
      = feedingDemo.getViewer();
  nodeHandle.setParam("/deep_pose/publish_spnet", true);

  if (!autoContinueDemo)
    waitForUser("Ready to start.", TERMINATE_AT_USER_PROMPT);

  for (int trial = 0; trial < 10; trial++)
  {
    ROS_INFO_STREAM("STARTING TRIAL " << trial << std::endl);

    feedingDemo.skewer("", nodeHandle, 1);
  }
  // ===== DONE =====
  waitForUser("Demo finished.", TERMINATE_AT_USER_PROMPT);
}

};
