
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"

using namespace feeding;

///
/// OVERVIEW OF FEEDING DEMO CODE
///
/// First, everything is initalized.
/// The FeedingDemo object is responsible for robot and the workspace.
/// The FTThresholdController sets the thresholds in the
/// MoveUntilTouchController
/// The Perception object can perceive food.
///
/// Then the demo is run step by step.
///
int main(int argc, char** argv)
{

  // ===== STARTUP =====

  // Is the real robot used or simulation?
  bool adaReal = false;

  // Should the demo continue without asking for human input at each step?
  bool autoContinueDemo = false;

  // the FT sensing can stop trajectories if the forces are too big
  bool useFTSensing = false;

  handleArguments(argc, argv, adaReal, autoContinueDemo, useFTSensing);
  ROS_INFO_STREAM("Simulation Mode: " << !adaReal);

  // start node
  ros::init(argc, argv, "feeding");
  ros::NodeHandle nodeHandle("~");
  ros::AsyncSpinner spinner(2); // 2 threads
  spinner.start();

  // start demo
  FeedingDemo feedingDemo(adaReal, useFTSensing, nodeHandle);

  // visualization
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      feedingDemo.getWorld(),
      getRosParam<std::string>("/visualization/topicName", nodeHandle),
      getRosParam<std::string>("/visualization/baseFrameName", nodeHandle));
  viewer.setAutoUpdate(true);

  feedingDemo.closeHand();

  if (!waitForUser("Startup complete."))
  {
    return 0;
  }

  feedingDemo.moveToStartConfiguration();

  // ===== ABOVE PLATE =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move forque above plate"))
    {
      return 0;
    }
  }
  feedingDemo.moveAbovePlate();

  // ===== DONE =====
  waitForUser("Demo finished.");
  return 0;
}
