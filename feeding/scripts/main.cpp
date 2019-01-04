#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/Rn.hpp>
#include <mcheck.h>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"

#include "experiments.hpp"

using ada::util::getRosParam;
using ada::util::waitForUser;


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
  using namespace feeding;
  // ===== STARTUP =====

  // Is the real robot used or simulation?
  bool adaReal = false;

  // Should the demo continue without asking for human input at each step?
  bool autoContinueDemo = false;

  // the FT sensing can stop trajectories if the forces are too big
  bool useFTSensing = false;

  bool TERMINATE_AT_USER_PROMPT = true;

  handleArguments(argc, argv, adaReal, autoContinueDemo, useFTSensing);

  ROS_INFO_STREAM("Simulation Mode: " << !adaReal);

#ifndef REWD_CONTROLLERS_FOUND
  ROS_WARN_STREAM(
      "Package rewd_controllers not found. The F/T sensor connection is not "
      "going to work.");
#endif

  // start node
  ros::init(argc, argv, "feeding");
  ros::NodeHandle nodeHandle("~");
  nodeHandle.setParam("/feeding/facePerceptionOn", false);
  ros::AsyncSpinner spinner(2); // 2 threads
  spinner.start();

  // start demo
  FeedingDemo feedingDemo(adaReal, useFTSensing, nodeHandle);

  FTThresholdHelper ftThresholdHelper(adaReal && useFTSensing, nodeHandle);
  ftThresholdHelper.init();

  Perception perception(
      feedingDemo.getWorld(),
      feedingDemo.getAda().getMetaSkeleton(),
      nodeHandle);

  ftThresholdHelper.init();

  waitForUser("Startup complete.", TERMINATE_AT_USER_PROMPT);

  feedingDemo.moveToStartConfiguration();

  demo(
      feedingDemo,
      ftThresholdHelper,
      perception,
      nodeHandle,
      autoContinueDemo,
      adaReal);

  return 0;
}

