
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <mcheck.h>
#include <aikido/statespace/Rn.hpp>


namespace feeding {

/*int defaultmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);

int pushingmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);

int skewerpushmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);*/

int datacollectionmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);
int datacollectionmainNoPush(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);
/*
int scoopmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);
*/
/*int studymain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);

int acquisitionmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);

int acquisitiontiltedmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);
*/
/*int bltmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);*/
/*
int bltskewerpushmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);

int bltpushingmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal);*/
};


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

  // if (mcheck(NULL) != 0) {
  //   std::cout << "MCHECK FAILED!" << std::endl;
  //   return 1;
  // }


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
  nodeHandle.setParam("/feeding/facePerceptionOn", false);
  ros::AsyncSpinner spinner(2); // 2 threads
  spinner.start();

  // start demo
  FeedingDemo feedingDemo(adaReal, useFTSensing, nodeHandle);

  FTThresholdHelper ftThresholdHelper(adaReal && useFTSensing, nodeHandle);

  Perception perception(
      feedingDemo.getWorld(),
      feedingDemo.getAda().getMetaSkeleton(),
      nodeHandle);

  // visualization
  aikido::rviz::WorldInteractiveMarkerViewerPtr viewer
      = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(
          feedingDemo.getWorld(),
          getRosParam<std::string>("/visualization/topicName", nodeHandle),
          getRosParam<std::string>("/visualization/baseFrameName", nodeHandle));
  viewer->setAutoUpdate(true);

  std::string collisionCheckResult;
  if (!feedingDemo.isCollisionFree(collisionCheckResult))
  {
    // throw std::runtime_error(collisionCheckResult);
  }

  ftThresholdHelper.init();
  feedingDemo.closeHand();


  if (!waitForUser("Startup complete."))
  {
    return 0;
  }

  feedingDemo.moveToStartConfiguration();

  return feeding::datacollectionmain(feedingDemo,
                     ftThresholdHelper,
                     perception,
                     viewer,
                     nodeHandle,
                     autoContinueDemo,
                     adaReal);
}
