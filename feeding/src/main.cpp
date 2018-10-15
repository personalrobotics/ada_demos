
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <mcheck.h>

#include <aikido/statespace/Rn.hpp>

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

  // feedingDemo.moveAbovePlate();
  // ===== ABOVE PLATE =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move forque above forque"))
    {
      return 0;
    }
  }
  // feedingDemo.moveAbovePlate();

// while (true) {
  feedingDemo.moveHighAboveForque();
  // feedingDemo.moveNotThatHighAboveForque();

  // feedingDemo.mAda->getHand()->executePreshape("almost_closed").wait();
  feedingDemo.openHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  feedingDemo.moveAboveForque();


  // waitForUser("In?");
  feedingDemo.moveIntoForque();

  // waitForUser("Close?");
  feedingDemo.closeHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // waitForUser("Out?");
  feedingDemo.moveOutOfForque();

  // waitForUser("Above Plate");
  feedingDemo.moveAbovePlate();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));


  if (!waitForUser("Move it back.")) {
    return 0;
  }
// }

  
  feedingDemo.moveHighAboveForque();
  // waitForUser("Above Forque");
  feedingDemo.moveAboveForque();

  // waitForUser("In?");
  feedingDemo.moveIntoForque();

  // waitForUser("Open?");
  feedingDemo.openHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // waitForUser("Out?");
  feedingDemo.moveOutOfForque();

  waitForUser("Test finished.");
  return 0;
  // waitForUser("Above Forque");
  feedingDemo.moveAboveForque();

  // waitForUser("In?");
  feedingDemo.moveIntoForque();

  // waitForUser("Open?");
  feedingDemo.openHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // waitForUser("Out?");
  feedingDemo.moveOutOfForque();

  waitForUser("Test finished.");
  return 0;

  // ===== IN FRONT OF PERSON =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move forque in front of person"))
    {
      return 0;
    }
  }
  feedingDemo.moveInFrontOfPerson();
  nodeHandle.setParam("/feeding/facePerceptionOn", true);

  // while (true) {
  //     Eigen::Isometry3d faceTransform;
  //   bool perceptionSuccessful = perception.perceiveFace(faceTransform);
  // }
  // ===== TOWARDS PERSON =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move towards person"))
    {
      return 0;
    }
  }
  feedingDemo.moveTowardsPerson(&perception, viewer);
  nodeHandle.setParam("/feeding/facePerceptionOn", false);
  ROS_WARN("Human is eating");
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          getRosParam<int>("/feedingDemo/waitMillisecsAtPerson", nodeHandle)));
  feedingDemo.ungrabAndDeleteFood();

  // ===== AWAY FROM PERSON =====
  feedingDemo.moveAwayFromPerson();

  // ===== BACK TO PLATE =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move back to plate"))
    {
      return 0;
    }
  }
  feedingDemo.moveAbovePlate();

  // ===== DONE =====
  waitForUser("Demo finished.");
  return 0;
}
