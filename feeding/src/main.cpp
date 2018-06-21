
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include "feeding/FTThresholdController.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
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

  // start demo
  FeedingDemo feedingDemo(adaReal, useFTSensing, nodeHandle);

  FTThresholdController ftThresholdController(adaReal && useFTSensing, nodeHandle);

  Perception perception(
      feedingDemo.getWorld(), *feedingDemo.getAda(), nodeHandle);

  // visualization
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      feedingDemo.getWorld(),
      getRosParam<std::string>("/visualization/topicName", nodeHandle),
      getRosParam<std::string>("/visualization/baseFrameName", nodeHandle));
  viewer.setAutoUpdate(true);

  std::string collisionCheckResult;
  if (!feedingDemo.isCollisionFree(collisionCheckResult))
  {
    throw std::runtime_error(collisionCheckResult);
  }

  ftThresholdController.init();
  feedingDemo.closeHand();

  waitForUser("Startup complete.");

  feedingDemo.moveToStartConfiguration();

  // ===== ABOVE PLATE =====
  if (!autoContinueDemo)
  {
    waitForUser("Move forque above plate");
  }
  feedingDemo.moveAbovePlate();

  // ===== ABOVE FOOD =====
  if (!autoContinueDemo)
    waitForUser("Perceive Food");
  Eigen::Isometry3d foodTransform;
  if (adaReal)
  {
    bool perceptionSuccessful = perception.perceiveFood(foodTransform);
    if (!perceptionSuccessful)
      throw std::runtime_error("Perception failed");
  }
  else
  {
    foodTransform = feedingDemo.getDefaultFoodTransform();
  }
  if (!autoContinueDemo)
  {
    waitForUser("Move forque above food");
  }
  feedingDemo.moveAboveFood(foodTransform);

  // ===== INTO FOOD =====
  if (!autoContinueDemo)
  {
    waitForUser("Move forque into food");
  }
  ftThresholdController.setThreshold(GRAB_FOOD_FT_THRESHOLD);
  feedingDemo.moveIntoFood();
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          getRosParam<int>("/feedingDemo/waitMillisecsAtFood", nodeHandle)));
  feedingDemo.grabFoodWithForque();

  // ===== OUT OF FOOD =====
  if (!autoContinueDemo)
  {
    waitForUser("Move forque out of food");
  }
  ftThresholdController.setThreshold(AFTER_GRAB_FOOD_FT_THRESHOLD);
  feedingDemo.moveOutOfFood();
  ftThresholdController.setThreshold(STANDARD_FT_THRESHOLD);

  // ===== IN FRONT OF PERSON =====
  if (!autoContinueDemo)
  {
    waitForUser("Move forque in front of person");
  }
  feedingDemo.moveInFrontOfPerson();

  // ===== TOWARDS PERSON =====
  if (!autoContinueDemo)
  {
    waitForUser("Move towards person");
  }
  feedingDemo.moveTowardsPerson();
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          getRosParam<int>("/feedingDemo/waitMillisecsAtPerson", nodeHandle)));
  feedingDemo.ungrabAndDeleteFood();

  // ===== AWAY FROM PERSON =====
  feedingDemo.moveAwayFromPerson();

  // ===== BACK TO PLATE =====
  if (!autoContinueDemo)
  {
    waitForUser("Move back to plate");
  }
  feedingDemo.moveAbovePlate();

  // ===== DONE =====
  waitForUser("Demo finished.");
  ros::shutdown();
  return 0;
}
