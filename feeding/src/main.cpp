
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include "feeding/FTThresholdHelper.hpp"
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

  ftThresholdHelper.init();
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

  // ===== ABOVE FOOD =====
  if (!autoContinueDemo)
    if (!waitForUser("Perceive Food"))
    {
      return 0;
    }
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
    if (!waitForUser("Move forque above food"))
    {
      return 0;
    }
  }
  feedingDemo.moveAboveFood(foodTransform);

  // auto testTSR = pr_tsr::getDefaultPlateTSR();
  // testTSR.mT0_w = foodTransform;
  // testTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};

  // testTSR.mBw = createBwMatrixForTSR(
  //     0.01, 0.01, 0, 0);
  // testTSR.mTw_e.matrix()
  //     *= feedingDemo.getAda().getHand()->getEndEffectorTransform("plate")->matrix();
  // feedingDemo.moveArmToTSR(testTSR);

  // ===== INTO FOOD =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move forque into food"))
    {
      return 0;
    }
  }
  if (!ftThresholdHelper.setThresholds(GRAB_FOOD_FT_THRESHOLD))
  {
    return 1;
  }
  feedingDemo.moveIntoFood(&perception, viewer);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          getRosParam<int>("/feedingDemo/waitMillisecsAtFood", nodeHandle)));
  feedingDemo.grabFoodWithForque();

  // ===== OUT OF FOOD =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move forque out of food"))
    {
      return 0;
    }
  }
  if (!ftThresholdHelper.setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD))
  {
    return 1;
  }
  feedingDemo.moveOutOfFood();
  if (!ftThresholdHelper.setThresholds(STANDARD_FT_THRESHOLD))
  {
    return 1;
  }

  // ===== IN FRONT OF PERSON =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move forque in front of person"))
    {
      return 0;
    }
  }
  feedingDemo.moveInFrontOfPerson();

  // ===== TOWARDS PERSON =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move towards person"))
    {
      return 0;
    }
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
