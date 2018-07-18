
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"

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

  /*
  // test function
  auto tmpStateSpace = std::make_shared<aikido::statespace::R2>();
  auto tmpInterpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(tmpStateSpace);
  auto tmpTraj = std::make_shared<aikido::trajectory::Interpolated>(tmpStateSpace, tmpInterpolator);
  Eigen::Vector2d tmpMaxVel(1., 1.);
  Eigen::Vector2d tmpMaxAccel(2., 2.);

  auto tmpState = tmpStateSpace->createState();
  tmpState.setValue(Eigen::Vector2d(1., 2.));
  tmpTraj->addWaypoint(0., tmpState);
  tmpState.setValue(Eigen::Vector2d(3., 4.));
  tmpTraj->addWaypoint(1., tmpState);

  Eigen::Vector2d tmpStartVel(1., 1.);
  Eigen::Vector2d tmpEndVel(0., 0.);

  auto tmpTimed1 = aikido::planner::parabolic::computeParabolicTiming(*tmpTraj, tmpMaxVel, tmpMaxAccel);
  auto tmpTimed2 = createTimedSplineTrajectory(*tmpTraj, tmpStartVel, tmpEndVel,
    tmpMaxVel, tmpMaxAccel);

  dumpSplinePhasePlot(*tmpTimed1, "tmpTimed1.txt", 0.01);
  dumpSplinePhasePlot(*tmpTimed2, "tmpTimed2.txt", 0.01);

  */

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
  while (waitForUser("Perceive Face")) {
    Eigen::Isometry3d faceTransform;
    if (adaReal)
    {
      bool perceptionSuccessful = perception.perceiveFace(faceTransform);
      if (!perceptionSuccessful)
        ROS_INFO("Perception failed");
        //throw std::runtime_error("Perception failed");
    }
    else
    {
      throw std::runtime_error("Simulation is not allowed!");
    }
  }

  // ===== DONE =====
  waitForUser("Demo finished.");
  return 0;
}
