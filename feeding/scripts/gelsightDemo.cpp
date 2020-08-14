
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>
#include <aikido/constraint/Satisfied.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/PickUpFork.hpp"
#include "feeding/action/PutDownFork.hpp"
#include "feeding/action/FeedFoodToPerson.hpp"
#include "feeding/action/Skewer.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include <cstdlib>
#include <ctime>

using ada::util::getRosParam;
using ada::util::waitForUser;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;

namespace feeding {

void waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  std::cin.get();
}

Eigen::VectorXd getCurrentConfig(std::shared_ptr<ada::Ada> robot)
{
  using namespace Eigen;
  IOFormat CommaInitFmt(
      StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  // TODO (Tapo): Change this back once the robot vs. arm is cleared
  auto defaultPose = robot->getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
  return defaultPose;
}


bool prompt_y_n(std::string msg)
{
    char choice;
    std::cout << msg << ". Press [y/n]";
    std::cin >> choice; 

    if (choice == 'y')
        return true;
    else if (choice == 'n')
        return false;
    else
        throw std::runtime_error( "Must press y or n");
}

void moveArmTo(
    std::shared_ptr<ada::Ada> robot,
    const MetaSkeletonStateSpacePtr& armSpace,
    const MetaSkeletonPtr& armSkeleton,
    const Eigen::VectorXd& goalPos,
    double planningTimeout)
{
  waitForUser("Plan to move hand. Press [Enter] to proceed.");

  std::cout << "Goal Pose: " << goalPos.transpose() << std::endl;

  auto satisfied = std::make_shared<aikido::constraint::Satisfied>(armSpace);
  auto trajectory = robot->planToConfiguration(
      armSpace, armSkeleton, goalPos, nullptr, planningTimeout);

  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution");
  }

  ROS_INFO_STREAM("Evaluate the found trajectory at half way");
  auto state = armSpace->createState();
  trajectory->evaluate(0.5, state);
  Eigen::VectorXd positions;
  armSpace->convertStateToPositions(state, positions);
  ROS_INFO_STREAM(positions.transpose());

  auto smoothTrajectory
      = robot->smoothPath(armSkeleton, trajectory.get(), satisfied);
  aikido::trajectory::TrajectoryPtr timedTrajectory
      = std::move(robot->retimePath(armSkeleton, smoothTrajectory.get()));

  waitForUser("Press key to move arm to goal");
  auto future = robot->executeTrajectory(timedTrajectory);

  future.wait();
  getCurrentConfig(robot);
}

void gelsightDemo(
    FeedingDemo& feedingDemo,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("==========  GELSIGHT DEMO ==========");

  auto robot = feedingDemo.getAda();
  auto ftThresholdHelper = feedingDemo.getFTThresholdHelper();

  // Define MetaSkeleton Spaces
  auto armSkeleton = robot->getArm()->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());

  srand(time(NULL));

  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
  home[0] = -2.1006;
  home[1] = 3.08929;
  home[2] = 1.41905;
  home[3] = -0.473552;
  home[4] = 1.76847;
  home[5] = -1.18618; 

  if (prompt_y_n("Set custom force threshold?"))
  {
    double fThreshold;
    std::cout << "Enter force threshold (N): " << std::endl;
    std::cin >> fThreshold;
    bool resultFT = ftThresholdHelper->setThresholds(fThreshold, 3);
  }
  else
  {
    bool resultFT = ftThresholdHelper->setThresholds(5, 3);
  }

  moveArmTo(robot, armSpace, armSkeleton, home, feedingDemo.mPlanningTimeout);

  auto hand = robot->getArm()->getHand(); 
  if (!prompt_y_n("Is forque currently grasped?"))
  {  
    auto future = hand->executePreshape("open");
    future.wait();

    waitForUser("Position Forque between gripper for grasping.\n Press [ENTER] to proceed");

  }

  auto future = hand->executePreshape("light_grip"); //TODO: customize finger positions. Going to need sensors for this
  future.wait();
/////////////// Gripper //////////////
  while(true) {
    if (prompt_y_n("Hard grip? "))
    {
      future = hand->executePreshape("hard_grip");
      future.wait();
    }

  ///////////////Move Hand Down//////////////
    //double length = 0.01;
    const std::vector<double> velocityLimits{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

    double length = 0.1;
    Eigen::Vector3d endEffectorPushDirection(0, 0, -1);
    Eigen::Vector3d endEffectorRecoverDirection(0, 0, 1);
    auto result = robot->moveArmToEndEffectorOffset(
      endEffectorPushDirection,
      //endEffectorRecoverDirection,
        length,
        nullptr,
        1,
        1,
        1,
        velocityLimits);

    future.wait();

    result = robot->moveArmToEndEffectorOffset(
      endEffectorRecoverDirection,
      // endEffectorPushDirection,
        length,
        nullptr,
        1,
        1,
        1);
    if (prompt_y_n("Another trial?"))
      moveArmTo(robot, armSpace, armSkeleton, home, feedingDemo.mPlanningTimeout);
    else
      break;
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
};
