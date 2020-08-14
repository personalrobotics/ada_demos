
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

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

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <boost/program_options.hpp>

using ada::util::getRosParam;
using ada::util::waitForUser;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::robot::Robot;

namespace feeding {

static const double planningTimeout{5.};

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

void moveArmTo(
    std::shared_ptr<ada::Ada> robot,
    const MetaSkeletonStateSpacePtr& armSpace,
    const MetaSkeletonPtr& armSkeleton,
    const Eigen::VectorXd& goalPos)
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


//void gelslight_demo(
void gelslightCalibration(
    FeedingDemo& feedingDemo,
    std::shared_ptr<Perception>& perception,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("==========  GELSLIGHT CALIBRATION ==========");
  
  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  srand(time(NULL));

  Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
  home[0] = -2.06676;
  home[1] = 3.36636;
  home[2] = 1.65604;
  home[3] = -0.473418;
  home[4] = 1.76851;
  home[5] = -1.18618;

  Eigen::VectorXd home2(Eigen::VectorXd::Zero(6));
  home2[0] = -1.95785;
  home2[1] = 3.08261;
  home2[2] = 1.65612;
  home2[3] = -0.718617;
  home2[4] = 2.10093;
  home2[5] = -0.962561;

  Eigen::VectorXd home3(Eigen::VectorXd::Zero(6));
  home3[0] = -1.73721;
  home3[1] = 2.79596;
  home3[2] = 1.4342;
  home3[3] = -1.01632;
  home3[4] = 2.41338;
  home3[5] = -1.10514;

  Eigen::VectorXd measurePose(Eigen::VectorXd::Zero(6));
  measurePose[0] = -1.40947;
  measurePose[1] = 3.17954;
  measurePose[2] = 1.3428;
  measurePose[3] = -2.22972;
  measurePose[4] = 2.08668;
  measurePose[5] = 1.13232;

  ROS_INFO("Start trajectory executor");
  auto robot = feedingDemo.getAda();
  auto armSkeleton = robot->getArm()->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());
  getCurrentConfig(robot);
  moveArmTo(robot, armSpace, armSkeleton, measurePose);

  while (true)
  {
    if (feedingDemo.getFTThresholdHelper())
    {
        ROS_INFO("HELLO FROM GETFTTHRESHOLDHELPER!");
        //feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);
        feedingDemo.getFTThresholdHelper()->setThresholds(3,2);
    }
    
    auto hand = robot->getHand();
    auto future = hand->executePreshape("hard_grip");
    
    if (!prompt_y_n("Is forque in grasp? "))
    {
        hand = robot->getHand();
        future = hand->executePreshape("open");
        future.wait();

        waitForUser("Position Forque between gripper for grasping.\n Press [ENTER] to proceed");

        future = hand->executePreshape("light_grip"); 
        future.wait();
    }
    ////////////// Position arm to above plate ///////////
    moveArmTo(robot, armSpace, armSkeleton, home3);


    ///////////////Move Hand Down//////////////
      //double length = 0.01;
      double length = 0.1;
      Eigen::Vector3d endEffectorPushDirection(0, 0, -0.5);
      Eigen::Vector3d endEffectorRecoverDirection(0, 0, 1);

      auto result = robot->moveArmToEndEffectorOffset(
        endEffectorPushDirection,
          length,
          nullptr,
          1,
          1,
          1);

      //future.wait();
    if (feedingDemo.getFTThresholdHelper())
    {
        //feedingDemo.getFTThresholdHelper()->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD);
      feedingDemo.getFTThresholdHelper()->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD);
    }
    
      result = robot->moveArmToEndEffectorOffset(
        endEffectorRecoverDirection,
          length,
          nullptr,
          1,
          1,
          1);

    moveArmTo(robot, armSpace, armSkeleton, measurePose);

    if (prompt_y_n("Another trial?"))
      {
        //moveArmTo(robot, armSpace, armSkeleton, home2);
        continue;
      }
    else
        break;
  }
  // ===== DONE =====
  ROS_INFO("Demo finished.");
  ros::shutdown;
  }



void gelslightBiteAcquisition(
    FeedingDemo& feedingDemo,
    std::shared_ptr<Perception>& perception,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("==========  GELSLIGHT BITE ACQUISITION ==========");
  
  // Gelslight sensor subscriber
  //ros::Subscriber gs_sub = nodeHandle.subscribe("gelsight_ft", 10);

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  srand(time(NULL));

  Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
  home[0] = -2.06676;
  home[1] = 3.36636;
  home[2] = 1.65604;
  home[3] = -0.473418;
  home[4] = 1.76851;
  home[5] = -1.18618;
  
  Eigen::VectorXd measurePose(Eigen::VectorXd::Zero(6));
  measurePose[0] = -1.40947;
  measurePose[1] = 3.17954;
  measurePose[2] = 1.3428;
  measurePose[3] = -2.22972;
  measurePose[4] = 2.08668;
  measurePose[5] = 1.13232;

  ROS_INFO("Start trajectory executor");
  auto robot = feedingDemo.getAda();
  auto armSkeleton = robot->getArm()->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());

  moveArmTo(robot, armSpace, armSkeleton, home);

  while (true)
  {
    if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);
    
    auto hand = robot->getHand();
    auto future = hand->executePreshape("light_grip"); 

    if (!prompt_y_n("Is forque in grasp? "))
    {
        hand = robot->getHand();
        future = hand->executePreshape("open");
        future.wait();

        waitForUser("Position Forque between gripper for grasping.\n Press [ENTER] to proceed");

        future = hand->executePreshape("light_grip"); 
        future.wait();
    }

    /////////////////////////////////////////////////////
    ////////////////// PICK UP FOOD /////////////////////
    /////////////////////////////////////////////////////
    
    if (prompt_y_n("Hard grip? "))
      {
        future = hand->executePreshape("hard_grip");
        future.wait();
      }
    ///////////////Move Hand Down//////////////
      //double length = 0.01;
      double length = 0.1;
      Eigen::Vector3d endEffectorPushDirection(0, 0, -1);
      Eigen::Vector3d endEffectorRecoverDirection(0, 0, 1);

      auto result = robot->moveArmToEndEffectorOffset(
        endEffectorPushDirection,
          length,
          nullptr,
          1,
          1,
          1);

      //future.wait();
    if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD);
    
      result = robot->moveArmToEndEffectorOffset(
        endEffectorRecoverDirection,
          length,
          nullptr,
          1,
          1,
          1);

    /////////////////////////////////////////////////////
    ///////////// DETERMINE BITE ACQUISITION ////////////
    /////////////////////////////////////////////////////

    moveArmTo(robot, armSpace, armSkeleton, measurePose);



  }
  // ===== DONE =====
  ROS_INFO("Demo finished.");
  ros::shutdown;
  }
};