#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <libada/Ada.hpp>

#include "FTThresholdHelper.hpp"
#include "util.hpp"


namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::robot::Robot;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

dart::common::Uri adaUrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera_forque.urdf"};
dart::common::Uri adaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera_forque.srdf"};

static const double planningTimeout{5.};

void waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  std::cin.get();
}

Eigen::VectorXd getCurrentConfig(ada::Ada& robot)
{
  using namespace Eigen;
  IOFormat CommaInitFmt(
      StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  // TODO (Tapo): Change this back once the robot vs. arm is cleared
  auto defaultPose = robot.getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
  return defaultPose;
}

void moveArmTo(
    ada::Ada& robot,
    const MetaSkeletonStateSpacePtr& armSpace,
    const MetaSkeletonPtr& armSkeleton,
    const Eigen::VectorXd& goalPos)
{
  waitForUser("Plan to move hand. Press [Enter] to proceed.");

  std::cout << "Goal Pose: " << goalPos.transpose() << std::endl;

  auto satisfied = std::make_shared<aikido::constraint::Satisfied>(armSpace);
  auto trajectory = robot.planToConfiguration(
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
      = robot.smoothPath(armSkeleton, trajectory.get(), satisfied);
  aikido::trajectory::TrajectoryPtr timedTrajectory
      = std::move(robot.retimePath(armSkeleton, smoothTrajectory.get()));

  waitForUser("Press key to move arm to goal");
  auto future = robot.executeTrajectory(timedTrajectory);

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

int main(int argc, char** argv)
{
  bool adaSim = false;

  // Default options fomHandr flags
  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()("help", "Produce help message")(
      "adasim,a", po::bool_switch(&adaSim), "Run ADA in sim");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    return 0;
  }

  std::cout << "Simulation Mode: " << adaSim << std::endl;

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "kevins_garbage");
  ros::NodeHandle nh("~");

  nh.setParam("/feeding/facePerceptionOn", false);

  // Create AIKIDO World
  aikido::planner::WorldPtr env(
      new aikido::planner::World("kevins_garbage"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  
  // GILWOO: 
  // Additionally pass in
  //ada::Ada robot(env, adaSim, adaUrdfUri, adaSrdfUri);

  // Kevin: Uncomment below and comment above for FT
  ada::Ada robot(env, adaSim, adaUrdfUri, adaSrdfUri,
    "j2n6s200_forque_end_effector", "move_until_touch_topic_controller"); 
  

  // GILWOO: construct  FTThresholdHelper
  // Copy FTThresholdHelper.hpp & cpp into this folder


    /////////////// FT Sensor //////////////
  FTThresholdHelper ftThresholdHelper(true, nh);
  
  // GILWOO: Set the threshold:
  //bool resultFT = ftThresholdHelper.setThresholds(30, 3);

  auto robotSkeleton = robot.getMetaSkeleton();

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/simple_trajectories";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
      << execTopicName
      << "' InteractiveMarker topic in RViz.");
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      env, execTopicName, baseFrameName);

  auto space = robot.getStateSpace();
  auto collision = robot.getSelfCollisionConstraint(space, robotSkeleton);

  dart::dynamics::MetaSkeletonPtr metaSkeleton = robot.getMetaSkeleton();
  auto metaSpace = std::make_shared<MetaSkeletonStateSpace>(metaSkeleton.get());

  auto armSkeleton = robot.getArm()->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());

  if (adaSim)
  {
    Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
    home[1] = 3.14;
    home[2] = 3.14;
    armSkeleton->setPositions(home);

    auto startState
        = space->getScopedStateFromMetaSkeleton(robotSkeleton.get());

    aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
    if (!collision->isSatisfied(startState, &collisionCheckOutcome))
    {
      throw std::runtime_error(
          "Robot is in collison: " + collisionCheckOutcome.toString());
    }
  }

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);
  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
  home[0] = -2.06676;
  home[1] = 3.36636;
  home[2] = 1.65604;
  home[3] = -0.473418;
  home[4] = 1.76851;
  home[5] = -1.18618;
  bool resultFT = ftThresholdHelper.setThresholds(30, 3);
  if (!adaSim)
  {
    ROS_INFO("Start trajectory executor");
    robot.startTrajectoryExecutor();


    moveArmTo(robot, armSpace, armSkeleton, home);

    auto hand = robot.getArm()->getHand();
    auto future = hand->executePreshape("open");
    future.wait();

    waitForUser("Position Forque between gripper for grasping.\n Press [ENTER] to proceed");

    future = hand->executePreshape("light_grip"); //TODO: customize finger positions. Going to need sensors for this
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
      double length = 0.1;
      Eigen::Vector3d endEffectorPushDirection(0, 0, -1);
      Eigen::Vector3d endEffectorRecoverDirection(0, 0, 1);
      auto result = robot.moveArmToEndEffectorOffset(
        //endEffectorPushDirection,
        endEffectorRecoverDirection,
          length,
          nullptr,
          1,
          1,
          1);

      future.wait();

      result = robot.moveArmToEndEffectorOffset(
        //endEffectorRecoverDirection,
        endEffectorPushDirection,
          length,
          nullptr,
          1,
          1,
          1);
      if (prompt_y_n("Another trial?"))
        moveArmTo(robot, armSpace, armSkeleton, home);
      else
        break;
    }
  }




/*
  auto result = ada->moveArmToEndEffectorOffset(
        endEffectorDirection,
        length,
        nullptr,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance);
*/
    //ROS_INFO_STREAM(" Execution result: " << result);


  ros::shutdown;
  return 0;
  /*


  /////////////////////////////////////////////////////////////////////////////
  /////////////   Arrange Arm to Position and Record Position    //////////////
  /////////////////////////////////////////////////////////////////////////////  

  /////////////////////////////////////////////////////////////////////////////
  //   Grip Forque with hand
  /////////////////////////////////////////////////////////////////////////////  
  auto hand = robot.getArm()->getHand();
  waitForUser("Position Forque between gripper for grasping.\n press [ENTER] to proceed");

  auto future = hand->executePreshape("closed");	//TODO: customize finger positions. Going to need sensors for this
  future.wait();

  /////////////////////////////////////////////////////////////////////////////
  //   Push down until Forque slips out????
  /////////////////////////////////////////////////////////////////////////////  
  // TODO: Have a safety precaution to protect F/T on Forque. Make robot stop when pass some threshold.

  
  /////////////////////////////////////////////////////////////////////////////
  //   Move hand
  /////////////////////////////////////////////////////////////////////////////
  auto hand = robot.getArm()->getHand();
  waitForUser("Close Hand.\n Press [ENTER] to proceed:");
  auto future = hand->executePreshape("closed");
  future.wait();

  waitForUser("Open Hand.\n Press [ENTER] to proceed:");
  future = hand->executePreshape("open");
  future.wait();

  /////////////////////////////////////////////////////////////////////////////
  //   Trajectory execution
  /////////////////////////////////////////////////////////////////////////////

  auto currentPose = armSkeleton->getPositions();
  std::cout << "ARM current position:\n"
            << currentPose.transpose() << std::endl;
  Eigen::VectorXd movedPose(currentPose);
  movedPose(5) -= 0.5;

  moveArmTo(robot, armSpace, armSkeleton, movedPose);

  waitForUser("Press [ENTER] to exit. ");
  ros::shutdown();
  return 0;
  */

}