#include <chrono>
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

namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::robot::Robot;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

dart::common::Uri adaUrdfUri{"package://ada_description/robots/ada.urdf"};
dart::common::Uri adaSrdfUri{"package://ada_description/robots/ada.srdf"};

static const double planningTimeout{5.};
static const double maxDistanceBtwValidityChecks{0.01};
bool adaSim = true;

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

void moveArmTo(ada::Ada& robot,
               MetaSkeletonStateSpacePtr armSpace,
               MetaSkeletonPtr skeleton,
               const Eigen::VectorXd& viaPos,
               const Eigen::VectorXd& viaVelocity,
               const Eigen::VectorXd& goalPos)
{
  auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);

  double viaTime = 0.0;
  std::cout << "ARM SPACE DIMENSION: " << armSpace->getDimension() << std::endl;
  auto trajectory = robot.planMinimumTimeViaConstraint(
      armSpace, skeleton, goalPos, viaPos, viaVelocity, nullptr, viaTime,
      planningTimeout, maxDistanceBtwValidityChecks);

  auto viaState = armSpace->createState();
  Eigen::VectorXd viaVel(armSpace->getDimension());
  trajectory->evaluate(viaTime, viaState);
  trajectory->evaluateDerivative(viaTime, 1, viaVel);
  std::cout << "VALIDATION:";
  armSpace->print(viaState, std::cout);
  std::cout << std::endl;
  std::cout << "VELOCITY:";
  std::cout << viaVel << std::endl;

  if (!trajectory)
  {
    throw std::runtime_error("failed to find a 1.solution");
  }
  else
  {
    std::cout << "FIND A TRAJECTORY - Duration ";
    std::cout << trajectory->getDuration() << std::endl;
  }
  
  waitForUser("READY TO EXECUTE");
  robot.executeTrajectory(std::move(trajectory)).wait();
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

int main(int argc, char** argv)
{
  // Default options for flags
  int target;

  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()("help", "Produce help message")(
      "adasim,h",
      po::bool_switch(&adaSim)->default_value(true),
      "Run ADA in sim");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    return 0;
  }

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "simple_trajectories");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(
      new aikido::planner::World("simple_trajectories"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  ada::Ada robot(env, adaSim, adaUrdfUri, adaSrdfUri);
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

  std::cout << "POS UPPER LIMITS:" << armSkeleton->getPositionUpperLimits() << std::endl;
  std::cout << "POS LOWER LIMITS:" << armSkeleton->getPositionLowerLimits() << std::endl;
  std::cout << "VEL UPPER LIMITS:" << armSkeleton->getVelocityUpperLimits() << std::endl;
  std::cout << "VEL LOWER LIMITS:" << armSkeleton->getVelocityLowerLimits() << std::endl;
  std::cout << "ACC UPPER LIMITS:" << armSkeleton->getAccelerationUpperLimits() << std::endl;
  std::cout << "ACC LOWER LIMITS:" << armSkeleton->getAccelerationLowerLimits() << std::endl;

  if (adaSim)
  {
    Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
    home[1] = 3.14;
    home[2] = 3.14;
    armSkeleton->setPositions(home);

    auto startState
        = space->getScopedStateFromMetaSkeleton(robotSkeleton.get());

    if (!collision->isSatisfied(startState))
    {
      throw std::runtime_error("Robot is in collison");
    }
  }

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);
  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

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

  if (!adaSim)
  {
    std::cout << "Start trajectory executor" << std::endl;
    robot.startTrajectoryExecutor();
  }

  moveArmTo(robot, armSpace, armSkeleton, movedPose);

  waitForUser("Press key to continue.");
  Eigen::VectorXd nextMovedPose(movedPose);
  nextMovedPose(3) -= 0.1;
  nextMovedPose(4) -= 0.1;
  Eigen::VectorXd nextViaVelocity(6);
  nextViaVelocity << 0.0, 0.0, 0.0, 0.4, 0.4, 0.0;

  Eigen::VectorXd goalPose(nextMovedPose);
  nextMovedPose(3) -= 0.3;
  nextMovedPose(4) -= 0.3;
 
  ROS_INFO("Starting the kinodynamic testing");
  moveArmTo(robot, armSpace, armSkeleton, 
            nextMovedPose, nextViaVelocity, goalPose);  

  waitForUser("Press [ENTER] to exit. ");

  if (!adaSim)
  {
    std::cout << "Stop trajectory executor" << std::endl;
    robot.stopTrajectoryExecutor();
  }
  ros::shutdown();
  return 0;
}
