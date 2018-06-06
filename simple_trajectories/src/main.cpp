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

dart::common::Uri adaUrdfUri{"package://ada_description/robots_urdf/ada.urdf"};
dart::common::Uri adaSrdfUri{"package://ada_description/robots_urdf/ada.srdf"};

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

  if (!trajectory)
  {
    throw std::runtime_error("failed to find a 1.solution");
  }
  else
  {
    std::cout << "FIND A TRAJECTORY - Duration ";
    std::cout << trajectory->getDuration() << std::endl;
 
    // evaluate start pose, goal, via
    Eigen::VectorXd refStatePos = skeleton->getPositions();
 
    auto evalStartState = armSpace->createState();
    auto evalGoalState = armSpace->createState();
    auto evalViaState = armSpace->createState();
    Eigen::VectorXd evalStartPos(armSpace->getDimension());
    Eigen::VectorXd evalGoalPos(armSpace->getDimension());
    Eigen::VectorXd evalViaPos(armSpace->getDimension());
    trajectory->evaluate(trajectory->getStartTime(), evalStartState);
    trajectory->evaluate(trajectory->getEndTime(), evalGoalState);
    trajectory->evaluate(viaTime, evalViaState);
    armSpace->convertStateToPositions(evalStartState, evalStartPos);
    armSpace->convertStateToPositions(evalGoalState, evalGoalPos);
    armSpace->convertStateToPositions(evalViaState, evalViaPos);
    std::cout << "COMPARE START" << std::endl;
    std::cout << "[" << refStatePos.transpose() << "]" << std::endl;
    std::cout << "[" << evalStartPos.transpose() << "]" << std::endl;
    std::cout << "COMPARE VIA" << std::endl;
    std::cout << "[" << viaPos.transpose() << "]" << std::endl;
    std::cout << "[" << evalViaPos.transpose() << "]" << std::endl;
    std::cout << "COMPARE GOAL" << std::endl;
    std::cout << "[" << goalPos.transpose() << "]" << std::endl;
    std::cout << "[" << goalPos.transpose() << "]" << std::endl;
    
    Eigen::VectorXd evalStartVelocity(armSpace->getDimension());
    Eigen::VectorXd evalViaVelocity(armSpace->getDimension());
    Eigen::VectorXd evalGoalVelocity(armSpace->getDimension());
    trajectory->evaluateDerivative(trajectory->getStartTime(), 1, evalStartVelocity);
    trajectory->evaluateDerivative(viaTime, 1, evalViaVelocity);
    trajectory->evaluateDerivative(trajectory->getEndTime(), 1, evalGoalVelocity);
    std::cout << "START VEL" << std::endl;
    std::cout << "[" << evalStartVelocity.transpose() << "]" << std::endl;
    std::cout << "VIA VEL" << std::endl;
    std::cout << "[" << viaVelocity.transpose() << "]" << std::endl;
    std::cout << "[" << evalViaVelocity.transpose() << "]" << std::endl;
    std::cout << "GOAL VEL" << std::endl;
    std::cout << "[" << evalGoalVelocity.transpose() << "]" << std::endl; 
    
    auto positionUpperLimits = armSpace->getProperties().getPositionUpperLimits();
    auto positionLowerLimits = armSpace->getProperties().getPositionLowerLimits();
    auto velocityUpperLimits = armSpace->getProperties().getVelocityUpperLimits();
    auto velocityLowerLimits = armSpace->getProperties().getVelocityLowerLimits();
    // validate a trajectory
    for(double t=trajectory->getStartTime(); t<trajectory->getEndTime(); t+=0.005)
    {
      // get the position
      auto tmpState = armSpace->createState();
      Eigen::VectorXd tmpPos(armSpace->getDimension());
      trajectory->evaluate(t, tmpState);
      armSpace->convertStateToPositions(tmpState, tmpPos);
      
      // get the velocity
      Eigen::VectorXd tmpVel(armSpace->getDimension());
      trajectory->evaluateDerivative(t, 1, tmpVel);
  
      for(std::size_t d=0; d < armSpace->getDimension(); d++)
      {
        if(tmpPos[d]>positionUpperLimits[d] ||
           tmpPos[d]<positionLowerLimits[d])
        {
          std::cout << "@ " << t << " POS(" << tmpPos[d] << ") ";
          std::cout << "[" << positionLowerLimits[d] << " , ";
          std::cout << positionUpperLimits[d] << "]" << std::endl; 
        }
        
        if(tmpVel[d]>velocityUpperLimits[d] ||
           tmpVel[d]<velocityLowerLimits[d])
        {
          std::cout << "@ " << t << " VEL(" << tmpPos[d] << ") ";
          std::cout << "[" << velocityLowerLimits[d] << " , ";
          std::cout << velocityUpperLimits[d] << "]" << std::endl; 
        } 
      }
      /*
      std::cout << "@ " << t;
      std::cout << " POS: " << tmpPos.transpose();
      std::cout << " VEL: " << tmpVel.transpose() << std::endl;
      */
    }
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
  bool adaReal = true;

  // Default options for flags
  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()("help", "Produce help message")(
      "herbreal,h", po::bool_switch(&adaReal), "Run ADA in real");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    return 0;
  }

  bool adaSim = !adaReal;
  std::cout << "Simulation Mode: " << adaSim << std::endl;

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "simple_trajectories");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(
      new aikido::planner::World("simple_trajectories"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA. ");
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

  std::cout << "POS UPPER LIMITS:" << armSkeleton->getPositionUpperLimits().transpose() << std::endl;
  std::cout << "POS LOWER LIMITS:" << armSkeleton->getPositionLowerLimits().transpose() << std::endl;
  std::cout << "VEL UPPER LIMITS:" << armSkeleton->getVelocityUpperLimits().transpose() << std::endl;
  std::cout << "VEL LOWER LIMITS:" << armSkeleton->getVelocityLowerLimits().transpose() << std::endl;
  std::cout << "ACC UPPER LIMITS:" << armSkeleton->getAccelerationUpperLimits().transpose() << std::endl;
  std::cout << "ACC LOWER LIMITS:" << armSkeleton->getAccelerationLowerLimits().transpose() << std::endl;

  if (adaSim)
  {
    Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
    home << -1.92989, 3.03971, 2.64889, -1.34884, 1.62496, -0.602342;
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

  if (!adaSim)
  {
    std::cout << "Start trajectory executor" << std::endl;
    robot.startTrajectoryExecutor();
  }

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
  //Eigen::VectorXd movedPose(currentPose);
  //movedPose(5) -= 0.5;
  Eigen::VectorXd movedPose(6);
  movedPose << -1.92989, 3.02971, 2.63889, -1.34884, 1.62496, -1.10234;

  if (!adaSim)
  {
    std::cout << "Start trajectory executor" << std::endl;
    robot.startTrajectoryExecutor();
  }

  std::cout << "SIMPLE VALIDATION " << std::endl;
  std::cout << "[POS]: " << armSkeleton->getPositions().transpose() << std::endl;
  std::cout << "[UPPER]: " << armSkeleton->getPositionUpperLimits().transpose() << std::endl;
  std::cout << "[LOWER]: " << armSkeleton->getPositionLowerLimits().transpose() << std::endl;
  std::cout << "END VALIDATION " << std::endl;

  moveArmTo(robot, armSpace, armSkeleton, movedPose);

  waitForUser("Press key to continue.");
  Eigen::VectorXd viaConfig(movedPose);
  viaConfig(1) -= 0.01;
  viaConfig(2) -= 0.01;
  Eigen::VectorXd viaVelocity(6);
  viaVelocity << 0.0, -0.8, -0.4, 0.8, 0.0, 0.0;

  Eigen::VectorXd goalConfig(movedPose);
  goalConfig(1) -= 0.02;
  goalConfig(2) -= 0.02;
 
  ROS_INFO("Starting the kinodynamic testing");
  moveArmTo(robot, armSpace, armSkeleton, 
            viaConfig, viaVelocity, goalConfig);  


  waitForUser("Press [ENTER] to exit. ");

  if (!adaSim)
  {
    std::cout << "Stop trajectory executor" << std::endl;
    robot.stopTrajectoryExecutor();
  }
  ros::shutdown();
  return 0;
}
