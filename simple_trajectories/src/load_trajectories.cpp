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
#include "csv.h"

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

static const double planningTimeout{15.};
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

aikido::trajectory::SplinePtr getTrajectoryFromFile(std::string filename,
    const statespace::dart::MetaSkeletonStateSpacePtr& metaSkeletonStateSpace)
{
  io::CSVReader<13> in(filename);
  in.read_header(io::ignore_extra_column);
  Eigen::Vector6d stateVec, velocityVec;
  double timeStep;
  std::vector<Eigen::Vector6d> stateSeq, velocitySeq;
  std::vector<double> timeSeq;
  while(in.read_row(timeStep, stateVec[0], stateVec[1], stateVec[2], stateVec[3], 
                    stateVec[4], stateVec[5], velocityVec[0], velocityVec[1],
                    velocityVec[2], velocityVec[3], velocityVec[4], velocityVec[5]))
  {
    timeSeq.push_back(timeStep);
    stateSeq.push_back(stateVec);
    velocitySeq.push_back(velocityVec);
  }
  
  auto outputTrajectory = dart::common::make_unique<aikido::trajectory::Spline>(
      metaSkeletonStateSpace);
  auto segmentStartState = metaSkeletonStateSpace->createState();
  
  Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(dimension);
  for (std::size_t i = 0; i < points.size() - 1; i++)
  {
    Eigen::VectorXd positionCurr = stateSeq[i];
    Eigen::VectorXd velocityCurr = velocitySeq[i];
    Eigen::VectorXd positionNext = stateSeq[i+1];
    Eigen::VectorXd velocityNext = velocitySeq[i+1];

    double timeCurr = timeSeq[i];
    double timeNext = timeSeq[i+1];

    CubicSplineProblem problem(
        Eigen::Vector2d(0.0, timeNext - timeCurr), 4, dimension);
    problem.addConstantConstraint(0, 0, zeroPosition);
    problem.addConstantConstraint(0, 1, velocityCurr);
    problem.addConstantConstraint(1, 0, positionNext - positionCurr);
    problem.addConstantConstraint(1, 1, velocityNext);
    const auto spline = problem.fit();

    metaSkeletonStateSpace->expMap(positionCurr, segmentStartState);

    // Add the ramp to the output trajectory.
    assert(spline.getCoefficients().size() == 1);
    const auto& coefficients = spline.getCoefficients().front();
    outputTrajectory->addSegment(
        coefficients, timeNext - timeCurr, segmentStartState);
  }

  return outputTrajectory;
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
    home <<  -1.7833, 3.37821, 1.67694, -1.48822, 0.751753, 1.1297;
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



  /////////////////////////////////////////////////////////////////////////////
  //   Trajectory execution
  /////////////////////////////////////////////////////////////////////////////

  auto currentPose = armSkeleton->getPositions();
  std::cout << "ARM current position:\n"
            << currentPose.transpose() << std::endl;
  //Eigen::VectorXd movedPose(currentPose);
  //movedPose(5) -= 0.5;
  Eigen::VectorXd movedPose(6);
  movedPose << -1.92989, 3.02971, 2.74529, -0.57672, 1.66878, -0.0733088;

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
  Eigen::VectorXd viaVelocity(6);
  viaConfig << -1.92989, 4.00172, 2.74529, -0.57672, 1.66878, -0.0733088;
  viaVelocity << 0.0, -0.1, -0.01, 0.0, 0.0, 0.0;

  Eigen::VectorXd goalConfig(movedPose);
  goalConfig(1) -= 0.4;
  goalConfig(2) -= 0.1;
 
  ROS_INFO("Starting the kinodynamic testing");
  moveArmTo(robot, armSpace, armSkeleton, 
            viaConfig, viaVelocity, goalConfig);  


  waitForUser("Open Hand.\n Press [ENTER] to proceed:");
  future = hand->executePreshape("open");
  future.wait();


  waitForUser("Press [ENTER] to exit. ");

  if (!adaSim)
  {
    std::cout << "Stop trajectory executor" << std::endl;
    robot.stopTrajectoryExecutor();
  }
  ros::shutdown();
  return 0;
}
