#include <chrono>
#include <iostream>
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
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

std::unique_ptr<aikido::trajectory::Spline> getTrajectoryFromFile(std::string filename,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& metaSkeletonStateSpace)
{
  std::size_t dimension = metaSkeletonStateSpace->getDimension();
  io::CSVReader<13> in(filename);
  //in.read_header(io::ignore_extra_column);
  Eigen::Vector6d stateVec, velocityVec;
  double timeStep;
  std::vector<Eigen::Vector6d> stateSeq, velocitySeq;
  std::vector<double> timeSeq;
  while(in.read_row(timeStep, stateVec[0], velocityVec[0], stateVec[1], velocityVec[1],
                    stateVec[2], velocityVec[2], stateVec[3], velocityVec[3],
                    stateVec[4], velocityVec[4], stateVec[5], velocityVec[5]))
  {
    timeSeq.push_back(timeStep);
    stateSeq.push_back(stateVec);
    velocitySeq.push_back(velocityVec);
  }
  
  auto outputTrajectory = dart::common::make_unique<aikido::trajectory::Spline>(
      metaSkeletonStateSpace);
  auto segmentStartState = metaSkeletonStateSpace->createState();
  
  using CubicSplineProblem
      = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

  Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(dimension);
  for (std::size_t i = 0; i < stateSeq.size() - 1; i++)
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
  bool adaReal = false;

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

  std::size_t robotNum = 1;
  std::vector<std::shared_ptr<ada::Ada>> robots;
  std::vector<aikido::statespace::dart::MetaSkeletonStateSpacePtr> armSpaces;
  for(std::size_t i=0;i<robotNum;i++)
  {
    std::shared_ptr<ada::Ada> robot = std::make_shared<ada::Ada>(env, adaSim, adaUrdfUri, adaSrdfUri);
    robots.push_back(robot);

    auto armSkeleton = robot->getArm()->getMetaSkeleton();
    auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());

    armSpaces.push_back(armSpace);
  }

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/simple_trajectories";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
      << execTopicName
      << "' InteractiveMarker topic in RViz.");
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      env, execTopicName, baseFrameName);

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);
  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  std::string filenamePrefix = "FIRSTHALF";
  std::stringstream ss;
  for(std::size_t i=0; i<robotNum; i++)
  {
    ss.clear();
    ss << filenamePrefix << "_" << i << ".txt";
    std::string filename = ss.str();
    auto traj = getTrajectoryFromFile(filename, armSpaces[i]);
    auto future = robots[i]->executeTrajectory(std::move(traj));
  }  

  waitForUser("Wait for trajectory execution. \n Press [ENTER] after completion.");

  ros::shutdown();
  return 0;
}
