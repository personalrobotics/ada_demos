#include <chrono>
#include <iostream>
#include <Eigen/Dense>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
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

dart::common::Uri adaUrdfUri{"package://ada_description/robots_urdf/ada_with_camera_forque.urdf"};
dart::common::Uri adaSrdfUri{"package://ada_description/robots_urdf/ada_with_camera_forque.srdf"};

static const double planningTimeout{5.};
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

int main(int argc, char** argv)
{
  // program should run on the real robot, not in simulation
  bool adaReal = true;

  // program should open the hand
  bool openHand = true;

  // program should close the hand
  bool closeHand = true;

  // Default options for flags
  po::options_description po_desc("hand_control options");
  po_desc.add_options()("help,h", "Produce help message")(
      "adareal,a", po::bool_switch(&adaReal), "Run ADA in real")(
      "open,o", po::bool_switch(&openHand), "Open hand")(
      "close,c", po::bool_switch(&closeHand), "Close hand");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    return 0;
  }

  bool adaSim = !adaReal;
  ROS_INFO_STREAM("Simulation Mode: " << adaSim);

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
  aikido::rviz::InteractiveMarkerViewer viewer(
      execTopicName, baseFrameName, env);

  auto space = robot.getStateSpace();
  auto collision = robot.getSelfCollisionConstraint(space, robotSkeleton);

  dart::dynamics::MetaSkeletonPtr metaSkeleton = robot.getMetaSkeleton();
  auto metaSpace = std::make_shared<MetaSkeletonStateSpace>(metaSkeleton.get());

  auto armSkeleton = robot.getArm()->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());

  if (adaSim)
  {
    // sets the arm to a default position so that we can easily operate the hand
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

  if (!adaSim)
  {
    std::cout << "Start trajectory executor" << std::endl;
    robot.startTrajectoryExecutor();
  }

  // Move hand
  auto hand = robot.getArm()->getHand();

  if (closeHand) {
    waitForUser("Close Hand.\n Press [ENTER] to proceed:");
    auto future = hand->executePreshape("closed");
    future.wait();
  }

  if (openHand) {
    waitForUser("Open Hand.\n Press [ENTER] to proceed:");
    auto future = hand->executePreshape("open");
    future.wait();
  }

  if (!openHand && !closeHand) {
    ROS_WARN("You didn't specify what to do! Use -o or -c as parameters.");
  }

  waitForUser("Press [ENTER] to exit. ");

  if (!adaSim)
  {
    ROS_INFO("Stop trajectory executor");
    robot.stopTrajectoryExecutor();
  }
  ros::shutdown();
  return 0;
}
