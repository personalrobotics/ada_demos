#include <chrono>
#include <iostream>
#include <Eigen/Dense>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/perception/PointCloud.hpp>
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

static const std::string topicName("/camera/depth/color/points");
static const std::string baseFrameName("map");

static const double detectionTimeout{5.};
static const double planningTimeout{5.};

bool adaReal = false;

void waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  std::cin.get();
}

Eigen::VectorXd getCurrentConfig(const ada::Ada& robot)
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
  auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);

  auto trajectory = robot.planToConfiguration(
      armSpace, armSkeleton, goalPos, nullptr, planningTimeout);

  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution");
  }

  auto smoothTrajectory
      = robot.smoothPath(armSkeleton, trajectory.get(), testable);
  aikido::trajectory::TrajectoryPtr timedTrajectory
      = std::move(robot.retimePath(armSkeleton, smoothTrajectory.get()));

  auto future = robot.executeTrajectory(timedTrajectory);
  future.wait();
}

/// Perceive objects with PoseEstimatorModule.
/// \param[in]  nh: ROS node handle to use for detection
/// \param[in]  detectorTopicName: The MarkerArray topic name
///                 of a specific detector to use
/// \param[in]  detectorDataURI: The JSON data file for the detector
/// \param[in]  resourceRetriever: Resource retriever to resolve package URIs
/// \param[in]  robot: Robot that the transforms are defined relative to
void perceivePointCloud(
    ros::NodeHandle nodeHandle, std::string detectorTopicName)
{
  using aikido::perception::PointCloud;

  PointCloud objDetector(nodeHandle, detectorTopicName);

  objDetector.updatePointCloud(ros::Duration(detectionTimeout));
}

int main(int argc, char** argv)
{
  // Default options for flags
  int target;

  po::options_description po_desc("voxel demo options");
  po_desc.add_options()("help", "Produce help message")(
      "adareal,h",
      po::bool_switch(&adaReal)->default_value(false),
      "Run ADA in real")(
      "target,t",
      po::value<int>(&target)->default_value(3),
      "A target trajectory to execute");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    std::cout << "target 0: closing hands" << std::endl
              << "target 1: opening hands" << std::endl
              << "target 2: move arms to relaxed home positions" << std::endl
              << "target 3: perceive point cloud" << std::endl;
    return 0;
  }

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "simple_trajectories");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  auto env = std::make_shared<aikido::planner::World>("voxel");

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  ada::Ada robot(env, !adaReal);
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

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);

  // Predefined positions ////////////////////////////////////////////////////

  Eigen::VectorXd armHome(6);
  armHome << 0.00, 3.14, 3.14, 0.00, 0.00, 0.00;

  Eigen::VectorXd armRelaxedHome(6);
  armRelaxedHome << 1.00, 1.00, 1.00, 1.00, 1.00, 1.00;

  auto arm = robot.getArm();
  auto armSkeleton = arm->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());
  armSkeleton->setPositions(armHome);

  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  if (target == 0) // target 0: closing hands
  {
    robot.getHand()->executePreshape("closed").wait();
    waitForUser("Press [ENTER] to exit: ");
    return 0;
  }
  else if (target == 1) // target 1: opening hands
  {
    robot.getHand()->executePreshape("open").wait();
    waitForUser("Press [ENTER] to exit: ");
  }
  else if (target == 2)
  {
    waitForUser("Press key to look at the goal pos.");
    armSkeleton->setPositions(armRelaxedHome);

    waitForUser("Press key to set arm back to start pos.");
    armSkeleton->setPositions(armHome);

    waitForUser("Press key to move arm to goal");
    moveArmTo(robot, armSpace, armSkeleton, armRelaxedHome);

    waitForUser("Press [ENTER] to exit: ");
  }
  else if (target == 3)
  {
    bool pause = false;
    while (true)
    {
      std::cout << "Enter 1 to continue, 0 to exit: ";
      std::cin >> pause;
      if (pause == 0)
        break;

      perceivePointCloud(nh, topicName);
    }
  }

  if (adaReal)
  {
    robot.switchFromTrajectoryExecutorsToGravityCompensationControllers();
  }

  std::cin.get();
  return 0;
}
