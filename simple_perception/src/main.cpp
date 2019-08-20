#include <iostream>
#include <Eigen/Dense>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/perception/AssetDatabase.hpp>
#include <aikido/perception/PoseEstimatorModule.hpp>
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

dart::common::Uri adaUrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.urdf"};
dart::common::Uri adaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.srdf"};

void waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  std::cin.get();
}

std::string getRosParamString(
    const std::string& paramName,
    const ros::NodeHandle& nh,
    const std::string& paramDefault = "")
{
  std::string value;
  if (!nh.getParam(paramName, value))
  {
    value = paramDefault;
  }
  return value;
}

int main(int argc, char** argv)
{
  bool adaSim = true;

  // Default options for flags
  po::options_description po_desc("simple_perception options");
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
  ros::init(argc, argv, "simple_perception");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(
      new aikido::planner::World("simple_perception"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  ada::Ada robot(env, adaSim, adaUrdfUri, adaSrdfUri);
  auto robotSkeleton = robot.getMetaSkeleton();

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/simple_perception";

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

  if (!adaSim)
  {
    ROS_INFO("Start trajectory executor");
    robot.startTrajectoryExecutor();
  }

  /////////////////////////////////////////////////////////////////////////////
  //   Start Perception Module
  /////////////////////////////////////////////////////////////////////////////
  std::string detectorDataURI
      = "package://pr_assets/data/objects/tag_data_foods.json";
  std::string referenceFrameName = robotSkeleton->getBodyNode(0)->getName();
  std::string foodDetectorTopicName = getRosParamString(
      "/perception/foodDetectorTopicName", nh, "/simulated_pose/marker_array");

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  std::unique_ptr<aikido::perception::PoseEstimatorModule> mDetector
      = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
          new aikido::perception::PoseEstimatorModule(
              nh,
              foodDetectorTopicName,
              std::make_shared<aikido::perception::AssetDatabase>(
                  resourceRetriever, detectorDataURI),
              resourceRetriever,
              referenceFrameName,
              aikido::robot::util::getBodyNodeOrThrow(
                  *metaSkeleton, referenceFrameName)));

  /////////////////////////////////////////////////////////////////////////////
  //   Detect Objects
  /////////////////////////////////////////////////////////////////////////////

  ROS_INFO("Running perception! Press ^C to exit...");

  while (ros::ok())
  {
    mDetector->detectObjects(env, ros::Duration(1.0));
    ros::spinOnce();
  }

  return 0;
}
