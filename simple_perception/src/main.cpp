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
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
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
  std::cin.sync_with_stdio(false);
  ROS_INFO(msg.c_str());
  while (!std::cin.rdbuf()->in_avail())
  {
    ros::spinOnce();
  }
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

#define BASE_NAME "j2n6s200_link_base"
#define JOULE_NAME "j2n6s200_joule"
void publishTransform(const dart::dynamics::MetaSkeletonPtr metaSkeleton)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  // Get Transform from MetaSkeleton
  auto bodyNode = metaSkeleton->getBodyNode(BASE_NAME);
  auto jouleNode = metaSkeleton->getBodyNode(JOULE_NAME);
  Eigen::Isometry3d e = jouleNode->getTransform(bodyNode);
  tf::transformEigenToTF(e, transform);
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), BASE_NAME, JOULE_NAME));
}

void detectObjects(
    aikido::planner::WorldPtr env,
    std::shared_ptr<aikido::perception::PoseEstimatorModule> mDetector)
{
  mDetector->detectObjects(env, ros::Duration(1.0));
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

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/simple_perception";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
      << execTopicName
      << "' InteractiveMarker topic in RViz.");
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      env, execTopicName, baseFrameName);

  dart::dynamics::MetaSkeletonPtr metaSkeleton = robot.getMetaSkeleton();

  // Create TF publisher
  ros::Timer tfTimer = nh.createTimer(
      ros::Duration(0.1), boost::bind(publishTransform, metaSkeleton));

  auto armSkeleton = robot.getArm()->getMetaSkeleton();

  if (adaSim)
  {
    Eigen::VectorXd home(Eigen::VectorXd::Zero(6));
    home[1] = 3.14;
    home[2] = 3.14;
    armSkeleton->setPositions(home);
  }

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);
  ROS_INFO("You can view ADA in RViz now.");
  if (adaSim)
  {
    ROS_INFO(
        "If running from the launch file, here are the simulated object "
        "positions (map frame) from deep_pose_estimators:");
    ros::Duration(0.5).sleep();
  }
  waitForUser("Press [ENTER] to proceed:");

  if (!adaSim)
  {
    waitForUser(
        "Move robot so camera can see objects. \n Press [ENTER] to proceed:");
  }

  /////////////////////////////////////////////////////////////////////////////
  //   Start Perception Module
  /////////////////////////////////////////////////////////////////////////////
  std::string detectorDataURI
      = "package://pr_assets/data/objects/tag_data_foods.json";
  std::string referenceFrameName = BASE_NAME;
  std::string foodDetectorTopicName = getRosParamString(
      "/perception/detectorTopicName", nh, "/simulated_pose/marker_array");

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  std::shared_ptr<aikido::perception::PoseEstimatorModule> mDetector
      = std::shared_ptr<aikido::perception::PoseEstimatorModule>(
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

  ROS_INFO("Running perception! You should now see published markers in RViz.");
  ROS_INFO("Press ^C to exit...");

  // Create detect objects thread
  ros::Timer perceptionTimer = nh.createTimer(
      ros::Duration(1.0), boost::bind(detectObjects, env, mDetector));
  ros::spin();
  return 0;
}