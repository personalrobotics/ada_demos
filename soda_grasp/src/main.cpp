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
#include <pr_tsr/can.hpp>

#include <libada/Ada.hpp>

namespace po = boost::program_options;

using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

using aikido::constraint::dart::TSR;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

using aikido::planner::parabolic::ParabolicSmoother;
using aikido::planner::kunzretimer::KunzRetimer;

static const double planningTimeout{5.};
bool adaReal = false;
bool feeding = false;

void waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  std::cin.get();
}

const SkeletonPtr makeBodyFromURDF(
    const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
    const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  dart::utils::DartLoader urdfLoader;
  const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
      ->setTransform(transform);
  return skeleton;
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
  auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);

  auto trajectory = robot.planToConfiguration(
      armSpace, armSkeleton, goalPos, nullptr, planningTimeout);

  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution");
  }

  auto smoothTrajectory
      = robot.postProcessPath<ParabolicSmoother>(
          trajectory.get(),
          testable,
          ParabolicSmoother::Params());
  aikido::trajectory::TrajectoryPtr timedTrajectory
      = std::move(robot.postProcessPath<KunzRetimer>(
          smoothTrajectory.get(),
          testable,
          KunzRetimer::Params()));

  auto future = robot.executeTrajectory(timedTrajectory);
  future.wait();
}

int main(int argc, char** argv)
{
  // Default options for flags
  int target;
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "soda_grasp");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("soda_grasp"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  ada::Ada robot(env, !adaReal);
  auto robotSkeleton = robot.getMetaSkeleton();

  // Load Soda Can in simulation
  ROS_INFO("Loading Can.");
  const std::string sodaName{"can"};
  const std::string sodaURDFUri("package://pr_ordata/data/objects/can.urdf");

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/soda_grasp";

  // Start the RViz viewer.
  ROS_INFO_STREAM(
      "Starting viewer. Please subscribe to the '"
      << execTopicName << "' InteractiveMarker topic in RViz.");
  aikido::rviz::InteractiveMarkerViewer viewer(
      execTopicName, baseFrameName, env);

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);

  // Predefined positions ////////////////////////////////////////////////////
  Eigen::VectorXd armRelaxedHome(Eigen::VectorXd::Ones(6));
  armRelaxedHome << 0.631769, -2.82569, -1.31347, -1.29491, -0.774963, 1.6772;

  auto arm = robot.getArm();
  auto armSkeleton = arm->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());
  auto hand = robot.getHand();
  armSkeleton->setPositions(armRelaxedHome);

  Eigen::Isometry3d sodaPose;
  sodaPose = Eigen::Isometry3d::Identity();
  sodaPose.translation() = Eigen::Vector3d(0.5, -0.142525, 0.502);
  auto soda = makeBodyFromURDF(resourceRetriever, sodaURDFUri, sodaPose);
  robot.getWorld()->addSkeleton(soda);

  // Get Soda Can TSR
  auto sodaTSR = pr_tsr::getDefaultCanTSR();
  sodaTSR.mT0_w = sodaPose;

  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  auto defaultPose = getCurrentConfig(robot);

  viewer.addFrameMarker(hand->getEndEffectorBodyNode(), 0.2, 0.01, 1.0);
  sodaTSR.mTw_e.matrix() *= hand->getEndEffectorTransform("cylinder")->matrix();
  auto goalTsr = std::make_shared<TSR>(sodaTSR);

  auto trajectory = robot.planToTSR(
      armSpace,
      armSkeleton,
      hand->getEndEffectorBodyNode(),
      goalTsr,
      nullptr,
      planningTimeout,
      5);

  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution");
  }

  auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);
  aikido::trajectory::TrajectoryPtr timedTrajectory
      = robot.postProcessPath<KunzRetimer>(
          trajectory.get(),
          testable,
          KunzRetimer::Params());

  auto future = robot.executeTrajectory(timedTrajectory);
  future.wait();

  getCurrentConfig(robot);

  std::cin.get();
  return 0;
}
