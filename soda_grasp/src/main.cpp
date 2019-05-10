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
#include <pr_tsr/can.hpp>
#include <libada/Ada.hpp>

#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>

using aikido::constraint::dart::createSampleableBounds;
using aikido::constraint::dart::InverseKinematicsSampleable;
using aikido::distance::NominalConfigurationRanker;
using aikido::statespace::dart::MetaSkeletonStateSaver;

using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;

namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::constraint::dart::TSR;
static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

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
      = robot.smoothPath(armSkeleton, trajectory.get(), testable);
  aikido::trajectory::TrajectoryPtr timedTrajectory
      = std::move(robot.retimePath(armSkeleton, smoothTrajectory.get()));

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
      << execTopicName
      << "' InteractiveMarker topic in RViz.");
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      env, execTopicName, baseFrameName);

  // Add ADA to the viewer.
  viewer.setAutoUpdate(true);

  // Predefined positions ////////////////////////////////////////////////////

  Eigen::VectorXd armRelaxedHome(Eigen::VectorXd::Zero(6));
  armRelaxedHome[1] = 3.14;
  armRelaxedHome[2] = 3.14;

  // Eigen::VectorXd armRelaxedHome(Eigen::VectorXd::Ones(6));
  // armRelaxedHome << 0.631769, -2.82569, -1.31347, -1.29491, -0.774963, 1.6772;

  Eigen::VectorXd goalConfigOut(6);

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

  waitForUser("KEK You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  auto defaultPose = getCurrentConfig(robot);

  viewer.addFrame(hand->getEndEffectorBodyNode(), 0.2, 0.01, 1.0);
  sodaTSR.mTw_e.matrix() *= hand->getEndEffectorTransform("cylinder")->matrix();
  auto goalTsr = std::make_shared<TSR>(sodaTSR);

  // Create an IK solver.
  InverseKinematicsPtr mIK = InverseKinematics::create(hand->getEndEffectorBodyNode());
  mIK->setDofs(armSkeleton->getDofs());

  InverseKinematicsSampleable ikSampleable(
    armSpace,
    armSkeleton,
    std::const_pointer_cast<aikido::constraint::dart::TSR>(goalTsr),
    createSampleableBounds(armSpace, robot.cloneRNG()),
    mIK,
    10);
  auto generator = ikSampleable.createSampleGenerator();

  // auto saver = MetaSkeletonStateSaver(armSkeleton);
  // DART_UNUSED(saver);

  auto coreSkeleton = armSkeleton->getBodyNode(0)->getSkeleton();

  std::vector<MetaSkeletonStateSpace::ScopedState> configurations;

  // Use ranker to pick the goal config closest to the start config.
  auto startState = armSpace->createState();
  armSpace->getState(armSkeleton.get(), startState);
  auto nominalState = armSpace->createState();
  armSpace->copyState(startState, nominalState);
  auto configurationRanker = std::make_shared<const NominalConfigurationRanker>(
      armSpace,
      armSkeleton,
      nominalState,
      std::vector<double>());

  auto goalState = armSpace->createState();

  // Sampling loop.
  static const std::size_t maxSamples{100};
  std::size_t samples = 0;
  while (samples < maxSamples && generator->canSample())
  {
    // Sample from TSR
    std::lock_guard<std::mutex> lock(coreSkeleton->getMutex());
    bool sampled = generator->sample(goalState);

    // Increment even if it's not a valid sample since this loop
    // has to terminate even if none are valid.
    ++samples;

    if (!sampled)
      continue;

    configurations.emplace_back(goalState.clone());
  }

  // NOTE: Zero config represents failure here!
  if (configurations.empty())
  {
    std::cout << "" << std::endl;
    std::cout << "Couldn't sample TSR IK!" << std::endl;

    return 1;
  }

  configurationRanker->rankConfigurations(configurations);

  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    // Now that configs are sorted, return the first free goal state.
    armSpace->convertStateToPositions(configurations[i], goalConfigOut);

    waitForUser("PRESS TO VIZ");
    armSkeleton->setPositions(goalConfigOut);

    waitForUser("PRESS TO PLAN");

    armSkeleton->setPositions(armRelaxedHome);

    auto trajectory = robot.planToConfiguration(
        armSpace, armSkeleton, goalConfigOut, nullptr, planningTimeout);

    if (trajectory)
    {
      auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);

      auto smoothTrajectory
        = robot.smoothPath(armSkeleton, trajectory.get(), testable);
      aikido::trajectory::TrajectoryPtr timedTrajectory
          = std::move(robot.retimePath(armSkeleton, smoothTrajectory.get()));

      waitForUser("ENTER TO EXEC!");
      auto future = robot.executeTrajectory(timedTrajectory);
      future.wait();

      return 0;
    }
  }



  // moveArmTo(robot, armSpace, armSkeleton, goalConfigOut);

  //
  // auto trajectory = robot.planToTSR(
  //     armSpace,
  //     armSkeleton,
  //     hand->getEndEffectorBodyNode(),
  //     goalTsr,
  //     nullptr,
  //     20.0,
  //     100);
  //
  // if (!trajectory)
  // {
  //   throw std::runtime_error("Failed to find a solution");
  // }
  //
  // auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);
  // aikido::trajectory::TrajectoryPtr timedTrajectory
  //     = robot.retimePath(armSkeleton, trajectory.get());
  //
  // auto future = robot.executeTrajectory(timedTrajectory);
  // future.wait();
  //
  // getCurrentConfig(robot);
  //
  // std::cin.get();
  return 0;
}
