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

using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

using aikido::robot::Robot;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;

using aikido::planner::kunzretimer::KunzRetimer;
using aikido::planner::parabolic::ParabolicSmoother;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

dart::common::Uri adaUrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.urdf"};
dart::common::Uri adaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.srdf"};

static const double planningTimeout{5.};

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

  auto smoothTrajectory = robot.postProcessPath<ParabolicSmoother>(
      trajectory.get(), satisfied, ParabolicSmoother::Params());
  aikido::trajectory::TrajectoryPtr timedTrajectory
      = std::move(robot.postProcessPath<KunzRetimer>(
          smoothTrajectory.get(), satisfied, ada::KunzParams()));

  waitForUser("Press key to move arm to goal");
  auto future = robot.executeTrajectory(timedTrajectory);

  future.wait();
  getCurrentConfig(robot);
}

int main(int argc, char** argv)
{
  bool adaSim = true;

  // Default options for flags
  po::options_description po_desc("simple_trajectories options");
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
      << execTopicName << "' InteractiveMarker topic in RViz.");
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

  moveArmTo(robot, armSpace, armSkeleton, movedPose);

  /////////////////////////////////////////////////////////////////////////////
  //   Hardcoded Position (pointing down)
  /////////////////////////////////////////////////////////////////////////////
  Eigen::VectorXd homeConfig(6);
  homeConfig << -2.11666, 3.34967, 2.04129, -2.30031, -2.34026, 2.9545;
  std::vector<double> velocityLimits{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  std::cout << "Moving ARM to hard-coded position." << std::endl;
  waitForUser("Press [ENTER] to proceed:");
  robot.moveArmToConfiguration(
      homeConfig, nullptr, 2.0, velocityLimits);
  waitForUser("Done. \n Press [ENTER] to proceed:");
  

  /////////////////////////////////////////////////////////////////////////////
  //   Cartesian Velocity control (real only)
  /////////////////////////////////////////////////////////////////////////////
  if(!adaSim) {
    std::cout << "Enabling Cartesian Velocity control." << std::endl;
    robot.setVelocityControl(true);
    std::cout << "Moving 5cm in X" << std::endl;
    waitForUser("Press [ENTER] to proceed:");
    Eigen::Vector3d zero{0.0, 0.0, 0.0};
    Eigen::Vector3d Xvel{0.05, 0.0, 0.0};
    auto result = robot.moveArmCommandVelocity(Xvel, zero, ros::Duration(1.0), false);
    std::cout << "Waiting for result... ";
    result.wait();
    std::cout << "Done! Status: " << result.get() << std::endl;
    robot.setVelocityControl(false);
    std::cout <<"Returning to hard-coded position..." << std::endl;
    waitForUser("Press [ENTER] to proceed:");
    robot.moveArmToConfiguration(
      homeConfig, nullptr, 2.0, velocityLimits);
  }

  waitForUser("Press [ENTER] to exit. ");
  ros::shutdown();
  return 0;
}
