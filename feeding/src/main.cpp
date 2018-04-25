#include <iostream>
#include <Eigen/Dense>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <libada/Ada.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <chrono>
#include <thread>
#include <pr_tsr/can.hpp>

namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::constraint::dart::TSR;
static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

static const int maxNumberTrials{1};
static const double planningTimeout{5.};
bool adaReal = false;

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
  IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  // TODO (Tapo): Change this back once the robot vs. arm is cleared
  auto defaultPose = robot.getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
  return defaultPose;
}

void moveArmTo(ada::Ada& robot,
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

  auto smoothTrajectory = robot.smoothPath(armSkeleton, trajectory.get(), testable);
  aikido::trajectory::TrajectoryPtr timedTrajectory =
    std::move(robot.retimePath(armSkeleton, smoothTrajectory.get()));

  auto future = robot.executeTrajectory(timedTrajectory);
  future.wait();
}


void moveArmToTSR(aikido::constraint::dart::TSR& tsr,
                ada::Ada& robot,
                const MetaSkeletonStateSpacePtr& armSpace,
                const MetaSkeletonPtr& armSkeleton,
                const aikido::robot::HandPtr& hand) {
  auto goalTSR = std::make_shared<TSR> (tsr);

  auto trajectory = robot.planToTSR(armSpace, armSkeleton, hand->getBodyNode(),
    goalTSR, nullptr, maxNumberTrials, planningTimeout);

  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution");
  }

  auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);
  auto smoothTrajectory = robot.smoothPath(armSkeleton, trajectory.get(), testable);
  aikido::trajectory::TrajectoryPtr timedTrajectory = robot.retimePath(armSkeleton, smoothTrajectory.get());

  auto future = robot.executeTrajectory(timedTrajectory);
  future.wait();

  getCurrentConfig(robot);
}


int main(int argc, char** argv)
{
  // Default options for flags
  int target;
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "feeding");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("feedinfeedingg"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  ada::Ada robot(env, !adaReal);
  auto robotSkeleton = robot.getMetaSkeleton();

  // Load Soda Can in simulation
  ROS_INFO("Loading Can.");
  const std::string sodaName{"can"};
  const std::string sodaURDFUri("package://pr_ordata/data/objects/can.urdf");

  const auto resourceRetriever = std::make_shared<aikido::io::CatkinResourceRetriever>();

  // Start Visualization Topic
  static const std::string execTopicName = topicName + "/feeding";

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
  Eigen::VectorXd armRelaxedHome(Eigen::VectorXd::Ones(6));
  armRelaxedHome << 0.631769 , -2.82569  ,-1.31347,  -1.29491 ,-0.774963 ,  1.6772;

  auto arm = robot.getArm();
  auto armSkeleton = arm->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());
  auto hand = std::static_pointer_cast<ada::AdaHand>(robot.getHand());
  armSkeleton->setPositions(armRelaxedHome);

  Eigen::Isometry3d sodaPose;
  sodaPose = Eigen::Isometry3d::Identity();
  sodaPose.translation() = Eigen::Vector3d(0.5, -0.142525,  0.302); // 0.502
  auto soda = makeBodyFromURDF(resourceRetriever, sodaURDFUri, sodaPose);
  robot.getWorld()->addSkeleton(soda);

  // Get Soda Can TSR
  

  waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:");

  auto defaultPose = getCurrentConfig(robot);

  viewer.addFrame(hand->getBodyNode(), 0.2, 0.01, 1.0);

  // ***** MOVE ABOVE PLATE *****
  double heightAbovePlate = 0.2;
  double horizontal_tolerance_above_plate = 0.1;
  double vertical_tolerance_above_plate = 0.03;

  auto abovePlateTSR = pr_tsr::getDefaultCanTSR();
  abovePlateTSR.mT0_w = sodaPose;
  abovePlateTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAbovePlate};

  Eigen::MatrixXd abovePlateBw = Eigen::Matrix<double, 6, 2>::Zero();
  abovePlateBw(0, 0) = -horizontal_tolerance_above_plate;
  abovePlateBw(0, 1) = horizontal_tolerance_above_plate;
  abovePlateBw(1, 0) = -horizontal_tolerance_above_plate;
  abovePlateBw(1, 1) = horizontal_tolerance_above_plate;
  abovePlateBw(2, 0) = -vertical_tolerance_above_plate;
  abovePlateBw(2, 1) = vertical_tolerance_above_plate;
  abovePlateBw(5, 0) = -M_PI;
  abovePlateBw(5, 1) = M_PI;
  abovePlateTSR.mBw = abovePlateBw;

  //auto marker = viewer.addTSRMarker(abovePlateTSR, 20);

  abovePlateTSR.mTw_e.matrix() *= hand->getEndEffectorTransform("plate")->matrix();
  moveArmToTSR(abovePlateTSR, robot, armSpace, armSkeleton, hand);


  // ***** GET FOOD TSR *****
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  double heightAboveFood = 0.1;
  double horizontal_tolerance_near_food = 0.002;
  double vertical_tolerance_near_food = 0.002;

  auto foodTSR = pr_tsr::getDefaultCanTSR();
  Eigen::Isometry3d foodPose = sodaPose;
  // TODO
  //foodPose.translate()
  foodTSR.mT0_w = foodPose;
  foodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};

  Eigen::MatrixXd nearFoodBw = Eigen::Matrix<double, 6, 2>::Zero();
  nearFoodBw(0, 0) = -horizontal_tolerance_near_food;
  nearFoodBw(0, 1) = horizontal_tolerance_near_food;
  nearFoodBw(1, 0) = -horizontal_tolerance_near_food;
  nearFoodBw(1, 1) = horizontal_tolerance_near_food;
  nearFoodBw(2, 0) = -vertical_tolerance_near_food;
  nearFoodBw(2, 1) = vertical_tolerance_near_food;
  nearFoodBw(5, 0) = -M_PI;
  nearFoodBw(5, 1) = M_PI;
  foodTSR.mBw = nearFoodBw;
  foodTSR.mTw_e.matrix() *= hand->getEndEffectorTransform("plate")->matrix();

  // ***** MOVE ABOVE FOOD, INTO FOOD AND ABOVE PLATE *****

  aikido::constraint::dart::TSR aboveFoodTSR(foodTSR);
  aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAboveFood};

  //auto marker2 = viewer.addTSRMarker(aboveFoodTSR, 20);
  moveArmToTSR(aboveFoodTSR, robot, armSpace, armSkeleton, hand);

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  //auto marker3 = viewer.addTSRMarker(foodTSR, 20);
  moveArmToTSR(foodTSR, robot, armSpace, armSkeleton, hand);

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  moveArmToTSR(abovePlateTSR, robot, armSpace, armSkeleton, hand);


  // ***** MOVE TO PERSON *****

  auto personTSR = pr_tsr::getDefaultCanTSR();
  Eigen::Isometry3d personPose;
  personPose = Eigen::Isometry3d::Identity();
  personPose.translation() = Eigen::Vector3d(0.2, -0.32525,  0.602);
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};


  double horizontal_tolerance_near_person = 0.1;
  double vertical_tolerance_near_person = 0.1;

  Eigen::MatrixXd personBw = Eigen::Matrix<double, 6, 2>::Zero();
  personBw(0, 0) = -horizontal_tolerance_near_person;
  personBw(0, 1) = horizontal_tolerance_near_person;
  personBw(1, 0) = -horizontal_tolerance_near_person;
  personBw(1, 1) = horizontal_tolerance_near_person;
  personBw(2, 0) = -vertical_tolerance_near_person;
  personBw(2, 1) = vertical_tolerance_near_person;
  personTSR.mBw = personBw;
  personTSR.mTw_e.matrix() *= hand->getEndEffectorTransform("person")->matrix();

  //auto marker4 = viewer.addTSRMarker(personTSR, 20);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  moveArmToTSR(personTSR, robot, armSpace, armSkeleton, hand);

  std::cin.get();
  return 0;
}