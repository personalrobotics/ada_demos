#include <chrono>
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/io/util.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pr_tsr/plate.hpp>
#include <libada/Ada.hpp>

namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;
using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::CollisionFree;
using aikido::trajectory::TrajectoryPtr;
static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

static const int maxNumberTrials{1};
static const double planningTimeout{5.};
static const double positionTolerance = 0.005;
static const double angularTolerance = 0.04;
bool adaReal = false;
bool feeding = true;

bool waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  char input = ' ';
  std::cin.get(input);
  return input != 'n';
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

void moveArmOnTrajectory(
    TrajectoryPtr trajectory,
    ada::Ada& robot,
    const MetaSkeletonStateSpacePtr& armSpace,
    const MetaSkeletonPtr& armSkeleton,
    bool smooth = true)
{
  // Example for moving to configuration
  //

  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution");
  }

  auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);
  aikido::trajectory::TrajectoryPtr timedTrajectory;
  if (smooth)
  {
    ROS_INFO("smoothing...");
    auto smoothTrajectory
        = robot.smoothPath(armSkeleton, trajectory.get(), testable);
    ROS_INFO("timing...");
    timedTrajectory
        = std::move(robot.retimePath(armSkeleton, smoothTrajectory.get()));
  }
  else
  {
    ROS_INFO("timing...");
    timedTrajectory
        = std::move(robot.retimePath(armSkeleton, trajectory.get()));
  }

  ROS_INFO("executing...");
  auto future = robot.executeTrajectory(timedTrajectory);
  future.wait();
  ROS_INFO("movement done");

  getCurrentConfig(robot);
}

void moveArmToConfiguration(
    const Eigen::VectorXd& configuration,
    ada::Ada& robot,
    const MetaSkeletonStateSpacePtr& armSpace,
    const MetaSkeletonPtr& armSkeleton,
    const aikido::robot::HandPtr& hand,
    const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint
    = nullptr)
{

  auto trajectory = robot.planToConfiguration(
      armSpace,
      armSkeleton,
      configuration,
      collisionFreeConstraint,
      planningTimeout);

  moveArmOnTrajectory(trajectory, robot, armSpace, armSkeleton);
}

void moveArmToTSR(
    aikido::constraint::dart::TSR& tsr,
    ada::Ada& robot,
    const MetaSkeletonStateSpacePtr& armSpace,
    const MetaSkeletonPtr& armSkeleton,
    const aikido::robot::HandPtr& hand,
    const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint
    = nullptr)
{
  auto goalTSR = std::make_shared<TSR>(tsr);

  auto trajectory = robot.planToTSR(
      armSpace,
      armSkeleton,
      hand->getHandBaseBodyNode(),
      goalTSR,
      collisionFreeConstraint,
      maxNumberTrials,
      planningTimeout);

  moveArmOnTrajectory(trajectory, robot, armSpace, armSkeleton);
}

Eigen::Isometry3d createIsometry(
    double x,
    double y,
    double z,
    double roll = 0,
    double pitch = 0,
    double yaw = 0)
{
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(x, y, z);
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;

  return isometry;
}

Eigen::MatrixXd createBwMatrixForTSR(
    double horizontalTolerance,
    double verticalTolerance,
    double yawMin,
    double yawMax)
{
  Eigen::MatrixXd bw = Eigen::Matrix<double, 6, 2>::Zero();
  bw(0, 0) = -horizontalTolerance;
  bw(0, 1) = horizontalTolerance;
  bw(1, 0) = -horizontalTolerance;
  bw(1, 1) = horizontalTolerance;
  bw(2, 0) = -verticalTolerance;
  bw(2, 1) = verticalTolerance;
  bw(5, 0) = yawMin;
  bw(5, 1) = yawMax;
  return bw;
}

int main(int argc, char** argv)
{
  // Default options for flags
  int target;
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "feeding");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("feeding"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  dart::common::Uri adaUrdfUri{"package://ada_description/robots/ada_with_camera_forque.urdf"};
  dart::common::Uri adaSrdfUri{"package://ada_description/robots/ada_with_camera_forque.srdf"};
  std::string endEffectorName = "j2n6s200_forque_end_effector";
  ada::Ada robot(env, !adaReal, adaUrdfUri, adaSrdfUri, endEffectorName);
  auto robotSkeleton = robot.getMetaSkeleton();

  // Load Plate and FootItem in simulation
  ROS_INFO("Loading Plate and FoodItem.");
  const std::string plateName{"plate"};
  const std::string plateURDFUri("package://pr_ordata/data/objects/plate.urdf");
  const std::string tableName{"table"};
  const std::string tableURDFUri(
      "package://pr_ordata/data/furniture/table.urdf");
  const std::string foodItemName{"foodItem"};
  const std::string foodItemURDFUri(
      "package://pr_ordata/data/objects/food_item.urdf");
  const std::string tomName{"tom"};
  const std::string tomURDFUri("package://pr_ordata/data/objects/tom.urdf");

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

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

  // Predefined configurations
  // ////////////////////////////////////////////////////
  Eigen::VectorXd armRelaxedHome(Eigen::VectorXd::Ones(6));
  armRelaxedHome << 0.631769, -2.82569, -1.31347, -1.29491, -0.774963, 1.6772;
  Eigen::VectorXd abovePlateConfig(Eigen::VectorXd::Ones(6));
  abovePlateConfig << 0.536541, -3.39606, -1.80746, 0.601788, -1.88629,
      -2.20747;
  Eigen::VectorXd inFrontOfPersonConfig(Eigen::VectorXd::Ones(6));
  inFrontOfPersonConfig << 1.09007, -2.97579, -0.563162, -0.907691, 1.09752,
      -1.47537;

  auto arm = robot.getArm();
  auto armSkeleton = arm->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());
  auto hand = robot.getHand();
  armSkeleton->setPositions(armRelaxedHome);

  // Predefined poses
  Eigen::Isometry3d platePose = createIsometry(0.4, -0.142525, 0.102);
  Eigen::Isometry3d tablePose = createIsometry(1.1, 0.05, -0.64);
  Eigen::Isometry3d foodPose = platePose;
  Eigen::Isometry3d personPose = createIsometry(0.1, -0.77525, 0.502);
  Eigen::Isometry3d tomPose = createIsometry(0.1, -0.77525, 0.502, 0, 0, M_PI);

  auto plate = loadSkeletonFromURDF(resourceRetriever, plateURDFUri, platePose);
  robot.getWorld()->addSkeleton(plate);
  auto table = loadSkeletonFromURDF(resourceRetriever, tableURDFUri, tablePose);
  robot.getWorld()->addSkeleton(table);
  auto foodItem
      = loadSkeletonFromURDF(resourceRetriever, foodItemURDFUri, foodPose);
  robot.getWorld()->addSkeleton(foodItem);
  auto tom = loadSkeletonFromURDF(resourceRetriever, tomURDFUri, tomPose);
  robot.getWorld()->addSkeleton(tom);

  // Setting up collisions
  CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(
          armSkeleton.get(), hand->getHandBaseBodyNode());
  std::shared_ptr<CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup(table.get(), tom.get());
  auto collisionFreeConstraint = std::make_shared<CollisionFree>(
      armSpace, armSkeleton, collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

  if (!waitForUser(
          "You can view ADA in RViz now. \n Press [ENTER] to proceed:"))
  {
    return 0;
  }

  auto defaultPose = getCurrentConfig(robot);

  // ***** MOVE ABOVE PLATE *****
  double heightAbovePlate = 0.15;
  double horizontal_tolerance_above_plate = 0.05;
  double vertical_tolerance_above_plate = 0.03;

  auto abovePlateTSR = pr_tsr::getDefaultPlateTSR();
  abovePlateTSR.mT0_w = platePose;
  abovePlateTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAbovePlate};

  abovePlateTSR.mBw = createBwMatrixForTSR(
      horizontal_tolerance_above_plate,
      vertical_tolerance_above_plate,
      -M_PI,
      M_PI);
  abovePlateTSR.mTw_e.matrix()
      *= hand->getEndEffectorTransform("plate")->matrix();

  moveArmToConfiguration(
      abovePlateConfig,
      robot,
      armSpace,
      armSkeleton,
      hand,
      collisionFreeConstraint);
  // moveArmToTSR(abovePlateTSR, robot, armSpace, armSkeleton, hand,
  // collisionFreeConstraint);

  // ***** GET FOOD TSR *****
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  double heightAboveFood = 0.07;
  double horizontal_tolerance_near_food = 0.002;
  double vertical_tolerance_near_food = 0.002;

  auto foodTSR = pr_tsr::getDefaultPlateTSR();
  foodTSR.mT0_w = foodPose;
  foodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0.02};

  foodTSR.mBw = createBwMatrixForTSR(
      horizontal_tolerance_near_food,
      vertical_tolerance_near_food,
      -M_PI,
      M_PI);
  foodTSR.mTw_e.matrix() *= hand->getEndEffectorTransform("plate")->matrix();

  // ***** MOVE ABOVE FOOD, INTO FOOD AND ABOVE PLATE *****

  aikido::constraint::dart::TSR aboveFoodTSR(foodTSR);
  aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAboveFood};

  moveArmToTSR(
      aboveFoodTSR,
      robot,
      armSpace,
      armSkeleton,
      hand,
      collisionFreeConstraint);

  try
  {
    ROS_INFO("planning...");
    auto intoFoodTrajectory = robot.planToEndEffectorOffset(
        armSpace,
        armSkeleton,
        hand->getHandBaseBodyNode(),
        collisionFreeConstraint,
        Eigen::Vector3d(0, 0, -1),
        heightAboveFood,
        planningTimeout,
        positionTolerance,
        angularTolerance);
    ROS_INFO("executing...");
    moveArmOnTrajectory(
        intoFoodTrajectory, robot, armSpace, armSkeleton, false);
    ROS_INFO("done");
  }
  catch (int e)
  {
    ROS_INFO("caught expection");
    return 1;
  }

  hand->grab(foodItem);

  try
  {
    ROS_INFO("planning...");
    auto abovePlateTrajectory = robot.planToEndEffectorOffset(
        armSpace,
        armSkeleton,
        hand->getHandBaseBodyNode(),
        collisionFreeConstraint,
        Eigen::Vector3d(0, 0, 1),
        heightAbovePlate,
        planningTimeout,
        positionTolerance,
        angularTolerance);
    ROS_INFO("executing...");
    moveArmOnTrajectory(
        abovePlateTrajectory, robot, armSpace, armSkeleton, false);
  }
  catch (int e)
  {
    ROS_INFO("caught expection");
    return 1;
  }

  // ***** MOVE TO PERSON *****
  double distanceToPerson = 0.25;

  auto personTSR = pr_tsr::getDefaultPlateTSR();
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  double horizontal_tolerance_near_person = 0.01;
  double vertical_tolerance_near_person = 0.004;

  personTSR.mBw = createBwMatrixForTSR(
      horizontal_tolerance_near_person, vertical_tolerance_near_person, 0, 0);
  personTSR.mTw_e.matrix() *= hand->getEndEffectorTransform("person")->matrix();

  try
  {
    moveArmToConfiguration(
        inFrontOfPersonConfig,
        robot,
        armSpace,
        armSkeleton,
        hand,
        collisionFreeConstraint);
    // moveArmToTSR(personTSR, robot, armSpace, armSkeleton, hand,
    // collisionFreeConstraint);
  }
  catch (int e)
  {
    ROS_INFO("caught expection when planning to person");
    return 1;
  }

  try
  {
    auto toPersonTrajectory = robot.planToEndEffectorOffset(
        armSpace,
        armSkeleton,
        hand->getHandBaseBodyNode(),
        collisionFreeConstraint,
        Eigen::Vector3d(0, -1, 0),
        distanceToPerson,
        planningTimeout,
        positionTolerance,
        angularTolerance);
    moveArmOnTrajectory(
        toPersonTrajectory, robot, armSpace, armSkeleton, false);
  }
  catch (int e)
  {
    ROS_INFO("caught expection");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  hand->ungrab();
  robot.getWorld()->removeSkeleton(foodItem);

  try
  {
    auto toPersonTrajectory = robot.planToEndEffectorOffset(
        armSpace,
        armSkeleton,
        hand->getHandBaseBodyNode(),
        collisionFreeConstraint,
        Eigen::Vector3d(0, 1, 0),
        distanceToPerson / 2,
        planningTimeout,
        positionTolerance,
        angularTolerance);
    moveArmOnTrajectory(
        toPersonTrajectory, robot, armSpace, armSkeleton, false);
  }
  catch (int e)
  {
    ROS_INFO("caught expection");
    return 1;
  }

  moveArmToConfiguration(
      abovePlateConfig,
      robot,
      armSpace,
      armSkeleton,
      hand,
      collisionFreeConstraint);
  // moveArmToTSR(abovePlateTSR, robot, armSpace, armSkeleton, hand,
  // collisionFreeConstraint);

  std::cin.get();
  return 0;
}
