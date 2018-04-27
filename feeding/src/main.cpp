#include <iostream>
#include <Eigen/Dense>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <libada/Ada.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <chrono>
#include <thread>
#include <pr_tsr/plate.hpp>

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
bool adaReal = false;

bool waitForUser(const std::string& msg)
{
  ROS_INFO(msg.c_str());
  char input = ' ';
  std::cin.get(input);
  return input != 'n';
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

void moveArmOnTrajectory(TrajectoryPtr trajectory,
                         ada::Ada& robot,
                         const MetaSkeletonStateSpacePtr& armSpace,
                         const MetaSkeletonPtr& armSkeleton, bool smooth = true)
{  
  // Example for moving to configuration
  //

  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution");
  }

  auto testable = std::make_shared<aikido::constraint::Satisfied>(armSpace);
  aikido::trajectory::TrajectoryPtr timedTrajectory;
  if (smooth) {
    ROS_INFO("smoothing...");
    auto smoothTrajectory = robot.smoothPath(armSkeleton, trajectory.get(), testable);
    ROS_INFO("timing...");
    timedTrajectory = std::move(robot.retimePath(armSkeleton, smoothTrajectory.get()));
  } else {
    ROS_INFO("timing...");
    timedTrajectory = std::move(robot.retimePath(armSkeleton, trajectory.get()));
  }

  ROS_INFO("executing...");
  auto future = robot.executeTrajectory(timedTrajectory);
  future.wait();
  ROS_INFO("movement done");

  getCurrentConfig(robot);
}

void moveArmToConfiguration(const Eigen::VectorXd& configuration,
                ada::Ada& robot,
                const MetaSkeletonStateSpacePtr& armSpace,
                const MetaSkeletonPtr& armSkeleton,
                const aikido::robot::HandPtr& hand,
                const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint = nullptr) {
  
  auto trajectory = robot.planToConfiguration(armSpace, armSkeleton, configuration, collisionFreeConstraint, planningTimeout);

  moveArmOnTrajectory(trajectory, robot, armSpace, armSkeleton);
}

void moveArmToTSR(aikido::constraint::dart::TSR& tsr,
                ada::Ada& robot,
                const MetaSkeletonStateSpacePtr& armSpace,
                const MetaSkeletonPtr& armSkeleton,
                const aikido::robot::HandPtr& hand,
                const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint = nullptr) {
  auto goalTSR = std::make_shared<TSR> (tsr);

  auto trajectory = robot.planToTSR(armSpace, armSkeleton, hand->getBodyNode(),
    goalTSR, collisionFreeConstraint, maxNumberTrials, planningTimeout);

  moveArmOnTrajectory(trajectory, robot, armSpace, armSkeleton);
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

  // Load Plate and FootItem in simulation
  ROS_INFO("Loading Plate and FoodItem.");
  const std::string plateName{"plate"};
  const std::string plateURDFUri("package://pr_ordata/data/objects/plate.urdf");
  const std::string tableName{"table"};
  const std::string tableURDFUri("package://pr_ordata/data/furniture/table.urdf");
  const std::string foodItemName{"foodItem"};
  const std::string foodItemURDFUri("package://pr_ordata/data/objects/food_item.urdf");
  const std::string tomName{"tom"};
  const std::string tomURDFUri("package://pr_ordata/data/objects/tom.urdf");

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

  // Predefined configurations ////////////////////////////////////////////////////
  Eigen::VectorXd armRelaxedHome(Eigen::VectorXd::Ones(6));
  armRelaxedHome << 0.631769 , -2.82569  ,-1.31347,  -1.29491 ,-0.774963 ,  1.6772;
  Eigen::VectorXd abovePlateConfig(Eigen::VectorXd::Ones(6));
  abovePlateConfig << 0.536541,  -3.39606,  -1.80746,  0.601788,  -1.88629,  -2.20747;
  Eigen::VectorXd inFrontOfPersonConfig(Eigen::VectorXd::Ones(6));
  inFrontOfPersonConfig << 1.09007,  -2.97579,  -0.563162,  -0.907691,  1.09752,  -1.47537;

  auto arm = robot.getArm();
  auto armSkeleton = arm->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());
  auto hand = std::static_pointer_cast<ada::AdaHand>(robot.getHand());
  armSkeleton->setPositions(armRelaxedHome);

  Eigen::Isometry3d platePose;
  platePose = Eigen::Isometry3d::Identity();
  platePose.translation() = Eigen::Vector3d(0.4, -0.142525,  0.102);
  Eigen::Isometry3d tablePose;
  tablePose = Eigen::Isometry3d::Identity();
  tablePose.translation() = Eigen::Vector3d(1.1, 0.05,  -0.63);
  Eigen::Isometry3d foodPose = platePose;
  // TODO
  //foodPose.translate()
  Eigen::Isometry3d personPose;
  personPose = Eigen::Isometry3d::Identity();
  personPose.translation() = Eigen::Vector3d(0.1, -0.77525,  0.502);
  Eigen::Isometry3d tomPose = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  tomPose.linear() = rotation;
  tomPose.translation() = personPose.translation();

  auto plate = makeBodyFromURDF(resourceRetriever, plateURDFUri, platePose);
  robot.getWorld()->addSkeleton(plate);
  auto table = makeBodyFromURDF(resourceRetriever, tableURDFUri, tablePose);
  robot.getWorld()->addSkeleton(table);
  auto foodItem = makeBodyFromURDF(resourceRetriever, foodItemURDFUri, foodPose);
  robot.getWorld()->addSkeleton(foodItem);
  auto tom = makeBodyFromURDF(resourceRetriever, tomURDFUri, tomPose);
  robot.getWorld()->addSkeleton(tom);

  // Setting up collisions
  CollisionDetectorPtr collisionDetector = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<CollisionGroup> armCollisionGroup = collisionDetector->createCollisionGroup(armSkeleton.get(), hand->getBodyNode());
  std::shared_ptr<CollisionGroup> envCollisionGroup = collisionDetector->createCollisionGroup(table.get());
  auto collisionFreeConstraint = std::make_shared<CollisionFree>(armSpace, armSkeleton, collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(armCollisionGroup, envCollisionGroup);
  //collisionFreeConstraint = nullptr;

  if (!waitForUser("You can view ADA in RViz now. \n Press [ENTER] to proceed:")) {return 0;}

  auto defaultPose = getCurrentConfig(robot);

  //viewer.addFrame(hand->getBodyNode(), 0.2, 0.01, 1.0);

  // ***** MOVE ABOVE PLATE *****
  double heightAbovePlate = 0.15;
  double horizontal_tolerance_above_plate = 0.05;
  double vertical_tolerance_above_plate = 0.03;

  auto abovePlateTSR = pr_tsr::getDefaultPlateTSR();
  abovePlateTSR.mT0_w = platePose;
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
  
  moveArmToConfiguration(abovePlateConfig, robot, armSpace, armSkeleton, hand, collisionFreeConstraint);
  //moveArmToTSR(abovePlateTSR, robot, armSpace, armSkeleton, hand, collisionFreeConstraint);

  // ***** GET FOOD TSR *****
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  double heightAboveFood = 0.07;
  double horizontal_tolerance_near_food = 0.002;
  double vertical_tolerance_near_food = 0.002;

  auto foodTSR = pr_tsr::getDefaultPlateTSR();
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
  moveArmToTSR(aboveFoodTSR, robot, armSpace, armSkeleton, hand, collisionFreeConstraint);

  //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  //auto marker3 = viewer.addTSRMarker(foodTSR, 20);


  try {
    ROS_INFO("planning...");
    auto intoFoodTrajectory = robot.getArm()->planToEndEffectorOffset(
      armSpace,
      armSkeleton,
      hand->getBodyNode(),
      collisionFreeConstraint,
      Eigen::Vector3d(0,0,-1),
      heightAboveFood,
      5,
      0.005,
      0.04);
    ROS_INFO("executing...");
    moveArmOnTrajectory(intoFoodTrajectory, robot, armSpace, armSkeleton, false);
    ROS_INFO("done");
  } catch (int e) {
    ROS_INFO("caught expection");
    return 1;
  }

  //moveArmToTSR(foodTSR, robot, armSpace, armSkeleton, hand, collisionFreeConstraint);

  hand->grab(foodItem);

  //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  //moveArmToConfiguration(abovePlateConfig, robot, armSpace, armSkeleton, hand);
  //moveArmToTSR(abovePlateTSR, robot, armSpace, armSkeleton, hand);
  
  try {
    ROS_INFO("planning...");
    auto abovePlateTrajectory = robot.getArm()->planToEndEffectorOffset(armSpace, armSkeleton, hand->getBodyNode(), nullptr, Eigen::Vector3d(0,0,1), heightAbovePlate, 5, 0.005, 0.04);
    ROS_INFO("executing...");
    moveArmOnTrajectory(abovePlateTrajectory, robot, armSpace, armSkeleton, false);
  } catch (int e) {
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
  //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  try {
    moveArmToConfiguration(inFrontOfPersonConfig, robot, armSpace, armSkeleton, hand, collisionFreeConstraint);
    //moveArmToTSR(personTSR, robot, armSpace, armSkeleton, hand, collisionFreeConstraint);
  } catch (int e) {
    ROS_INFO("caught expection when planning to person");
    return 1;
  }

  try {
    auto toPersonTrajectory = robot.getArm()->planToEndEffectorOffset(armSpace, armSkeleton, hand->getBodyNode(), collisionFreeConstraint, Eigen::Vector3d(0,-1,0), distanceToPerson, 5, 0.005, 0.04);
    moveArmOnTrajectory(toPersonTrajectory, robot, armSpace, armSkeleton, false);
  } catch (int e) {
    ROS_INFO("caught expection");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  hand->ungrab();
  robot.getWorld()->removeSkeleton(foodItem);

  try {
    auto toPersonTrajectory = robot.getArm()->planToEndEffectorOffset(armSpace, armSkeleton, hand->getBodyNode(), collisionFreeConstraint, Eigen::Vector3d(0,1,0), distanceToPerson/2, 5, 0.005, 0.04);
    moveArmOnTrajectory(toPersonTrajectory, robot, armSpace, armSkeleton, false);
  } catch (int e) {
    ROS_INFO("caught expection");
    return 1;
  }

  moveArmToConfiguration(abovePlateConfig, robot, armSpace, armSkeleton, hand, collisionFreeConstraint);

  std::cin.get();
  return 0;
}