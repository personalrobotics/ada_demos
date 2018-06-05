#include <chrono>
#include <iostream>
#include <thread>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/io/util.hpp>
#include <aikido/perception/ObjectDatabase.hpp>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <pr_control_msgs/SetForceTorqueThresholdAction.h>
#include <pr_tsr/plate.hpp>
#include <libada/Ada.hpp>

namespace po = boost::program_options;

using dart::dynamics::SkeletonPtr;
using dart::dynamics::MetaSkeletonPtr;
using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;

using SetFTThresholdAction = pr_control_msgs::SetForceTorqueThresholdAction;
using FTThresholdActionClient
    = actionlib::SimpleActionClient<SetFTThresholdAction>;

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::CollisionFree;
using aikido::constraint::TestablePtr;
using aikido::trajectory::TrajectoryPtr;
static const std::string foodsDataURI(
    "package://pr_ordata/data/objects/tag_data_foods.json");
static const std::string detectorTopicName("/deep_pose/marker_array");
static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

static const int maxNumberTrials{1};
static const double planningTimeout{5.};
static const double perceptionTimeout{5.};
static const double positionTolerance = 0.005;
static const double angularTolerance = 0.04;
static const double standardForceThreshold = 8;
static const double standardTorqueThreshold = 8;
static const double grabFoodForceThreshold = 22;
static const double grabFoodTorqueThreshold = 2;
static const double afterGrabForceThreshold = grabFoodForceThreshold + 10;
static const double afterGrabTorqueThreshold = grabFoodTorqueThreshold + 5;
static const double feedPersonForceThreshold = 2;
static const double feedPersonTorqueThreshold = 2;

bool adaSim = true;

bool waitForUser(const std::string& msg)
{
  ROS_INFO((msg + " Press [ENTER]").c_str());
  char input = ' ';
  std::cin.get(input);
  return input != 'n';
}

Eigen::Vector6d getCurrentConfig(ada::Ada& robot)
{
  using namespace Eigen;
  IOFormat CommaInitFmt(
      StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
  // TODO (Tapo): Change this back once the robot vs. arm is cleared
  auto defaultPose = robot.getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
  return defaultPose;
}

bool moveArmOnTrajectory(
    TrajectoryPtr trajectory,
    ada::Ada& robot,
    const MetaSkeletonStateSpacePtr& armSpace,
    const MetaSkeletonPtr& armSkeleton,
    const aikido::constraint::dart::CollisionFreePtr& collisionFreeConstraint)
{
  // Example for moving to configuration
  //

  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution");
  }

  //auto spaceConstraint = std::make_shared<aikido::constraint::Satisfied>(armSpace);
  std::vector<TestablePtr> constraints;
  if (collisionFreeConstraint) {
    constraints.push_back(collisionFreeConstraint);
  }
  auto testable = std::make_shared<aikido::constraint::TestableIntersection>(armSpace, constraints);
  aikido::trajectory::TrajectoryPtr timedTrajectory
        = robot.smoothPath(armSkeleton, trajectory.get(), testable);

  auto future = robot.executeTrajectory(std::move(timedTrajectory));
  try
  {
    future.get();
  }
  catch (const std::exception& e)
  {
    ROS_INFO_STREAM("trajectory execution failed: " << e.what());
    return false;
  }

  getCurrentConfig(robot);
  return true;
}

bool moveArmToConfiguration(
    const Eigen::Vector6d& configuration,
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

  return moveArmOnTrajectory(trajectory, robot, armSpace, armSkeleton, collisionFreeConstraint);
}

bool moveArmToTSR(
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
      hand->getEndEffectorBodyNode(),
      goalTSR,
      collisionFreeConstraint,
      maxNumberTrials,
      planningTimeout);

  return moveArmOnTrajectory(trajectory, robot, armSpace, armSkeleton, collisionFreeConstraint);
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

bool setFTThreshold(
    std::unique_ptr<FTThresholdActionClient>& actionClient,
    double forceThreshold,
    double torqueThreshold,
    double timeout = 5)
{
  if (!actionClient)
  {
    return false;
  }
  pr_control_msgs::SetForceTorqueThresholdGoal goal;
  goal.force_threshold = forceThreshold;
  goal.torque_threshold = torqueThreshold;
  actionClient->sendGoal(goal);
  bool finished_before_timeout
      = actionClient->waitForResult(ros::Duration(5.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = actionClient->getState();
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
        ROS_WARN("F/T Thresholds could not be set: %s",state.toString().c_str());
        return false;
    } else {
        ROS_INFO("F/T Thresholds set successfully");
        return true;
    }
    return false;
  }
  else {
    ROS_WARN("F/T Thresholds could not be set: Timeout");
    return false;
  }
}

void perceiveFood(
    ros::NodeHandle nh,
    std::string detectorTopicName,
    const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
    ada::Ada& robot)
{

  std::string detectorDataURI = "package://pr_ordata/data/objects/tag_data_foods.json";
  std::string referenceFrameName = "j2n6s200_link_base";

  aikido::robot::util::getBodyNodeOrThrow(
      *robot.getMetaSkeleton(), "j2n6s200_link_base");

  aikido::perception::PoseEstimatorModule objDetector(
      nh,
      detectorTopicName,
      std::make_shared<aikido::perception::ObjectDatabase>(
          resourceRetriever, detectorDataURI),
      resourceRetriever,
      referenceFrameName,
      aikido::robot::util::getBodyNodeOrThrow(
          *robot.getMetaSkeleton(), "j2n6s200_link_base"));

  objDetector.detectObjects(robot.getWorld(), ros::Duration(perceptionTimeout));
}

int main(int argc, char** argv)
{
  bool adaReal = false;
  bool autoContinue = false;

  // Default options for flags
  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()("help,h", "Produce help message")(
      "adareal,a", po::bool_switch(&adaReal), "Run ADA in real")(
      "continueAuto,c", po::bool_switch(&autoContinue));

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    return 0;
  }

  bool adaSim = !adaReal;
  std::cout << "Simulation Mode: " << adaSim << std::endl;

  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "feeding");
  ros::NodeHandle nh("~");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("feeding"));

  // Load ADA either in simulation or real based on arguments
  ROS_INFO("Loading ADA.");
  dart::common::Uri adaUrdfUri{
      "package://ada_description/robots_urdf/ada_with_camera_forque.urdf"};
  dart::common::Uri adaSrdfUri{
      "package://ada_description/robots_urdf/ada_with_camera_forque.srdf"};
  std::string endEffectorName = "j2n6s200_forque_end_effector";
  ada::Ada robot(env, adaSim, adaUrdfUri, adaSrdfUri, endEffectorName);

  auto robotSkeleton = robot.getMetaSkeleton();
  auto robotSpace = robot.getStateSpace();

  Eigen::Isometry3d robotPose = createIsometry(0.7, 0.1, 0.05, 0, 0, 3.1415);

  // Load Plate and FootItem in simulation
  ROS_INFO("Loading Plate and FoodItem.");
  const std::string plateURDFUri("package://pr_ordata/data/objects/plate.urdf");
  const std::string tableURDFUri(
      "package://pr_ordata/data/furniture/table_feeding.urdf");
  const std::string foodItemURDFUri(
      "package://pr_ordata/data/objects/food_item.urdf");
  const std::string tomURDFUri("package://pr_ordata/data/objects/tom.urdf");
  const std::string workspaceURDFUri(
      "package://pr_ordata/data/furniture/workspace_feeding_demo.urdf");

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
  if (!waitForUser("Startup step 1 complete."))
  {
    return 0;
  }

  // Predefined configurations
  // ////////////////////////////////////////////////////
  Eigen::Vector6d armRelaxedHome(Eigen::Vector6d::Ones());
  armRelaxedHome << 0, 3.38, 4.0, 0.6, -1.9, -2.2;
  Eigen::Vector6d abovePlateConfig(Eigen::Vector6d::Ones());
  abovePlateConfig << 1.3, -3.38, 4.5, 0.6, -1.9, -2.2;
  Eigen::Vector6d inFrontOfPersonConfig(Eigen::Vector6d::Ones());
  inFrontOfPersonConfig << -3.1, 3.8, 1.0, -2.3, 2.0, 2.1;

  auto arm = robot.getArm();
  auto armSkeleton = arm->getMetaSkeleton();
  auto armSpace = std::make_shared<MetaSkeletonStateSpace>(armSkeleton.get());
  auto hand = robot.getHand();

  if (adaSim)
    armSkeleton->setPositions(armRelaxedHome);

  std::cout << armSkeleton->getPositions().transpose() << std::endl;
  // Predefined poses
  Eigen::Isometry3d platePose
      = robotPose.inverse() * createIsometry(0.3, 0.25, 0.00);
  Eigen::Isometry3d foodPose = platePose;
  // origin is corner of table top
//   Eigen::Isometry3d tablePose
//       = robotPose.inverse() * createIsometry(0.76, 0.38, -0.735);
  Eigen::Isometry3d tablePose
      = robotPose.inverse() * createIsometry(0.76, 0.38, -0.755);
  Eigen::Isometry3d personPose
      = robotPose.inverse() * createIsometry(0.3, -0.2, 0.502);
  Eigen::Isometry3d tomPose
      = robotPose.inverse() * createIsometry(0.3, -0.2, 0.502, 0, 0, M_PI);
  Eigen::Isometry3d workspacePose
      = robotPose.inverse() * createIsometry(0, 0, 0);

  auto plate = loadSkeletonFromURDF(resourceRetriever, plateURDFUri, platePose);
  //robot.getWorld()->addSkeleton(plate);
  auto table = loadSkeletonFromURDF(resourceRetriever, tableURDFUri, tablePose);
  robot.getWorld()->addSkeleton(table);
  auto workspace = loadSkeletonFromURDF(
      resourceRetriever, workspaceURDFUri, workspacePose);
  robot.getWorld()->addSkeleton(workspace);
  auto foodItem
      = loadSkeletonFromURDF(resourceRetriever, foodItemURDFUri, foodPose);
  if (adaSim) {
    robot.getWorld()->addSkeleton(foodItem);
    foodItem->getRootBodyNode()->setCollidable(false);
  }
  auto tom = loadSkeletonFromURDF(resourceRetriever, tomURDFUri, tomPose);
  robot.getWorld()->addSkeleton(tom);

  // Setting up collisions
  CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(
          robot.getMetaSkeleton().get(), hand->getEndEffectorBodyNode());
  std::shared_ptr<CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup(
          table.get(), tom.get(), workspace.get());
  auto collisionFreeConstraint = std::make_shared<CollisionFree>(
      armSpace, armSkeleton, collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

  if (!adaSim)
  {
    std::cout << "Start trajectory executor" << std::endl;
    robot.startTrajectoryExecutor();
  }

  // Start F/T threshold action client
  // Important: do this after starting trajectory executor
  std::unique_ptr<FTThresholdActionClient> ftThresholdActionClient;
  if (adaReal) {
    ftThresholdActionClient = std::unique_ptr<FTThresholdActionClient>(new FTThresholdActionClient("/move_until_touch_topic_controller/set_forcetorque_threshold/"));
    ROS_INFO("Waiting for FT Threshold Action Server to start...");
    ftThresholdActionClient->waitForServer();
    ROS_INFO("FT Threshold Action Server started.");
  }
  bool setFTSuccessful = true;
  while (!setFTSuccessful && adaReal) {
    setFTSuccessful = setFTThreshold(ftThresholdActionClient, standardForceThreshold, standardTorqueThreshold);
    if (setFTSuccessful) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (!ros::ok()) {exit(0);}
  }

  auto currentPose = getCurrentConfig(robot);

  if (!waitForUser("Startup step 2 complete."))
  {
    return 0;
  }


  hand->executePreshape("closed").wait();

  // ***** MOVE ABOVE PLATE *****
  double heightAbovePlate = 0.15;
  double horizontal_tolerance_above_plate = 0.005;
  double vertical_tolerance_above_plate = 0.005;

  auto abovePlateTSR = pr_tsr::getDefaultPlateTSR();
  abovePlateTSR.mT0_w = platePose;
  abovePlateTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAbovePlate};

  abovePlateTSR.mBw = createBwMatrixForTSR(
      horizontal_tolerance_above_plate,
      vertical_tolerance_above_plate,
      0,
      0);
  abovePlateTSR.mTw_e.matrix()
      *= hand->getEndEffectorTransform("plate")->matrix();

  ROS_INFO_STREAM("Goal configuration\t" << abovePlateConfig.transpose());

  if (!autoContinue)
    if (!waitForUser("Move arm to default pose"))
      return 0;
  if (!ros::ok()) return 0;

  auto startState
      = robotSpace->getScopedStateFromMetaSkeleton(robotSkeleton.get());

  aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
  if (!collisionFreeConstraint->isSatisfied(startState, &collisionCheckOutcome))
  {
    throw std::runtime_error(
        "Robot is in collison: " + collisionCheckOutcome.toString());
  }
  ROS_INFO("Robot is not in collision");

  bool successMoveAbovePlate1 = moveArmToTSR(
      abovePlateTSR,
      robot,
      armSpace,
      armSkeleton,
      hand,
      collisionFreeConstraint);
  if (!successMoveAbovePlate1)
  {
    ROS_WARN("Trajectory execution failed. Exiting...");
    exit(0);
  }


  // ***** GET FOOD TSR WITH PERCEPTION *****
  if (adaReal) {
    if (!autoContinue)
      if (!waitForUser("Perceive food"))
        return 0;

    perceiveFood(nh, detectorTopicName, resourceRetriever, robot);

    for (int i=0; i<robot.getWorld()->getNumSkeletons(); i++) {
        ROS_INFO_STREAM(robot.getWorld()->getSkeleton(i)->getName());
    }

    std::string perceivedFoodName = "apricot_1";
    auto perceivedFood = robot.getWorld()->getSkeleton(perceivedFoodName);
    if (perceivedFood != nullptr)
    {
      foodPose.translation() = perceivedFood->getJoint(0)->getChildBodyNode()->getTransform().translation();
      ROS_INFO_STREAM("perceived x: " << foodPose.translation().x() << ", y: " << foodPose.translation().y() << ", z: " << foodPose.translation().z());
    } else {
      throw std::runtime_error("Error when perceiving food");
    }
  }
  

  // ***** GET FOOD TSR *****
  double heightAboveFood = 0.1;
  double heightIntoFood = adaReal ? 0.03 : 0.0;
  double horizontal_tolerance_near_food = 0.002;
  double vertical_tolerance_near_food = 0.002;

  auto foodTSR = pr_tsr::getDefaultPlateTSR();
  foodTSR.mT0_w = foodPose;
  foodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};

  foodTSR.mBw = createBwMatrixForTSR(
      horizontal_tolerance_near_food,
      vertical_tolerance_near_food,
      -M_PI,
      M_PI);
  foodTSR.mTw_e.matrix() *= hand->getEndEffectorTransform("plate")->matrix();

  // ***** MOVE ABOVE FOOD, INTO FOOD AND ABOVE PLATE *****

  aikido::constraint::dart::TSR aboveFoodTSR(foodTSR);
  aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAboveFood - heightIntoFood};

  if (!autoContinue)
    if (!waitForUser("Move arm above food"))
      return 0;
  if (!ros::ok()) return 0;
  bool successMoveAboveFood = moveArmToTSR(
      aboveFoodTSR,
      robot,
      armSpace,
      armSkeleton,
      hand,
      collisionFreeConstraint);
  if (!successMoveAboveFood)
  {
    ROS_WARN("Trajectory execution failed. Exiting...");
    exit(0);
  }

  try
  {
    if (!autoContinue)
      if (!waitForUser("Move arm into food"))
        return 0;
    if (!ros::ok()) return 0;

    setFTThreshold(
        ftThresholdActionClient,
        grabFoodForceThreshold,
        grabFoodTorqueThreshold);
    auto intoFoodTrajectory = robot.planToEndEffectorOffset(
        armSpace,
        armSkeleton,
        hand->getEndEffectorBodyNode(),
        collisionFreeConstraint,
        Eigen::Vector3d(0, 0, -1),
        heightAboveFood,
        planningTimeout,
        positionTolerance,
        angularTolerance);
    moveArmOnTrajectory(
        intoFoodTrajectory, robot, armSpace, armSkeleton, collisionFreeConstraint);
    setFTThreshold(
        ftThresholdActionClient,
        afterGrabForceThreshold,
        afterGrabTorqueThreshold);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  catch (int e)
  {
    ROS_INFO("caught expection");
    return 1;
  }
  if (adaSim) {
    hand->grab(foodItem);
  }

  if (!autoContinue)
    if (!waitForUser("Move arm above plate"))
      return 0;
  if (!ros::ok()) return 0;
  try
  {
    auto abovePlateTrajectory = robot.planToEndEffectorOffset(
        armSpace,
        armSkeleton,
        hand->getEndEffectorBodyNode(),
        collisionFreeConstraint,
        Eigen::Vector3d(0, 0, 1),
        heightAbovePlate,
        planningTimeout,
        positionTolerance,
        angularTolerance);
    bool successMoveAbovePlate2 = moveArmOnTrajectory(
        abovePlateTrajectory, robot, armSpace, armSkeleton, collisionFreeConstraint);
    if (!successMoveAbovePlate2)
    {
      ROS_WARN("Trajectory execution failed. Exiting...");
      exit(0);
    }
    setFTThreshold(
        ftThresholdActionClient,
        standardForceThreshold,
        standardTorqueThreshold);
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

  ROS_INFO_STREAM("Goal configuration\t" << inFrontOfPersonConfig.transpose());

  if (!autoContinue)
    if (!waitForUser("Move arm to person"))
      return 0;
  if (!ros::ok()) return 0;
  try
  {
    bool successMoveToPerson = moveArmToTSR(
        personTSR, robot, armSpace, armSkeleton, hand, collisionFreeConstraint);
    if (!successMoveToPerson)
    {
      ROS_WARN("Trajectory execution failed. Exiting...");
      exit(0);
    }
  }
  catch (int e)
  {
    ROS_INFO("caught expection when planning to person");
    return 1;
  }

  try
  {
    setFTThreshold(
        ftThresholdActionClient,
        feedPersonForceThreshold,
        feedPersonTorqueThreshold);
    auto toPersonTrajectory = robot.planToEndEffectorOffset(
        armSpace,
        armSkeleton,
        hand->getEndEffectorBodyNode(),
        collisionFreeConstraint,
        Eigen::Vector3d(0, 1, 0),
        distanceToPerson,
        planningTimeout,
        positionTolerance,
        angularTolerance);
    moveArmOnTrajectory(
        toPersonTrajectory, robot, armSpace, armSkeleton, collisionFreeConstraint);
    setFTThreshold(
        ftThresholdActionClient,
        standardForceThreshold,
        standardTorqueThreshold);
  }
  catch (int e)
  {
    ROS_INFO("caught expection");
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(6000));
  if (adaSim) {
    hand->ungrab();
    robot.getWorld()->removeSkeleton(foodItem);
  }

  try
  {
    auto toPersonTrajectory = robot.planToEndEffectorOffset(
        armSpace,
        armSkeleton,
        hand->getEndEffectorBodyNode(),
        collisionFreeConstraint,
        Eigen::Vector3d(0, -1, 0),
        distanceToPerson,
        planningTimeout,
        positionTolerance,
        angularTolerance);
    bool successMoveAwayFromPerson = moveArmOnTrajectory(
        toPersonTrajectory, robot, armSpace, armSkeleton, collisionFreeConstraint);
    if (!successMoveAwayFromPerson)
    {
      ROS_WARN("Trajectory execution failed. Exiting...");
      exit(0);
    }
  }
  catch (int e)
  {
    ROS_INFO("caught expection");
    return 1;
  }

  if (!autoContinue)
    if (!waitForUser("Move arm to plate"))
      return 0;
  if (!ros::ok()) return 0;

  bool successMoveAbovePlate3 = moveArmToTSR(
      abovePlateTSR,
      robot,
      armSpace,
      armSkeleton,
      hand,
      collisionFreeConstraint);
  if (!successMoveAbovePlate3)
  {
    ROS_WARN("Trajectory execution failed. Exiting...");
    exit(0);
  }

  waitForUser("Demo finished.");

  if (!adaSim)
  {
    std::cout << "Stop trajectory executor" << std::endl;
    robot.stopTrajectoryExecutor();
  }
  ros::shutdown();
  return 0;
}
