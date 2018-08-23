#include "feeding/FeedingDemo.hpp"
#include <pr_tsr/plate.hpp>
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
FeedingDemo::FeedingDemo(
    bool adaReal, bool useFTSensing, ros::NodeHandle nodeHandle)
  : mAdaReal(adaReal), mNodeHandle(nodeHandle)
{

  mWorld = std::make_shared<aikido::planner::World>("feeding");

  std::string armTrajectoryExecutor = useFTSensing
                                          ? "move_until_touch_topic_controller"
                                          : "rewd_trajectory_controller";

  mAda = std::unique_ptr<ada::Ada>(
      new ada::Ada(
          mWorld,
          !mAdaReal,
          getRosParam<std::string>("/ada/urdfUri", mNodeHandle),
          getRosParam<std::string>("/ada/srdfUri", mNodeHandle),
          getRosParam<std::string>("/ada/endEffectorName", mNodeHandle),
          armTrajectoryExecutor));
  mArmSpace
      = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          mAda->getArm()->getMetaSkeleton().get());

  Eigen::Isometry3d robotPose = createIsometry(
      getRosParam<std::vector<double>>("/ada/baseFramePose", mNodeHandle));

  mWorkspace = std::unique_ptr<Workspace>(
      new Workspace(mWorld, robotPose, mAdaReal, mNodeHandle));

  // Setting up collisions
  dart::collision::CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(
          mAda->getMetaSkeleton().get(),
          mAda->getHand()->getEndEffectorBodyNode());
  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup(
          mWorkspace->getTable().get(),
          mWorkspace->getWorkspaceEnvironment().get(),
          mWorkspace->getWheelchair().get());
  mCollisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          mArmSpace, mAda->getArm()->getMetaSkeleton(), collisionDetector);
  mCollisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

  mAdaMover = std::unique_ptr<AdaMover>(
      new AdaMover(*mAda, mArmSpace, mCollisionFreeConstraint, nodeHandle));

  if (mAdaReal)
  {
    mAda->startTrajectoryExecutor();
  }
}

//==============================================================================
FeedingDemo::~FeedingDemo()
{
  if (mAdaReal)
  {
    // wait for a bit so controller actually stops moving
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    mAda->stopTrajectoryExecutor();
  }
}

//==============================================================================
aikido::planner::WorldPtr FeedingDemo::getWorld()
{
  return mWorld;
}

//==============================================================================
Workspace& FeedingDemo::getWorkspace()
{
  return *mWorkspace;
}

//==============================================================================
ada::Ada& FeedingDemo::getAda()
{
  return *mAda;
}

//==============================================================================
Eigen::Isometry3d FeedingDemo::getDefaultFoodTransform()
{
  return mWorkspace->getDefaultFoodItem()
      ->getRootBodyNode()
      ->getWorldTransform();
}

//==============================================================================
bool FeedingDemo::isCollisionFree(std::string& result)
{
  auto robotState = mAda->getStateSpace()->getScopedStateFromMetaSkeleton(
      mAda->getMetaSkeleton().get());
  aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
  if (!mCollisionFreeConstraint->isSatisfied(
          robotState, &collisionCheckOutcome))
  {
    result = "Robot is in collison: " + collisionCheckOutcome.toString();
    return false;
  }
  result = "Robot is not in collision";
  return true;
}

//==============================================================================
void FeedingDemo::printRobotConfiguration()
{
  Eigen::IOFormat CommaInitFmt(
      Eigen::StreamPrecision,
      Eigen::DontAlignCols,
      ", ",
      ", ",
      "",
      "",
      " << ",
      ";");
  auto defaultPose = mAda->getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
}

//==============================================================================
void FeedingDemo::openHand()
{
  mAda->getHand()->executePreshape("open").wait();
}

//==============================================================================
void FeedingDemo::closeHand()
{
  mAda->getHand()->executePreshape("closed").wait();
}

//==============================================================================
void FeedingDemo::grabFoodWithForque()
{
  if (!mAdaReal && mWorkspace->getDefaultFoodItem())
  {
    mAda->getHand()->grab(mWorkspace->getDefaultFoodItem());
  }
}

//==============================================================================
void FeedingDemo::ungrabAndDeleteFood()
{
  if (!mAdaReal)
  {
    mAda->getHand()->ungrab();
    mWorkspace->deleteFood();
  }
}

//==============================================================================
void FeedingDemo::moveToStartConfiguration()
{
  auto home
      = getRosParam<std::vector<double>>("/ada/homeConfiguration", mNodeHandle);
  if (mAdaReal)
  {
    // We decided to not move to an initial configuration for now
    // mAdaMover->moveArmToConfiguration(Eigen::Vector6d(home.data()));
  }
  else
  {
    mAda->getArm()->getMetaSkeleton()->setPositions(
        Eigen::Vector6d(home.data()));
  }
}

//==============================================================================
void FeedingDemo::moveAbovePlate()
{
  double heightAbovePlate
      = getRosParam<double>("/feedingDemo/heightAbovePlate", mNodeHandle);
  double horizontalToleranceAbovePlate = getRosParam<double>(
      "/planning/tsr/horizontalToleranceAbovePlate", mNodeHandle);
  double verticalToleranceAbovePlate = getRosParam<double>(
      "/planning/tsr/verticalToleranceAbovePlate", mNodeHandle);

  auto abovePlateTSR = pr_tsr::getDefaultPlateTSR();
  abovePlateTSR.mT0_w
      = mWorkspace->getPlate()->getRootBodyNode()->getWorldTransform();
  abovePlateTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAbovePlate};

  abovePlateTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceAbovePlate, verticalToleranceAbovePlate, 0, 0);
  abovePlateTSR.mTw_e.matrix()
      *= mAda->getHand()->getEndEffectorTransform("plate")->matrix();

  bool trajectoryCompleted = mAdaMover->moveArmToTSR(abovePlateTSR);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveAboveFood(const Eigen::Isometry3d& foodTransform)
{
  double heightAboveFood
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle);
  // If the robot is not simulated, we want to plan the trajectory to move a
  // little further downwards,
  // so that the MoveUntilTouchController can take care of stopping the
  // trajectory.
  double heightIntoFood
      = mAdaReal
            ? getRosParam<double>("/feedingDemo/heightIntoFood", mNodeHandle)
            : 0.0;
  double horizontalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearFood", mNodeHandle);
  double verticalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearFood", mNodeHandle);

  aikido::constraint::dart::TSR aboveFoodTSR;
  aboveFoodTSR.mT0_w = foodTransform;
  aboveFoodTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearFood, verticalToleranceNearFood, M_PI, M_PI);
  Eigen::Isometry3d eeTransform = *mAda->getHand()->getEndEffectorTransform("plate");
  eeTransform.linear() = eeTransform.linear() * Eigen::Matrix3d(Eigen::AngleAxisd(M_PI * 0.05, Eigen::Vector3d::UnitX()));
//   eeTransform.linear() = eeTransform.linear() * Eigen::Matrix3d(Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI * -0.1, Eigen::Vector3d::UnitX()));
  aboveFoodTSR.mTw_e.matrix()
      *= eeTransform.matrix();
  aboveFoodTSR.mTw_e.translation()
      = Eigen::Vector3d{0, 0, heightAboveFood - heightIntoFood};

  bool trajectoryCompleted = mAdaMover->moveArmToTSR(aboveFoodTSR);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveIntoFood()
{
  bool trajectoryCompleted = mAdaMover->moveToEndEffectorOffset(
      Eigen::Vector3d(0, 0, -1),
      getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle));
  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
}

//==============================================================================
void FeedingDemo::moveIntoFood(
    Perception* perception,
    aikido::rviz::WorldInteractiveMarkerViewerPtr viewer)
{
  ROS_INFO("Servoing into food");
  std::shared_ptr<aikido::control::TrajectoryExecutor> executor
      = mAda->getTrajectoryExecutor();

  std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> rosExecutor
      = std::dynamic_pointer_cast<aikido::control::ros::RosTrajectoryExecutor>(
          executor);

  if (rosExecutor == nullptr)
  {
    throw std::runtime_error("no ros executor");
  }

  /*
  std::unique_ptr<PerceptionServoClient> servoClient
  = std::unique_ptr<PerceptionServoClient>(new PerceptionServoClient
  (
      mNodeHandle,
      boost::bind(&Perception::perceiveFood, perception, _1),
      mArmSpace,
      mAdaMover,
      mAda->getArm()->getMetaSkeleton(),
      mAda->getHand()->getEndEffectorBodyNode(),
      rosExecutor,
      mCollisionFreeConstraint,
      viewer,
      0.1,
      5e-3));
  servoClient->start();

  servoClient->wait(20.0);
  */

  int numDofs = mAda->getArm()->getMetaSkeleton()->getNumDofs();
  Eigen::VectorXd velocityLimits = Eigen::VectorXd::Zero(numDofs);
  for (int i = 0; i < numDofs; i++)
    velocityLimits[i] = 0.2;

  PerceptionServoClient servoClient(
      mNodeHandle,
      boost::bind(&Perception::perceiveFood, perception, _1),
      mArmSpace,
      mAdaMover.get(),
      mAda->getArm()->getMetaSkeleton(),
      mAda->getHand()->getEndEffectorBodyNode(),
      rosExecutor,
      mCollisionFreeConstraint,
      0.1,
      velocityLimits);
  servoClient.start();

  servoClient.wait(10000.0);
}

//==============================================================================
void FeedingDemo::moveOutOfFood()
{
  bool trajectoryCompleted = mAdaMover->moveToEndEffectorOffset(
      Eigen::Vector3d(0, 0, 1),
      getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle));
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveInFrontOfPerson()
{
//   Eigen::Vector6d config;
//   config << -2.762510048228535, 4.3929478676357565, 4.9031890261545445, -2.408192444160033, 1.662426112524599, -0.9805006790663807;
//   mAdaMover->moveArmToConfiguration(config);
//   return;

  double distanceToPerson
      = getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle);
  double horizontalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearPerson", mNodeHandle);
  double verticalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearPerson", mNodeHandle);

//   for (float i= 0.25; i>=-0.01; i-=0.05) {
    //   ROS_INFO_STREAM("trying i = " << i);
  aikido::constraint::dart::TSR personTSR;
  Eigen::Isometry3d personPose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d personTranslation;
// working with -0.15*M_PI rotation around X
//   personTranslation << 0.296, 0.328, 0.70;
//   personTranslation << 0.296, 0.278, 0.75;

// working with rotation around Y and Z
  personTranslation << 0.296, 0.328, 0.70;
  personTranslation = mWorkspace->getPersonPose().translation();
//   personPose.translation() = personTranslation;
  personPose.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  personTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearPerson, verticalToleranceNearPerson, 0, 0);
  Eigen::Isometry3d eeTransform = *mAda->getHand()->getEndEffectorTransform("person");
//   eeTransform.linear() = eeTransform.linear() * Eigen::Matrix3d(Eigen::AngleAxisd(M_PI * -0.15, Eigen::Vector3d::UnitX()));
//   eeTransform.linear() = eeTransform.linear() * Eigen::Matrix3d(Eigen::AngleAxisd(M_PI * -0.5, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ()));
  personTSR.mTw_e.matrix()
      *= eeTransform.matrix();

    bool trajectoryCompleted = false;
  try {
    trajectoryCompleted = mAdaMover->moveArmToTSR(personTSR);
  } catch(std::runtime_error e) {
      
  }
  if (!trajectoryCompleted)
  {
    // throw std::runtime_error("Trajectory execution failed");
    ROS_WARN("failed!");
  }
//   }
}

void FeedingDemo::moveInFrontOfPerson2()
{
  double duration = 1;
  double timelimit = 5;

//   Eigen::Vector6d transform;
//   transform << -30.0/180.0*M_PI, 0, 0, 0, 0, 0;
// //   transform << 15.0/180.0*M_PI, 0, 0, 0, 0, 0;
//   mAdaMover->moveWithEndEffectorTwist(transform, duration, timelimit);

//   Eigen::Vector6d transform2;
//   transform2 << 0, 0, -M_PI/6, 0, 0, 0;
//   mAdaMover->moveWithEndEffectorTwist(transform2, duration, timelimit);

  
  double distanceToPerson
      = getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle);
  double horizontalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearPerson", mNodeHandle);
  double verticalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearPerson", mNodeHandle);

  aikido::constraint::dart::TSR personTSR;
  Eigen::Isometry3d personPose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d personTranslation;
// working with -0.15*M_PI rotation around X
//   personTranslation << 0.296, 0.328, 0.70;
//   personTranslation << 0.296, 0.278, 0.75;

// working with rotation around Y and Z
//   personTranslation << 0.296, 0.328 - (i*0.3), 0.70;
  personTranslation << 0.296, 0.328, 0.70;
  personPose.translation() = personTranslation;
  personPose.linear()
      = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, 0 , 0};

  personTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearPerson, verticalToleranceNearPerson, 0, 0);
  Eigen::Isometry3d eeTransform = *mAda->getHand()->getEndEffectorTransform("person");
//   eeTransform.linear() = eeTransform.linear() * Eigen::Matrix3d(Eigen::AngleAxisd(M_PI * -0.15, Eigen::Vector3d::UnitX()));
  eeTransform.linear() = eeTransform.linear() * Eigen::Matrix3d(Eigen::AngleAxisd(M_PI * -0.5, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ()));
  personTSR.mTw_e.matrix()
      *= eeTransform.matrix();

    bool trajectoryCompleted = false;
  try {
    trajectoryCompleted = mAdaMover->moveArmToTSR(personTSR);
  } catch(std::runtime_error e) {
      
  }
  if (!trajectoryCompleted)
  {
    // throw std::runtime_error("Trajectory execution failed");
    ROS_WARN("failed!");
  }
}

//==============================================================================
void FeedingDemo::moveTowardsPerson()
{
  bool trajectoryCompleted = mAdaMover->moveToEndEffectorOffset(
      Eigen::Vector3d(0, 1, -0.6),
      getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle) * 0.9);
}

//==============================================================================
void FeedingDemo::moveTowardsPerson(
    Perception* perception,
    aikido::rviz::WorldInteractiveMarkerViewerPtr viewer)
{
  if (!mAdaReal)
  {
    moveTowardsPerson();
    return;
  }

  std::shared_ptr<aikido::control::TrajectoryExecutor> executor
      = mAda->getTrajectoryExecutor();
  std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> rosExecutor
      = std::dynamic_pointer_cast<aikido::control::ros::RosTrajectoryExecutor>(
          executor);

  if (rosExecutor == nullptr)
  {
    throw std::runtime_error("no ros executor");
  }

  int numDofs = mAda->getArm()->getMetaSkeleton()->getNumDofs();
  Eigen::VectorXd velocityLimits = Eigen::VectorXd::Zero(numDofs);
  for (int i = 0; i < numDofs; i++)
    velocityLimits[i] = 0.2;

  feeding::PerceptionServoClient servoClient(
      mNodeHandle,
      boost::bind(&Perception::perceiveFace, perception, _1),
      mArmSpace,
      mAdaMover.get(),
      mAda->getArm()->getMetaSkeleton(),
      mAda->getHand()->getEndEffectorBodyNode(),
      rosExecutor,
      mCollisionFreeConstraint,
      0.2,
      velocityLimits);
  servoClient.start();
  servoClient.wait(30);

  //   while (perception->isMouthOpen() && ros::ok())
  //   {
  //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //   }
  //   servoClient.stop();
}

//==============================================================================
void FeedingDemo::moveAwayFromPerson()
{
  Eigen::Vector3d targetPosition = mWorkspace->getPersonPose().translation() + Eigen::Vector3d(0, -0.3, 0);
  Eigen::Vector3d forqueTipPosition = mAda->getHand()->getEndEffectorBodyNode()->getTransform().translation();
  Eigen::Vector3d direction = targetPosition - forqueTipPosition;

  bool trajectoryCompleted = mAdaMover->moveToEndEffectorOffset(
      direction.normalized(),
      direction.norm());
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}
};
