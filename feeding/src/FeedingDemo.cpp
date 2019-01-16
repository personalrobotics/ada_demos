#include "feeding/FeedingDemo.hpp"
#include <aikido/rviz/TrajectoryMarker.hpp>
#include <aikido/distance/NominalConfigurationRanker.hpp>

#include <pr_tsr/plate.hpp>
#include <libada/util.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include "feeding/util.hpp"

using ada::util::getRosParam;
using ada::util::createBwMatrixForTSR;
using ada::util::createIsometry;
using aikido::distance::NominalConfigurationRanker;

const bool TERMINATE_AT_USER_PROMPT = true;

static const std::vector<double> weights = {1, 1, 10, 0.01, 0.01, 0.01};

namespace feeding {

//==============================================================================
FeedingDemo::FeedingDemo(
    bool adaReal,
    ros::NodeHandle nodeHandle,
    bool useFTSensingToStopTrajectories,
    std::shared_ptr<FTThresholdHelper> ftThresholdHelper,
    bool autoContinueDemo)
  : mAdaReal(adaReal)
  , mNodeHandle(nodeHandle)
  , mFTThresholdHelper(ftThresholdHelper)
  , mAutoContinueDemo(autoContinueDemo)
  , mIsFTSensingEnabled(useFTSensingToStopTrajectories)
{
  mWorld = std::make_shared<aikido::planner::World>("feeding");

  std::string armTrajectoryExecutor = mIsFTSensingEnabled
                                          ? "move_until_touch_topic_controller"
                                          : "rewd_trajectory_controller";

  mAda = std::make_shared<ada::Ada>(
      mWorld,
      !mAdaReal,
      getRosParam<std::string>("/ada/urdfUri", mNodeHandle),
      getRosParam<std::string>("/ada/srdfUri", mNodeHandle),
      getRosParam<std::string>("/ada/endEffectorName", mNodeHandle),
      armTrajectoryExecutor);
  mArmSpace = mAda->getArm()->getStateSpace();

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
          mWorkspace->getWheelchair().get()
          );
  mCollisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          mArmSpace, mAda->getArm()->getMetaSkeleton(), collisionDetector);
  mCollisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

  // visualization
  mViewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(
      mWorld,
      getRosParam<std::string>("/visualization/topicName", mNodeHandle),
      getRosParam<std::string>("/visualization/baseFrameName", mNodeHandle));
  mViewer->setAutoUpdate(true);

  if (mAdaReal)
  {
    mAda->startTrajectoryExecutor();
  }

  mFoodNames
      = getRosParam<std::vector<std::string>>("/foodItems/names", mNodeHandle);
  mSkeweringForces
      = getRosParam<std::vector<double>>("/foodItems/forces", mNodeHandle);

  for (int i = 0; i < mFoodNames.size(); i++)
    mFoodSkeweringForces[mFoodNames[i]] = mSkeweringForces[i];
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
void FeedingDemo::setPerception(std::shared_ptr<Perception> perception)
{
  mPerception = perception;
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
std::shared_ptr<ada::Ada> FeedingDemo::getAda()
{
  return mAda;
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
void FeedingDemo::grabFoodWithForque()
{
  if (!mWorkspace->getDefaultFoodItem())
  {
    mWorkspace->addDefaultFoodItemAtPose(
        mAda->getHand()->getEndEffectorBodyNode()->getTransform());
  }
  mAda->getHand()->grab(mWorkspace->getDefaultFoodItem());
}

//==============================================================================
void FeedingDemo::ungrabAndDeleteFood()
{
  mAda->getHand()->ungrab();
  mWorkspace->deleteFood();
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
    ROS_INFO_STREAM("Real mode, not moving.");
  }
  else
  {
    std::cout << Eigen::Vector6d(home.data()).transpose() << std::endl;
    mAda->getArm()->getMetaSkeleton()->setPositions(
        Eigen::Vector6d(home.data()));
  }
}

//==============================================================================
void FeedingDemo::moveAboveForque()
{

  double forkHolderAngle
      = getRosParam<double>("/study/forkHolderAngle", mNodeHandle);
  std::vector<double> forkHolderTranslation = getRosParam<std::vector<double>>(
      "/study/forkHolderTranslation", mNodeHandle);

  auto aboveForqueTSR = pr_tsr::getDefaultPlateTSR();
  Eigen::Isometry3d forquePose = Eigen::Isometry3d::Identity();
  // y positive is closer to wheelchair
  // z
  // forquePose.translation() = Eigen::Vector3d{0.57, -0.019, 0.012};
  // forquePose.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(0.15,
  // Eigen::Vector3d::UnitX()));
  forquePose.translation() = Eigen::Vector3d{forkHolderTranslation[0],
                                             forkHolderTranslation[1],
                                             forkHolderTranslation[2]};
  ROS_INFO_STREAM("fork holder angle: " << forkHolderAngle);
  forquePose.linear() = Eigen::Matrix3d(
      Eigen::AngleAxisd(forkHolderAngle, Eigen::Vector3d::UnitX()));
  aboveForqueTSR.mT0_w = forquePose;
  aboveForqueTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};

  aboveForqueTSR.mBw = createBwMatrixForTSR(0.0001, 0.0001, 0, 0);
  aboveForqueTSR.mTw_e.matrix()
      *= mAda->getHand()->getEndEffectorTransform("plate")->matrix();

  if (!mAda->moveArmToTSR(aboveForqueTSR, mCollisionFreeConstraint,
    getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
    getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle)))
    throw std::runtime_error("Trajectory execution failed");
}

//==============================================================================
bool FeedingDemo::moveAbovePlate()
{
  waitForUser("Move forque above plate");

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

  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("plate");
  ROS_INFO_STREAM(
      "move above plate\n" << eeTransform.linear()
                         << mAda->getArm()
                                ->getMetaSkeleton()
                                ->getPositions()
                                .matrix()
                                .transpose());
  eeTransform.linear()
      = eeTransform.linear()
        * Eigen::Matrix3d(
              Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ()));
  abovePlateTSR.mTw_e.matrix() // wrong?
      *= eeTransform.matrix();
  ROS_INFO_STREAM(
      "move above plate 2\n" << eeTransform.linear() << "\n"
                           << mAda->getArm()
                                  ->getMetaSkeleton()
                                  ->getPositions()
                                  .matrix()
                                  .transpose());

  std::vector<double> velocityLimits{0.2, 0.2, 0.2, 0.2, 0.2, 0.4};
  Eigen::VectorXd nominalConfiguration(6);
  nominalConfiguration << -2.00483, 3.26622, 1.8684, -2.38345, 4.11224, 5.03713;

  return mAda->moveArmToTSR(
      abovePlateTSR,
      mCollisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle),
      nominalConfiguration,
      getRanker(nominalConfiguration),
      velocityLimits,
      ada::TrajectoryPostprocessType::KUNZ);
}

//==============================================================================
void FeedingDemo::visualizeTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory)
{
  dart::dynamics::BodyNodePtr endEffector
      = mAda->getMetaSkeleton()->getBodyNode("j2n6s200_forque_end_effector");

  if (!mViewer)
  {
    ROS_WARN("Visualize trajectory: viewer is nullptr");
  }
  else if (!endEffector)
  {
    ROS_WARN("Visualize trajectory: endEffector is nullptr");
  }
  else if (!trajectory)
  {
    ROS_WARN("Visualize trajectory: trajectory is nullptr");
  }
  else
  {
    mViewer->removeTrajectoryMarker(trajectoryMarkerPtr);
    trajectoryMarkerPtr = mViewer->addTrajectoryMarker(
        trajectory,
        mAda->getArm()->getMetaSkeleton(),
        *endEffector,
        Eigen::Vector4d{1, 1, 1, 1},
        0.01,
        16u);
  }
}

//==============================================================================
void FeedingDemo::moveAbovePlateAnywhere()
{
  double heightAbovePlate
      = getRosParam<double>("/feedingDemo/heightAbovePlate", mNodeHandle);
  double horizontalToleranceAbovePlate = getRosParam<double>(
      "/planning/tsr/horizontalToleranceAbovePlate", mNodeHandle);
  double verticalToleranceAbovePlate = getRosParam<double>(
      "/planning/tsr/verticalToleranceAbovePlate", mNodeHandle);

  static std::default_random_engine generator(time(0));
  static std::uniform_real_distribution<double> distribution(-0.08, 0.08);
  static std::uniform_real_distribution<double> distribution2(-M_PI, M_PI);
  double randX = distribution(generator);
  double randY = distribution(generator);
  double angle = distribution2(generator);
  // double randX = 0, randY = 0;

  auto abovePlateTSR = pr_tsr::getDefaultPlateTSR();
  abovePlateTSR.mT0_w
      = mWorkspace->getPlate()->getRootBodyNode()->getWorldTransform();
  abovePlateTSR.mTw_e.translation() = Eigen::Vector3d{randX, randY, 0.07};

  // abovePlateTSR.mBw = createBwMatrixForTSR(0.1, verticalToleranceAbovePlate,
  // -M_PI, M_PI);
  abovePlateTSR.mBw = createBwMatrixForTSR(
      0.005, verticalToleranceAbovePlate, -M_PI * 0.1, M_PI * 0.1);
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("plate");
  eeTransform.linear()
      = eeTransform.linear()
        * Eigen::Matrix3d(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  abovePlateTSR.mTw_e.matrix() *= eeTransform.matrix();

  bool trajectoryCompleted
      = mAda->moveArmToTSR(abovePlateTSR, mCollisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle));

  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
bool FeedingDemo::moveAboveFood(
    const Eigen::Isometry3d& foodTransform,
    int pickupAngleMode,
    float rotAngle,
    float angle,
    bool useAngledTranslation)
{
  ROS_INFO_STREAM("Move above food");

  double heightAboveFood
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle);
  // If the robot is not simulated, we want to plan the trajectory to move a
  // little further downwards,
  // so that the MoveUntilTouchController can take care of stopping the
  // trajectory.
  double horizontalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearFood", mNodeHandle);
  double verticalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearFood", mNodeHandle);

  aikido::constraint::dart::TSR aboveFoodTSR;
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("food");
  if (pickupAngleMode == 0)
  {
    aboveFoodTSR.mT0_w = foodTransform;
    // eeTransform.linear() = eeTransform.linear() *
    // Eigen::Matrix3d(Eigen::AngleAxisd(0.5 * , Eigen::Vector3d::UnitX()));
  }
  else
  {
    Eigen::Isometry3d defaultFoodTransform = Eigen::Isometry3d::Identity();
    defaultFoodTransform.translation() = foodTransform.translation();
    aboveFoodTSR.mT0_w = defaultFoodTransform;

    // celery-style
    // eeTransform.linear() = eeTransform.linear() *
    // Eigen::Matrix3d(Eigen::AngleAxisd( M_PI * 0.5, Eigen::Vector3d::UnitZ())
    // * Eigen::AngleAxisd( M_PI - angle + 0.5, Eigen::Vector3d::UnitX()));

    if (pickupAngleMode == 1)
    {
      // strawberry-style
      eeTransform.linear()
          = eeTransform.linear()
            * Eigen::Matrix3d(
                  Eigen::AngleAxisd(-M_PI * 0.5, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(M_PI + 0.5, Eigen::Vector3d::UnitX()));
    }
    else
    {
      // banana-style
      eeTransform.linear()
          = eeTransform.linear()
            * Eigen::Matrix3d(
                  Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(M_PI + 0.5, Eigen::Vector3d::UnitX()));
    }
  }
  aboveFoodTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearFood, verticalToleranceNearFood, 0, 0);
  aboveFoodTSR.mTw_e.matrix() *= eeTransform.matrix();

  float distance = heightAboveFood * 0.85;

  if (pickupAngleMode == 0)
  {
    // vertical
    aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{-0.01, 0, -distance};
  }
  else if (pickupAngleMode == 1)
  {
    // strawberry style angled
    aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, distance};
  }
  else
  {
    // banana style angled
    aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{
        -sin(M_PI * 0.25) * distance, 0, cos(M_PI * 0.25) * distance};
  }

  bool trajectoryCompleted = false;
  try
  {
    trajectoryCompleted
        = mAda->moveArmToTSR(aboveFoodTSR, mCollisionFreeConstraint,
        getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
        getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle));
  }
  catch (...)
  {
    ROS_WARN("Error in trajectory completion!");
    trajectoryCompleted = false;
  }

  return trajectoryCompleted;
}

//==============================================================================
void FeedingDemo::moveNextToFood(
    const Eigen::Isometry3d& foodTransform,
    float angle,
    bool useAngledTranslation)
{
  waitForUser("Move forque next to food");

  double heightAboveFood
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle);
  // If the robot is not simulated, we want to plan the trajectory to move a
  // little further downwards,
  // so that the MoveUntilTouchController can take care of stopping the
  // trajectory.
  double horizontalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearFood", mNodeHandle);
  double verticalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearFood", mNodeHandle);

  double distBeforePush
      = getRosParam<double>("/feedingDemo/distBeforePush", mNodeHandle);

  aikido::constraint::dart::TSR nextToFoodTSR;
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("food");

  nextToFoodTSR.mT0_w = foodTransform;
  ROS_INFO_STREAM(eeTransform.linear());
  eeTransform.linear()
      = eeTransform.linear()
        * Eigen::Matrix3d(
              Eigen::AngleAxisd(
                  /*(-M_PI * 0.5) -*/ -angle, Eigen::Vector3d::UnitZ()));
  ROS_INFO_STREAM(eeTransform.linear());

  nextToFoodTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearFood, verticalToleranceNearFood, 0, 0);
  nextToFoodTSR.mTw_e.matrix() *= eeTransform.matrix();

  float distance = heightAboveFood * 0.85;

  float xOff = cos(angle - M_PI * 0.5) * distBeforePush;
  float yOff = sin(angle - M_PI * 0.5) * distBeforePush;
  nextToFoodTSR.mTw_e.translation() = Eigen::Vector3d{-xOff, yOff, 0.01};

  bool trajectoryCompleted
      = mAda->moveArmToTSR(nextToFoodTSR, mCollisionFreeConstraint,
        getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
        getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle));
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveNextToFood(
    Perception* perception, float angle, Eigen::Isometry3d forqueTransform)
{
  waitForUser("Move forque next to food");

  double prePushOffset
      = getRosParam<double>("/feedingDemo/prePushOffset", mNodeHandle);

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

  int numDofs = mAda->getArm()->getMetaSkeleton()->getNumDofs();
  Eigen::VectorXd velocityLimits = Eigen::VectorXd::Zero(numDofs);
  for (int i = 0; i < numDofs; i++)
    velocityLimits[i] = 0.2;

  PerceptionPreProcess offsetApplier(
      boost::bind(&Perception::perceiveFood, perception, _1),
      angle,
      prePushOffset,
      forqueTransform);

  PerceptionServoClient servoClient(
      mNodeHandle,
      boost::bind(&PerceptionPreProcess::applyOffset, &offsetApplier, _1),
      mArmSpace,
      mAda,
      mAda->getArm()->getMetaSkeleton(),
      mAda->getHand()->getEndEffectorBodyNode(),
      rosExecutor,
      mCollisionFreeConstraint,
      0.1,
      velocityLimits,
      0.1,
      0.002);
  servoClient.start();

  servoClient.wait(10000.0);
}

//==============================================================================
void FeedingDemo::rotateForque(
    const Eigen::Isometry3d& foodTransform,
    float angle,
    int pickupAngleMode,
    bool useAngledTranslation)
{
  waitForUser(
      "Rotate forque to angle " + std::to_string(angle) + " to push food");
  double heightAboveFood
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle);
  // If the robot is not simulated, we want to plan the trajectory to move a
  // little further downwards,
  // so that the MoveUntilTouchController can take care of stopping the
  // trajectory.
  double horizontalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearFood", mNodeHandle);
  double verticalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearFood", mNodeHandle);
  double distBeforePush
      = getRosParam<double>("/feedingDemo/distBeforePush", mNodeHandle);

  aikido::constraint::dart::TSR nextToFoodTSR;
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("food");

  // Eigen::Isometry3d defaultFoodTransform = Eigen::Isometry3d::Identity();
  // defaultFoodTransform.translation() = foodTransform.translation();
  nextToFoodTSR.mT0_w = foodTransform; // defaultFoodTransform;
  if (pickupAngleMode != 0)
  {
    Eigen::Isometry3d defaultFoodTransform = Eigen::Isometry3d::Identity();
    defaultFoodTransform.translation() = foodTransform.translation();
    nextToFoodTSR.mT0_w = defaultFoodTransform;
  }

  Eigen::AngleAxisd rotation = /*Eigen::Matrix3d(*/ Eigen::AngleAxisd(
      -angle, Eigen::Vector3d::UnitZ()) /*)*/;
  if (pickupAngleMode == 0)
  {
    eeTransform.linear() = eeTransform.linear() * Eigen::Matrix3d(rotation);
  }
  else if (pickupAngleMode == 1)
  {
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                rotation
                * Eigen::AngleAxisd(
                      M_PI + 0.5, rotation * Eigen::Vector3d::UnitX()));
    // Eigen::Matrix3d(Eigen::AngleAxisd( -M_PI * 0.5, Eigen::Vector3d::UnitZ())
    // * Eigen::AngleAxisd( M_PI + 0.5, Eigen::Vector3d::UnitX()) *
    // Eigen::AngleAxisd((M_PI * 0.5) -angle, Eigen::Vector3d::UnitZ()));
  }
  else
  {
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                rotation
                * Eigen::AngleAxisd(
                      M_PI - angle + 0.5,
                      rotation
                          * Eigen::Vector3d::
                                UnitX())); // Eigen::Matrix3d(Eigen::AngleAxisd(
                                           // M_PI * 0.5,
                                           // Eigen::Vector3d::UnitZ()) *
                                           // Eigen::AngleAxisd( M_PI - angle +
                                           // 0.5, Eigen::Vector3d::UnitX()) *
                                           // Eigen::AngleAxisd((-M_PI * 0.5)
                                           // -angle,
                                           // Eigen::Vector3d::UnitZ()));

    // eeTransform.linear() = eeTransform.linear() *
    // Eigen::Matrix3d(Eigen::AngleAxisd( M_PI * 0.5, Eigen::Vector3d::UnitZ(),
    // Eigen::AngleAxisd( /*(-M_PI * 0.5) -*/ -angle, Eigen::Vector3d::UnitZ())
    // * Eigen::AngleAxisd( M_PI - angle + 0.5, Eigen::Vector3d::UnitX()));
  }

  nextToFoodTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearFood, verticalToleranceNearFood, 0, 0);
  nextToFoodTSR.mTw_e.matrix() *= eeTransform.matrix();

  float distance = heightAboveFood * 0.85;

  float xOff = cos(angle) * distBeforePush;
  float yOff = sin(angle) * distBeforePush;
  if (pickupAngleMode == 0)
  {
    nextToFoodTSR.mTw_e.translation()
        = /*rotation **/ Eigen::Vector3d{-0.01, 0, -distance};
  }
  else if (pickupAngleMode == 1)
  {
    nextToFoodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, distance};
  }
  else
  {
    nextToFoodTSR.mTw_e.translation()
        = rotation * Eigen::Vector3d{-sin(M_PI * 0.25) * distance,
                                     0,
                                     cos(M_PI * 0.25) * distance};
  }
  // nextToFoodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, -distance};

  bool trajectoryCompleted
      = mAda->moveArmToTSR(nextToFoodTSR, mCollisionFreeConstraint,
        getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
        getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle));
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveOutOf(TargetItem item)
{
  if (item != FOOD && item != PLATE && item != FORQUE)
    throw std::invalid_argument(
        "MoveOutOf[" + TargetToString.at(item) + "] not supported");

  waitForUser("Move Out of " + TargetToString.at(item));
  setFTThreshold(AFTER_GRAB_FOOD_FT_THRESHOLD);

  double length;
  Eigen::Vector3d direction(0, 0, 1);

  if (item == PLATE)
    length = getRosParam<double>("/feedingDemo/heightOutOfPlate", mNodeHandle);
  else if (item == FOOD)
    length = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle)
             * 0.75;
  else
  {
    length = 0.04;
    direction = Eigen::Vector3d(0, -1, 0);
  }

  bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
      direction, length, nullptr, //mCollisionFreeConstraint,
    getRosParam<int>("/planning/timeoutSeconds", mNodeHandle),
    getRosParam<double>("/planning/endEffectorOffset/positionTolerance", mNodeHandle),
    getRosParam<double>("/planning/endEffectorOffset/angularTolerance", mNodeHandle));

  setFTThreshold(STANDARD_FT_THRESHOLD);
  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
}

//==============================================================================
bool FeedingDemo::moveInto(TargetItem item)
{
  if (item != FOOD && item != FORQUE)
    throw std::invalid_argument(
        "MoveInto[" + TargetToString.at(item) + "] not supported");

  if (item == TargetItem::FORQUE)
    return mAda->moveArmToEndEffectorOffset(
        Eigen::Vector3d(0, 1, 0), 0.032, mCollisionFreeConstraint,
    getRosParam<int>("/planning/timeoutSeconds", mNodeHandle),
    getRosParam<double>("/planning/endEffectorOffset/positionTolerance", mNodeHandle),
    getRosParam<double>("/planning/endEffectorOffset/angularTolerance", mNodeHandle)
        );

  if (mAdaReal && mPerception)
  {
    ROS_INFO("Servoing into food");
    std::shared_ptr<aikido::control::TrajectoryExecutor> executor
        = mAda->getTrajectoryExecutor();

    std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> rosExecutor
        = std::
            dynamic_pointer_cast<aikido::control::ros::RosTrajectoryExecutor>(
                executor);

    if (rosExecutor == nullptr)
    {
      throw std::runtime_error("no ros executor");
    }

    int numDofs = mAda->getArm()->getMetaSkeleton()->getNumDofs();
    Eigen::VectorXd velocityLimits = Eigen::VectorXd::Ones(numDofs) * 0.2;

    PerceptionServoClient servoClient(
        mNodeHandle,
        boost::bind(&Perception::perceiveFood, mPerception.get(), _1),
        mArmSpace,
        mAda,
        mAda->getArm()->getMetaSkeleton(),
        mAda->getHand()->getEndEffectorBodyNode(),
        rosExecutor,
        mCollisionFreeConstraint,
        0.1,
        velocityLimits,
        0.1,
        0.002);
    servoClient.start();

    return servoClient.wait(10000.0);
  }

  auto length
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle)
        + getRosParam<double>("/feedingDemo/heightIntoFood", mNodeHandle);

  return mAda->moveArmToEndEffectorOffset(
      Eigen::Vector3d(0, 0, -1), length, nullptr,// mCollisionFreeConstraint,
      getRosParam<int>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<double>("/planning/endEffectorOffset/positionTolerance", mNodeHandle),
      getRosParam<double>("/planning/endEffectorOffset/angularTolerance", mNodeHandle));
}

//==============================================================================
void FeedingDemo::pushFood(
    float angle, Eigen::Isometry3d* forqueTransform, bool useAngledTranslation)
{
  waitForUser("Push Food");
  setFTThreshold(PUSH_FOOD_FT_THRESHOLD);

  double distToPush
      = getRosParam<double>("/feedingDemo/distToPush", mNodeHandle);

  double pushVelLimit
      = getRosParam<double>("/feedingDemo/pushVelLimit", mNodeHandle);

  std::vector<double> velocityLimits{pushVelLimit,
                                     pushVelLimit,
                                     pushVelLimit,
                                     pushVelLimit,
                                     pushVelLimit,
                                     pushVelLimit};

  Eigen::Vector3d endEffectorDirection;

  if (!forqueTransform)
  {
    double distAfterPush
        = getRosParam<double>("/feedingDemo/distAfterPush", mNodeHandle);

    float xOff = cos(angle - M_PI * 0.5) * distAfterPush;
    float yOff = sin(angle - M_PI * 0.5) * distAfterPush;

    if (distToPush < 0)
    {
      xOff *= -1;
      yOff *= -1;
      distToPush *= -1;
    }

    endEffectorDirection = Eigen::Vector3d(-xOff, -yOff, 0);
  }
  else
  {
    // Positive y in forque frame is forward
    Eigen::Vector3d diff(0, 1, 0);

    if (distToPush < 0)
    {
      distToPush *= -1;
      diff = Eigen::Vector3d(0, -1, 0);
    }
    endEffectorDirection = forqueTransform->inverse().linear() * diff;
  }

  bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
      endEffectorDirection,
      distToPush,
      mCollisionFreeConstraint,
      getRosParam<int>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<double>("/planning/endEffectorOffset/positionTolerance", mNodeHandle),
      getRosParam<double>("/planning/endEffectorOffset/angularTolerance", mNodeHandle),
      velocityLimits);
  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
}

//==============================================================================
bool FeedingDemo::moveInFrontOfPerson()
{
  ROS_INFO_STREAM("move in front of person");

  double distanceToPerson
      = getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle);
  double horizontalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearPerson", mNodeHandle);
  double verticalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearPerson", mNodeHandle);

  aikido::constraint::dart::TSR personTSR;
  Eigen::Isometry3d personPose = Eigen::Isometry3d::Identity();
  personPose.translation() = mWorkspace->getPersonPose()
                                 .translation(); // + Eigen::Vector3d(0,0.2,0);
  personPose.linear()
      = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  personTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearPerson, verticalToleranceNearPerson, 0, 0);
  personTSR.mTw_e.matrix()
      *= mAda->getHand()->getEndEffectorTransform("person")->matrix();

  std::vector<double> velocityLimits{0.2, 0.2, 0.2, 0.2, 0.2, 0.4};
  auto trajectory = mAda->planArmToTSR(
    personTSR,
    mCollisionFreeConstraint,
    getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
    getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle),
    Eigen::VectorXd(0),
    getRanker()
    );

  if (!trajectory)
    return false;

  // visualizeTrajectory(trajectory);
  return mAda->moveArmOnTrajectory(
      trajectory,
      mCollisionFreeConstraint,
      ada::TrajectoryPostprocessType::KUNZ,
      velocityLimits);
}

//==============================================================================
bool FeedingDemo::tiltUpInFrontOfPerson()
{
  waitForUser("tiltUp in front of person");

  Eigen::Vector3d workingPersonTranslation(0.263, 0.269386, 0.652674);
  std::vector<double> tiltOffsetVector
      = getRosParam<std::vector<double>>("/study/personPose", mNodeHandle);
  Eigen::Vector3d tiltOffset{
      tiltOffsetVector[0], tiltOffsetVector[1], tiltOffsetVector[2]};
  Eigen::Vector3d personTranslation
      = mAda->getHand()->getEndEffectorBodyNode()->getTransform().translation()
        + tiltOffset;
  Eigen::Vector3d correctionTranslation
      = workingPersonTranslation - personTranslation;

  auto ranker = getRanker();

  for (double i = 0; i <= 1.0; i += 0.2)
  {
    aikido::constraint::dart::TSR personTSR;
    Eigen::Isometry3d personPose = Eigen::Isometry3d::Identity();
    personPose.translation() = personTranslation + correctionTranslation * i;
    ROS_INFO_STREAM(
        "personTranslation: " << personPose.translation().matrix().transpose());
    personPose.linear()
        = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    personTSR.mT0_w = personPose;
    personTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};

    personTSR.mBw = createBwMatrixForTSR(0.02, 0.02, -M_PI / 4, M_PI / 4);
    Eigen::Isometry3d eeTransform
        = *mAda->getHand()->getEndEffectorTransform("person");
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                Eigen::AngleAxisd(M_PI * -0.25, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(M_PI * 0.25, Eigen::Vector3d::UnitX()));
    personTSR.mTw_e.matrix() *= eeTransform.matrix();

    bool trajectoryCompleted = false;
    try
    {

      auto trajectory = mAda->planArmToTSR(personTSR, mCollisionFreeConstraint,
        getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
        getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle),
        Eigen::VectorXd(0),
        ranker);
      if (!trajectory)
      {
        ROS_INFO_STREAM("PlanToTSR failed.");
        continue;
      }

      // visualizeTrajectory(trajectory);
      bool trajectoryCompleted = mAda->moveArmOnTrajectory(
          trajectory,
          mCollisionFreeConstraint,
          ada::TrajectoryPostprocessType::KUNZ);

      if (trajectoryCompleted)
      {
        ROS_INFO_STREAM("tiltUp in font of person complete");
        return trajectoryCompleted;
      }else
      {
        ROS_INFO_STREAM("tiltUp in font of person failed");
      }
    }
    catch (std::runtime_error e)
    {
      ROS_WARN("tilt up trajectory failed!");
      continue;
    }
  }
  return false;
}

//==============================================================================
void FeedingDemo::tiltDownInFrontOfPerson()
{
  printRobotConfiguration();
  Eigen::Vector3d workingPersonTranslation(0.269274, 0.191136, 0.71243);
  Eigen::Vector3d personTranslation;
  personTranslation
      = mAda->getHand()->getEndEffectorBodyNode()->getTransform().translation()
        + Eigen::Vector3d{0.01, 0, 0.06} + Eigen::Vector3d{0, 0, 0.0};
  Eigen::Vector3d correctionTranslation
      = workingPersonTranslation - personTranslation;

  auto ranker = getRanker();

  for (double i = 0; i <= 1.0; i += 0.2)
  {
    aikido::constraint::dart::TSR personTSR;
    Eigen::Isometry3d personPose = Eigen::Isometry3d::Identity();
    personPose.translation() = personTranslation + correctionTranslation * i;
    ROS_INFO_STREAM(
        "personTranslation: " << personPose.translation().matrix().transpose());
    personPose.linear()
        = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    personTSR.mT0_w = personPose;
    personTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};

    personTSR.mBw = createBwMatrixForTSR(0.02, 0.02, -M_PI / 4, M_PI / 4);
    Eigen::Isometry3d eeTransform
        = *mAda->getHand()->getEndEffectorTransform("person");
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                Eigen::AngleAxisd(-M_PI * 0.25, Eigen::Vector3d::UnitX()));
    personTSR.mTw_e.matrix() *= eeTransform.matrix();

    bool trajectoryCompleted = false;
    try
    {
      auto trajectory = mAda->planArmToTSR(personTSR,
          mCollisionFreeConstraint,
          getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
          getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle),
          Eigen::VectorXd(0),
          ranker);
      visualizeTrajectory(trajectory);
      bool trajectoryCompleted = mAda->moveArmOnTrajectory(
          trajectory,
          mCollisionFreeConstraint,
          ada::TrajectoryPostprocessType::KUNZ);

    }
    catch (std::runtime_error e)
    {
      ROS_WARN("tilt down trajectory failed!");
      continue;
    }
    if (trajectoryCompleted)
    {
      ROS_INFO_STREAM("tilt down in font of person complete");
      return;
    }
    else
    {
      ROS_WARN("aborting tilt down!");
      return;
    }
  }
}
//==============================================================================
void FeedingDemo::moveDirectlyToPerson(bool tilted)
{
  double distanceToPerson = 0.02;
  double horizontalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearPerson", mNodeHandle);
  double verticalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearPerson", mNodeHandle);

  Eigen::Isometry3d personPose = createIsometry(
      getRosParam<std::vector<double>>("/study/personPose", mNodeHandle));
  if (tilted)
  {
    std::vector<double> tiltOffsetVector
        = getRosParam<std::vector<double>>("/study/tiltOffset", mNodeHandle);
    Eigen::Vector3d tiltOffset{
        tiltOffsetVector[0], tiltOffsetVector[1], tiltOffsetVector[2]};
    personPose.translation() += tiltOffset;
  }

  aikido::constraint::dart::TSR personTSR;
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  if (tilted)
  {
    personTSR.mBw = createBwMatrixForTSR(
        horizontalToleranceNearPerson,
        verticalToleranceNearPerson,
        -M_PI / 4,
        M_PI / 4);
    Eigen::Isometry3d eeTransform
        = *mAda->getHand()->getEndEffectorTransform("person");
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                Eigen::AngleAxisd(M_PI * -0.25, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(M_PI * 0.25, Eigen::Vector3d::UnitX()));
    personTSR.mTw_e.matrix() *= eeTransform.matrix();
  }
  else
  {
    personTSR.mBw = createBwMatrixForTSR(
        horizontalToleranceNearPerson, verticalToleranceNearPerson, 0, 0);
    personTSR.mTw_e.matrix()
        *= mAda->getHand()->getEndEffectorTransform("person")->matrix();
  }

  auto ranker = getRanker();

  std::vector<double> velocityLimits{0.2, 0.2, 0.2, 0.2, 0.2, 0.4};
  auto trajectory = mAda->planArmToTSR(
    personTSR, mCollisionFreeConstraint,
    getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
    getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle),
    Eigen::VectorXd(0),
    ranker);
  visualizeTrajectory(trajectory);
  bool trajectoryCompleted = mAda->moveArmOnTrajectory(
      trajectory,
      mCollisionFreeConstraint,
      ada::TrajectoryPostprocessType::KUNZ,
      velocityLimits);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
bool FeedingDemo::moveTowardsPerson()
{
  if (!mAdaReal)
  {
    return mAda->moveArmToEndEffectorOffset(
        Eigen::Vector3d(0, 1, -0.6),
        getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle) * 0.9,
        mCollisionFreeConstraint,
        getRosParam<int>("/planning/timeoutSeconds", mNodeHandle),
        getRosParam<double>("/planning/endEffectorOffset/positionTolerance", mNodeHandle),
        getRosParam<double>("/planning/endEffectorOffset/angularTolerance", mNodeHandle));
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
      boost::bind(&Perception::perceiveFace, mPerception.get(), _1),
      mArmSpace,
      mAda,
      mAda->getArm()->getMetaSkeleton(),
      mAda->getHand()->getEndEffectorBodyNode(),
      rosExecutor,
      mCollisionFreeConstraint,
      0.2,
      velocityLimits,
      0,
      0.06);
  servoClient.start();
  return servoClient.wait(30);
}

//==============================================================================
void FeedingDemo::moveAwayFromPerson()
{
  Eigen::Vector3d targetPosition
      = mWorkspace->getPersonPose().translation() + Eigen::Vector3d(0, -0.2, 0);
  Eigen::Vector3d forqueTipPosition
      = mAda->getHand()->getEndEffectorBodyNode()->getTransform().translation();
  Eigen::Vector3d direction = targetPosition - forqueTipPosition;

  bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
      direction.normalized(),
      direction.norm(),
      mCollisionFreeConstraint,
      getRosParam<int>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<double>("/planning/endEffectorOffset/positionTolerance", mNodeHandle),
      getRosParam<double>("/planning/endEffectorOffset/angularTolerance", mNodeHandle));

  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
aikido::rviz::WorldInteractiveMarkerViewerPtr FeedingDemo::getViewer()
{
  return mViewer;
}

//==============================================================================
void FeedingDemo::pickUpFork()
{
  mAda->openHand();

  waitForUser("Forque?");
  moveAboveForque();

  waitForUser("In?");
  moveInto(TargetItem::FORQUE);

  if (ada::util::waitForUser("Close?"))
  {
    mAda->closeHand();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  waitForUser("Out?");
  moveOutOf(TargetItem::FORQUE);

  waitForUser("Above Plate?");
  moveAbovePlate();
}

//==============================================================================
void FeedingDemo::putDownFork()
{
  mAda->closeHand();

  waitForUser("Forque?");
  moveAboveForque();

  waitForUser("In?");
  moveInto(TargetItem::FORQUE);

  mAda->openHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  waitForUser("Out?");
  moveOutOf(TargetItem::FORQUE);

  waitForUser("Above Plate?");
  moveAbovePlate();
}

//==============================================================================
void FeedingDemo::skewer(
    std::string foodName, ros::NodeHandle& nodeHandle, int max_trial_per_item)
{
  bool foodPickedUp = false;
  int num_tries = 0;

  if (foodName == "")
    foodName = getUserInput(true, nodeHandle);

  while (!foodPickedUp)
  {
    // ===== ABOVE PLATE =====
    waitForUser("Move forque above plate");
    moveAbovePlate();

    // ===== PERCEPTION =====
    auto foodTransform = detectFood(foodName, true);

    ROS_INFO_STREAM(
        "Alright! Let's get the " << foodName
                                  << "!\033[0;32m  (Gonna skewer with "
                                  << mFoodSkeweringForces[foodName]
                                  << "N)\033[0m"
                                  << std::endl);

    // ===== ABOVE FOOD =====
    // 0 vertical
    // 1 strawberry-style
    // 2 banana-style
    int pickupAngleMode
        = getRosParam<int>("/study/pickupAngleMode", nodeHandle);

    waitForUser("Move forque above food");

    moveAboveFood(foodTransform.get(), pickupAngleMode, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    foodTransform = detectFood(foodName, false);

    moveAboveFood(foodTransform.get(), pickupAngleMode, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ===== INTO FOOD =====
    waitForUser("Move forque into food");

    double zForceBeforeSkewering = 0;
    if (mFTThresholdHelper->startDataCollection(50))
    {
      Eigen::Vector3d currentForce, currentTorque;
      while (!mFTThresholdHelper->isDataCollectionFinished(
          currentForce, currentTorque))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      zForceBeforeSkewering = currentForce.x();
    }
    double torqueThreshold = 2;

    if (!mFTThresholdHelper->setThresholds(
            mFoodSkeweringForces[foodName], torqueThreshold))
      exit(1);

    moveInto(TargetItem::FOOD);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            getRosParam<int>("/feedingDemo/waitMillisecsAtFood", nodeHandle)));
    grabFoodWithForque();

    // ===== OUT OF FOOD =====
    waitForUser("Move forque out of food");

    moveOutOf(TargetItem::FOOD);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    double forceDifference = 100;
    double zForceAfter = 0;
    if (mFTThresholdHelper->startDataCollection(50))
    {
      Eigen::Vector3d currentForce, currentTorque;
      while (!mFTThresholdHelper->isDataCollectionFinished(
          currentForce, currentTorque))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      zForceAfter = currentForce.x();
      forceDifference = zForceBeforeSkewering - currentForce.x();
    }
    ROS_WARN_STREAM(
        "force difference: " << (zForceBeforeSkewering - zForceAfter));

    if (forceDifference > 0.02)
      return;
    else
    {
      ungrabAndDeleteFood();
      if (num_tries >= max_trial_per_item)
      {
        ROS_WARN_STREAM(
            "Ooops! I think I didn't manage to pick up the "
            << foodName
            << ".  Maybe we should try a different food item."
            << std::endl);

        foodName = getUserInput(true, nodeHandle);
        num_tries = 0;
      }
      else
      {
        ROS_WARN_STREAM(
            "Ooops! I think I didn't manage to pick up the "
            << foodName
            << ". Let me try one more time."
            << std::endl);

        num_tries++;
      }
    }
  }
}

//==============================================================================
void FeedingDemo::feedFoodToPerson(ros::NodeHandle& nodeHandle, bool tilted)
{
  nodeHandle.setParam("/feeding/facePerceptionOn", true);
  moveInFrontOfPerson();

  ROS_INFO_STREAM("Move towards person");
  moveTowardsPerson();

  if (tilted)
    tiltUpInFrontOfPerson();

  // ===== EATING =====
  ROS_WARN("Human is eating");
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          getRosParam<int>("/feedingDemo/waitMillisecsAtPerson", nodeHandle)));
  ungrabAndDeleteFood();

  waitForUser("Move away from person 1");
  waitForUser("Move away from person 2");

  if (!tilted)
    moveAwayFromPerson();

  // ===== BACK TO PLATE =====
  waitForUser("Move back to plate");

  moveAbovePlate();
}

//==============================================================================
boost::optional<Eigen::Isometry3d> FeedingDemo::detectFood(
    const std::string& foodName, bool waitTillDetected)
{
  if (!mAdaReal)
    return getDefaultFoodTransform();

  if (!mPerception->setFoodName(foodName))
  {
    ROS_WARN_STREAM(
        "I don't know about any food that's called '"
        << foodName
        << ". Wanna get something else?");
    return boost::optional<Eigen::Isometry3d>{};
  }

  Eigen::Isometry3d foodTransform;

  if (waitTillDetected)
  {
    while (true)
    {
      if (!mPerception->perceiveFood(foodTransform, true, mViewer))
      {
        ROS_WARN_STREAM("I can't see the " << foodName << std::endl);
        waitForUser("Try perception again?");
        continue;
      }
      return foodTransform;
    }
  }
  else
  {
    if (!mPerception->perceiveFood(foodTransform, true, mViewer))
    {
      ROS_WARN_STREAM("I can't see the " << foodName << std::endl);
      return boost::optional<Eigen::Isometry3d>{};
    }
    return foodTransform;
  }
}

//==============================================================================
void FeedingDemo::setFTThreshold(FTThreshold threshold)
{
  if (mAdaReal && mIsFTSensingEnabled)
    if (!mFTThresholdHelper->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD))
      exit(1);
}

//==============================================================================
void FeedingDemo::waitForUser(const std::string& prompt)
{
  if (!mAutoContinueDemo)
    ada::util::waitForUser(prompt, TERMINATE_AT_USER_PROMPT);
}

//==============================================================================
Eigen::Isometry3d FeedingDemo::detectAndMoveAboveFood(
    const std::string& foodName,
    int pickupAngleMode,
    float rotAngle,
    float angle,
    bool useAngledTranslation)
{
  while (true)
  {
    auto foodTransform = detectFood(foodName, true);
    assert(foodTransform);

    if (!moveAboveFood(
            foodTransform.get(),
            pickupAngleMode,
            rotAngle,
            angle,
            useAngledTranslation))
    {
      waitForUser("Trajectory failed! Reposition food and try again!");
      continue;
    }
    return foodTransform.get();
  }
}

//==============================================================================
void FeedingDemo::pushAndSkewer(
    const std::string& foodName,
    int pickupAngleMode,
    float rotAngle,
    float tiltAngle)
{
  // ===== ABOVE FOOD =====
  // TODO: check pickUpAngleMode
  auto foodTransform = detectAndMoveAboveFood(foodName, 0, rotAngle, 0, false);

  // Retry until success
  while (true)
  {
    // ===== ROTATE FORQUE ====
    rotateForque(foodTransform, rotAngle, 0);

    // ===== INTO TO FOOD ====
    double torqueThreshold = 2;
    setFTThreshold(STANDARD_FT_THRESHOLD);

    Eigen::Isometry3d forqueTransform;
    if (mAdaReal)
    {
      forqueTransform = mPerception->getForqueTransform();
      moveNextToFood(mPerception.get(), rotAngle, forqueTransform);
    }
    else
    {
      moveNextToFood(foodTransform, rotAngle);
    }

    // ===== MOVE OUT OF PLATE ====
    moveOutOf(TargetItem::PLATE); // GL: why is this necessary?

    // keep pushing until user says no, get feedback on how far to move
    // ===== PUSH FOOD ====
    if (mAdaReal)
    {
      pushFood(rotAngle, &forqueTransform);
    }
    else
    {
      pushFood(rotAngle);
    }
    break;
  }

  // ===== OUT OF FOOD =====
  setFTThreshold(AFTER_GRAB_FOOD_FT_THRESHOLD);
  moveOutOf(TargetItem::FOOD);

  // ===== MOVE BACK ABOVE PLATE =====
  moveAbovePlate();
}

//==============================================================================
void FeedingDemo::rotateAndSkewer(
    const std::string& foodName, float rotateForqueAngle)
{
  // TODO: check pickUpAngleMode
  auto foodTransform
      = detectAndMoveAboveFood(foodName, 0, rotateForqueAngle, 0, false);

  // Retry until success
  while (true)
  {
    // ===== ROTATE FORQUE ====
    rotateForque(foodTransform, rotateForqueAngle, 0);

    // ===== INTO TO FOOD ====
    moveInto(TargetItem::FOOD);

    // ===== OUT OF FOOD =====
    setFTThreshold(AFTER_GRAB_FOOD_FT_THRESHOLD);
    moveOutOf(TargetItem::FOOD);
    break;
  }
}

//==============================================================================
void FeedingDemo::reset()
{
  mWorkspace->reset();
}

//==============================================================================
aikido::distance::ConfigurationRankerPtr FeedingDemo::getRanker(
  const Eigen::VectorXd& configuration)
{
  auto metaSkeleton = mAda->getArm()->getMetaSkeleton();
  auto nominalState = mArmSpace->createState();

  if (configuration.size() > 0)
    mArmSpace->convertPositionsToState(configuration, nominalState);
  else
    nominalState = mArmSpace->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  return std::make_shared<NominalConfigurationRanker>(
    mArmSpace,
    metaSkeleton,
    weights,
    nominalState);
}
} // namespace feeding
