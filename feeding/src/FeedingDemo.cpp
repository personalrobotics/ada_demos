#include "feeding/FeedingDemo.hpp"
#include <aikido/rviz/TrajectoryMarker.hpp>
#include <pr_tsr/plate.hpp>
#include <libada/util.hpp>

#include "feeding/util.hpp"

using ada::util::getRosParam;
using ada::util::waitForUser;
using ada::util::createBwMatrixForTSR;
using ada::util::createIsometry;

const bool TERMINATE_AT_USER_PROMPT = true;

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

  mAda = std::make_shared<ada::Ada>(
      mWorld,
      !mAdaReal,
      getRosParam<std::string>("/ada/urdfUri", mNodeHandle),
      getRosParam<std::string>("/ada/srdfUri", mNodeHandle),
      getRosParam<std::string>("/ada/endEffectorName", mNodeHandle),
      armTrajectoryExecutor);
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
          mWorkspace->getWorkspaceEnvironment().get()
          // mWorkspace->getWheelchair().get()
          );
  mCollisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          mArmSpace, mAda->getArm()->getMetaSkeleton(), collisionDetector);
  mCollisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

  // visualization
  mViewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(
      mWorld,
      getRosParam<std::string>("/visualization/topicName", nodeHandle),
      getRosParam<std::string>("/visualization/baseFrameName", nodeHandle));
  mViewer->setAutoUpdate(true);

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
  }
  else
  {
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

  bool trajectoryCompleted
      = mAda->moveArmToTSR(aboveForqueTSR, mCollisionFreeConstraint);

  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveIntoForque()
{
  bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
      Eigen::Vector3d(0, 1, 0), 0.032, mCollisionFreeConstraint);
  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
}

//==============================================================================
void FeedingDemo::moveOutOfForque()
{
  bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
      Eigen::Vector3d(0, -1, 0), 0.04, mCollisionFreeConstraint);
  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
}

//==============================================================================
bool FeedingDemo::moveAbovePlate()
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
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("plate");
  eeTransform.linear()
      = eeTransform.linear()
        * Eigen::Matrix3d(
              Eigen::AngleAxisd(M_PI * 0.25, Eigen::Vector3d::UnitZ()));
  abovePlateTSR.mTw_e.matrix() *= eeTransform.matrix();

  std::vector<double> velocityLimits{0.2, 0.2, 0.2, 0.2, 0.2, 0.4};
  Eigen::VectorXd nominalConfiguration(6);
  nominalConfiguration << -2.00483, 3.26622, 1.8684, -2.38345, 4.11224, 5.03713;

  auto trajectory = mAda->planArmToTSR(
      abovePlateTSR, mCollisionFreeConstraint, nominalConfiguration);

  visualizeTrajectory(trajectory);
  return mAda->moveArmOnTrajectory(
      trajectory,
      mCollisionFreeConstraint,
      ada::TrajectoryPostprocessType::KUNZ,
      velocityLimits);
}

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
    trajectoryMarkerPtr = nullptr;
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
      = mAda->moveArmToTSR(abovePlateTSR, mCollisionFreeConstraint);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveAboveFood(
    const Eigen::Isometry3d& foodTransform,
    int pickupAngleMode,
    bool useAngledTranslation)
{
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

  // tsrMarkers.push_back(viewer->addTSRMarker(aboveFoodTSR, 100,
  // "someTSRName"));
  // std::this_thread::sleep_for(std::chrono::milliseconds(20000));

  auto trajectory = mAda->planArmToTSR(aboveFoodTSR, mCollisionFreeConstraint);
  visualizeTrajectory(trajectory);
  bool trajectoryCompleted = mAda->moveArmOnTrajectory(
      trajectory,
      mCollisionFreeConstraint,
      ada::TrajectoryPostprocessType::KUNZ);

  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
bool FeedingDemo::moveIntoFood()
{
  auto length
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle)
        + getRosParam<double>("/feedingDemo/heightIntoFood", mNodeHandle);

  bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
      Eigen::Vector3d(0, 0, -1), length, mCollisionFreeConstraint);

  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
  return true;
}

//==============================================================================
bool FeedingDemo::moveIntoFood(Perception* perception)
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

  int numDofs = mAda->getArm()->getMetaSkeleton()->getNumDofs();
  Eigen::VectorXd velocityLimits = Eigen::VectorXd::Ones(numDofs) * 0.2;

  PerceptionServoClient servoClient(
      mNodeHandle,
      boost::bind(&Perception::perceiveFood, perception, _1),
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

//==============================================================================
void FeedingDemo::moveOutOfFood()
{
  auto length
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle) * 0.75;

  bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
      Eigen::Vector3d(0, 0, 1), length, nullptr);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveOutOfFood(float dist)
{
  bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
      Eigen::Vector3d(0, 0, 1), dist, nullptr);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
bool FeedingDemo::moveInFrontOfPerson()
{
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
  auto trajectory = mAda->planArmToTSR(personTSR, mCollisionFreeConstraint);
  visualizeTrajectory(trajectory);
  return mAda->moveArmOnTrajectory(
      trajectory,
      mCollisionFreeConstraint,
      ada::TrajectoryPostprocessType::KUNZ,
      velocityLimits);
}

//==============================================================================
bool FeedingDemo::tiltUpInFrontOfPerson()
{
  printRobotConfiguration();

  // Eigen::Vector3d workingPersonTranslation(0.283465, 0.199386, 0.652674);
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

  for (double i = 0; i <= 1.0; i += 0.5)
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

    // auto markers = viewer->addTSRMarker(personTSR, 100, "personTSRSamples");

    bool trajectoryCompleted = false;
    try
    {
      auto trajectory = mAda->planArmToTSR(personTSR, mCollisionFreeConstraint);
      visualizeTrajectory(trajectory);
      bool trajectoryCompleted = mAda->moveArmOnTrajectory(
          trajectory,
          mCollisionFreeConstraint,
          ada::TrajectoryPostprocessType::KUNZ);
    }
    catch (std::runtime_error e)
    {
      ROS_WARN("tilt up trajectory failed!");
      continue;
    }
    return trajectoryCompleted;
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
      auto trajectory = mAda->planArmToTSR(personTSR, mCollisionFreeConstraint);
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

  // tsrMarkers.push_back(viewer->addTSRMarker(personTSR, 100, "someTSRName"));

  std::vector<double> velocityLimits{0.2, 0.2, 0.2, 0.2, 0.2, 0.4};
  auto trajectory = mAda->planArmToTSR(personTSR, mCollisionFreeConstraint);
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
  return mAda->moveArmToEndEffectorOffset(
      Eigen::Vector3d(0, 1, -0.6),
      getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle) * 0.9,
      mCollisionFreeConstraint);
}

//==============================================================================
bool FeedingDemo::moveTowardsPerson(Perception* perception)
{
  if (!mAdaReal)
  {
    moveTowardsPerson();
    return true;
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
      direction.normalized(), direction.norm(), mCollisionFreeConstraint);

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
  moveIntoForque();

  if (waitForUser("Close?"))
  {
    mAda->closeHand();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  waitForUser("Out?");
  moveOutOfForque();

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
  moveIntoForque();

  mAda->openHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  waitForUser("Out?");
  moveOutOfForque();

  waitForUser("Above Plate?");
  moveAbovePlate();
}

//==============================================================================
std::string FeedingDemo::getUserInput(
    bool food_only, ros::NodeHandle& nodeHandle)
{
  ROS_INFO_STREAM("Which food item do you want?");
  for (std::size_t i = 0; i < FOOD_NAMES.size(); ++i)
  {
    ROS_INFO_STREAM("(" << i + 1 << ") " << FOOD_NAMES[i] << std::endl);
  }
  if (!food_only)
  {
    for (std::size_t i = 0; i < ACTIONS.size(); ++i)
    {
      ROS_INFO_STREAM(
          "(" << i + FOOD_NAMES.size() + 1 << ") [" << ACTIONS[i] << "]"
              << std::endl);
    }
  }

  std::string foodName;
  int max_id;

  if (!food_only)
    max_id = FOOD_NAMES.size() + ACTIONS.size();
  else
    max_id = FOOD_NAMES.size();

  while (true)
  {
    std::cout << "> ";
    int id;
    std::cin >> id;
    if (id < 1 || id > max_id)
    {
      ROS_WARN_STREAM("Invalid argument. Try again.");
      continue;
    }
    if (id <= FOOD_NAMES.size())
    {
      foodName = FOOD_NAMES[id - 1];
      nodeHandle.setParam("/deep_pose/forceFoodName", foodName);
      nodeHandle.setParam("/deep_pose/spnet_food_name", foodName);
    }
    else
      foodName = ACTIONS[id - FOOD_NAMES.size()];
    return foodName;
  }
}

//==============================================================================
void FeedingDemo::skewer(
    std::string foodName,
    FTThresholdHelper& ftThresholdHelper,
    Perception& perception,
    ros::NodeHandle nodeHandle,
    bool autoContinueDemo,
    bool adaReal,
    int max_trial_per_item)
{
  bool foodPickedUp = false;
  int num_tries = 0;

  std::vector<std::string> foodNames
      = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
  std::vector<double> skeweringForces
      = getRosParam<std::vector<double>>("/foodItems/forces", nodeHandle);
  std::unordered_map<std::string, double> foodSkeweringForces;

  for (int i = 0; i < foodNames.size(); i++)
    foodSkeweringForces[foodNames[i]] = skeweringForces[i];

  if (foodName == "")
    foodName = getUserInput(true, nodeHandle);

  while (!foodPickedUp)
  {
    // ===== ABOVE PLATE =====
    if (!autoContinueDemo)
      waitForUser("Move forque above plate", TERMINATE_AT_USER_PROMPT);

    moveAbovePlate();

    // ===== PERCEPTION =====
    perception.setFoodName(foodName);
    Eigen::Isometry3d foodTransform;
    if (!perception.perceiveFood(foodTransform, true, mViewer))
    {
      ROS_WARN_STREAM("I can't see the " << foodName << std::endl);
      if (!autoContinueDemo)
        waitForUser("Try perception again?", TERMINATE_AT_USER_PROMPT);
      continue;
    }
    ROS_INFO_STREAM(
        "Alright! Let's get the " << foodName
                                  << "!\033[0;32m  (Gonna skewer with "
                                  << foodSkeweringForces[foodName]
                                  << "N)\033[0m"
                                  << std::endl);

    // ===== ABOVE FOOD =====
    // 0 vertical
    // 1 strawberry-style
    // 2 banana-style
    int pickupAngleMode
        = getRosParam<int>("/study/pickupAngleMode", nodeHandle);

    if (!autoContinueDemo)
      waitForUser("Move forque above food", TERMINATE_AT_USER_PROMPT);

    moveAboveFood(foodTransform, pickupAngleMode, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    if (adaReal)
    {
      if (!perception.perceiveFood(foodTransform, true, mViewer))
      {
        ROS_WARN_STREAM(
            "I can't see the " << foodName << " anymore...\033[0m"
                               << std::endl);
        continue;
      }
    }
    else
    {
      foodTransform = getDefaultFoodTransform();
    }

    moveAboveFood(foodTransform, pickupAngleMode, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ===== INTO FOOD =====
    if (!autoContinueDemo)
      waitForUser("Move forque into food", TERMINATE_AT_USER_PROMPT);

    double zForceBeforeSkewering = 0;
    if (ftThresholdHelper.startDataCollection(50))
    {
      Eigen::Vector3d currentForce, currentTorque;
      while (!ftThresholdHelper.isDataCollectionFinished(
          currentForce, currentTorque))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      zForceBeforeSkewering = currentForce.x();
    }
    double torqueThreshold = 2;

    if (!ftThresholdHelper.setThresholds(
            foodSkeweringForces[foodName], torqueThreshold))
      exit(1);

    if (adaReal)
      moveIntoFood(&perception);
    else
      moveIntoFood();

    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            getRosParam<int>("/feedingDemo/waitMillisecsAtFood", nodeHandle)));
    grabFoodWithForque();

    // ===== OUT OF FOOD =====
    if (!autoContinueDemo)
      waitForUser("Move forque out of food", TERMINATE_AT_USER_PROMPT);
    if (!ftThresholdHelper.setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD))
      exit(1);

    moveOutOfFood();
    if (!ftThresholdHelper.setThresholds(STANDARD_FT_THRESHOLD))
      exit(1);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    double forceDifference = 100;
    double zForceAfter = 0;
    if (ftThresholdHelper.startDataCollection(50))
    {
      Eigen::Vector3d currentForce, currentTorque;
      while (!ftThresholdHelper.isDataCollectionFinished(
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
void FeedingDemo::feedFoodToPerson(
    Perception& perception,
    ros::NodeHandle nodeHandle,
    bool autoContinueDemo,
    bool tilted)
{
  nodeHandle.setParam("/feeding/facePerceptionOn", true);
  moveInFrontOfPerson();

  moveTowardsPerson(&perception);

  if (tilted)
  {
    tiltUpInFrontOfPerson();
  }

  // ===== EATING =====
  ROS_WARN("Human is eating");
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          getRosParam<int>("/feedingDemo/waitMillisecsAtPerson", nodeHandle)));
  ungrabAndDeleteFood();

  if (!autoContinueDemo)
  {
    waitForUser("Move away from person 1", TERMINATE_AT_USER_PROMPT);
    waitForUser("Move away from person 2", TERMINATE_AT_USER_PROMPT);
  }

  if (!tilted)
  {
    moveAwayFromPerson();
  }

  // ===== BACK TO PLATE =====
  if (!autoContinueDemo)
    waitForUser("Move back to plate", TERMINATE_AT_USER_PROMPT);

  moveAbovePlate();
}

} // namespace feeding
