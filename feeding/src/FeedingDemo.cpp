#include "feeding/FeedingDemo.hpp"
#include <aikido/constraint/TestableIntersection.hpp>
#include <pr_tsr/plate.hpp>
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
FeedingDemo::FeedingDemo(bool adaReal, ros::NodeHandle nodeHandle)
  : adaReal(adaReal), nodeHandle(nodeHandle)
{

  world = std::make_shared<aikido::planner::World>("feeding");

  ada = std::unique_ptr<ada::Ada>(
      new ada::Ada(
          world,
          !adaReal,
          getRosParam<std::string>("/ada/urdfUri", nodeHandle),
          getRosParam<std::string>("/ada/srdfUri", nodeHandle),
          getRosParam<std::string>("/ada/endEffectorName", nodeHandle)));
  armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
      ada->getArm()->getMetaSkeleton().get());

  Eigen::Isometry3d robotPose = createIsometry(
      getRosParam<std::vector<double>>("/ada/baseFramePose", nodeHandle));

  workspace = std::unique_ptr<Workspace>(
      new Workspace(world, robotPose, adaReal, nodeHandle));

  // Setting up collisions
  dart::collision::CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(
          ada->getMetaSkeleton().get(),
          ada->getHand()->getEndEffectorBodyNode());
  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup(
          workspace->getTable().get(),
          workspace->getPerson().get(),
          workspace->getWorkspaceEnvironment().get(),
          workspace->getWheelchair().get());
  collisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          armSpace, ada->getArm()->getMetaSkeleton(), collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

  if (adaReal)
  {
    ada->startTrajectoryExecutor();
  }
}

//==============================================================================
FeedingDemo::~FeedingDemo()
{
  if (adaReal)
  {
    // wait for a bit so controller actually stops moving
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    ada->stopTrajectoryExecutor();
  }
}

//==============================================================================
aikido::planner::WorldPtr FeedingDemo::getWorld()
{
  return world;
}

//==============================================================================
Workspace& FeedingDemo::getWorkspace()
{
  return *workspace;
}

//==============================================================================
ada::Ada& FeedingDemo::getAda()
{
  return *ada;
}

//==============================================================================
Eigen::Isometry3d FeedingDemo::getDefaultFoodTransform()
{
  return workspace->getDefaultFoodItem()
      ->getRootBodyNode()
      ->getWorldTransform();
}

//==============================================================================
bool FeedingDemo::isCollisionFree(std::string& result)
{
  auto robotState = ada->getStateSpace()->getScopedStateFromMetaSkeleton(
      ada->getMetaSkeleton().get());
  aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
  if (!collisionFreeConstraint->isSatisfied(robotState, &collisionCheckOutcome))
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
  auto defaultPose = ada->getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
}

//==============================================================================
void FeedingDemo::openHand()
{
  ada->getHand()->executePreshape("open").wait();
}

//==============================================================================
void FeedingDemo::closeHand()
{
  ada->getHand()->executePreshape("closed").wait();
}

//==============================================================================
void FeedingDemo::grabFoodWithForque()
{
  if (!adaReal && workspace->getDefaultFoodItem())
  {
    ada->getHand()->grab(workspace->getDefaultFoodItem());
  }
}

//==============================================================================
void FeedingDemo::ungrabAndDeleteFood()
{
  if (!adaReal)
  {
    ada->getHand()->ungrab();
    workspace->deleteFood();
  }
}

//==============================================================================
void FeedingDemo::moveToStartConfiguration()
{
  auto home
      = getRosParam<std::vector<double>>("/ada/homeConfiguration", nodeHandle);
  if (adaReal)
  {
    // We decided to not move to an initial configuration for now
    // moveArmToConfiguration(Eigen::Vector6d(home.data()));
  }
  else
  {
    ada->getArm()->getMetaSkeleton()->setPositions(
        Eigen::Vector6d(home.data()));
  }
}

//==============================================================================
void FeedingDemo::moveAbovePlate()
{
  double heightAbovePlate
      = getRosParam<double>("/feedingDemo/heightAbovePlate", nodeHandle);
  double horizontalToleranceAbovePlate = getRosParam<double>(
      "/planning/tsr/horizontalToleranceAbovePlate", nodeHandle);
  double verticalToleranceAbovePlate = getRosParam<double>(
      "/planning/tsr/verticalToleranceAbovePlate", nodeHandle);

  auto abovePlateTSR = pr_tsr::getDefaultPlateTSR();
  abovePlateTSR.mT0_w
      = workspace->getPlate()->getRootBodyNode()->getWorldTransform();
  abovePlateTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAbovePlate};

  abovePlateTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceAbovePlate, verticalToleranceAbovePlate, 0, 0);
  abovePlateTSR.mTw_e.matrix()
      *= ada->getHand()->getEndEffectorTransform("plate")->matrix();

  bool trajectoryCompleted = moveArmToTSR(abovePlateTSR);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveAboveFood(const Eigen::Isometry3d& foodTransform)
{
  double heightAboveFood
      = getRosParam<double>("/feedingDemo/heightAboveFood", nodeHandle);
  // If the robot is not simulated, we want to plan the trajectory to move a
  // little further downwards,
  // so that the MoveUntilTouchController can take care of stopping the
  // trajectory.
  double heightIntoFood
      = adaReal ? getRosParam<double>("/feedingDemo/heightIntoFood", nodeHandle)
                : 0.0;
  double horizontalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearFood", nodeHandle);
  double verticalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearFood", nodeHandle);

  aikido::constraint::dart::TSR aboveFoodTSR;
  aboveFoodTSR.mT0_w = foodTransform;
  aboveFoodTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearFood, verticalToleranceNearFood, -M_PI, M_PI);
  aboveFoodTSR.mTw_e.matrix()
      *= ada->getHand()->getEndEffectorTransform("plate")->matrix();
  aboveFoodTSR.mTw_e.translation()
      = Eigen::Vector3d{0, 0, heightAboveFood - heightIntoFood};

  bool trajectoryCompleted = moveArmToTSR(aboveFoodTSR);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveIntoFood()
{
  bool trajectoryCompleted = moveWithEndEffectorOffset(
      Eigen::Vector3d(0, 0, -1),
      getRosParam<double>("/feedingDemo/heightAboveFood", nodeHandle));
  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
}

//==============================================================================
void FeedingDemo::moveOutOfFood()
{
  bool trajectoryCompleted = moveWithEndEffectorOffset(
      Eigen::Vector3d(0, 0, 1),
      getRosParam<double>("/feedingDemo/heightAboveFood", nodeHandle));
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveInFrontOfPerson()
{
  double distanceToPerson
      = getRosParam<double>("/feedingDemo/distanceToPerson", nodeHandle);
  double horizontalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearPerson", nodeHandle);
  double verticalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearPerson", nodeHandle);

  aikido::constraint::dart::TSR personTSR;
  Eigen::Isometry3d personPose = Eigen::Isometry3d::Identity();
  personPose.translation() = workspace->getPerson()
                                 ->getRootBodyNode()
                                 ->getWorldTransform()
                                 .translation();
  personPose.linear()
      = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  personTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearPerson, verticalToleranceNearPerson, 0, 0);
  personTSR.mTw_e.matrix()
      *= ada->getHand()->getEndEffectorTransform("person")->matrix();

  bool trajectoryCompleted = moveArmToTSR(personTSR);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
void FeedingDemo::moveTowardsPerson()
{
  bool trajectoryCompleted = moveWithEndEffectorOffset(
      Eigen::Vector3d(0, 1, 0),
      getRosParam<double>("/feedingDemo/distanceToPerson", nodeHandle) * 0.9);
}

//==============================================================================
void FeedingDemo::moveAwayFromPerson()
{
  bool trajectoryCompleted = moveWithEndEffectorOffset(
      Eigen::Vector3d(0, -1, 0),
      getRosParam<double>("/feedingDemo/distanceFromPerson", nodeHandle));
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
bool FeedingDemo::moveArmToTSR(const aikido::constraint::dart::TSR& tsr)
{
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(tsr);

  auto trajectory = ada->planToTSR(
      armSpace,
      ada->getArm()->getMetaSkeleton(),
      ada->getHand()->getEndEffectorBodyNode(),
      goalTSR,
      collisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", nodeHandle),
      getRosParam<int>("/planning/maxNumberOfTrials", nodeHandle));

  return moveArmOnTrajectory(trajectory);
}

//==============================================================================
bool FeedingDemo::moveWithEndEffectorOffset(
    const Eigen::Vector3d& direction, double length)
{
  auto trajectory = ada->planToEndEffectorOffset(
      armSpace,
      ada->getArm()->getMetaSkeleton(),
      ada->getHand()->getEndEffectorBodyNode(),
      collisionFreeConstraint,
      direction,
      length,
      getRosParam<double>("/planning/timeoutSeconds", nodeHandle),
      getRosParam<double>(
          "/planning/endEffectorOffset/positionTolerance", nodeHandle),
      getRosParam<double>(
          "/planning/endEffectorOffset/angularTolerance", nodeHandle));

  return moveArmOnTrajectory(trajectory, RETIME);
}

//==============================================================================
bool FeedingDemo::moveArmToConfiguration(const Eigen::Vector6d& configuration)
{
  auto trajectory = ada->planToConfiguration(
      armSpace,
      ada->getArm()->getMetaSkeleton(),
      configuration,
      collisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", nodeHandle));

  return moveArmOnTrajectory(trajectory, SMOOTH);
}

//==============================================================================
bool FeedingDemo::moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    TrajectoryPostprocessType postprocessType)
{
  if (!trajectory)
  {
    throw std::runtime_error("Trajectory execution failed: Empty trajectory.");
  }

  std::vector<aikido::constraint::ConstTestablePtr> constraints;
  if (collisionFreeConstraint)
  {
    constraints.push_back(collisionFreeConstraint);
  }
  auto testable = std::make_shared<aikido::constraint::TestableIntersection>(
      armSpace, constraints);

  aikido::trajectory::TrajectoryPtr timedTrajectory;
  switch (postprocessType)
  {
    case RETIME:
      timedTrajectory
          = ada->retimePath(ada->getArm()->getMetaSkeleton(), trajectory.get());
      break;

    case SMOOTH:
      timedTrajectory = ada->smoothPath(
          ada->getArm()->getMetaSkeleton(), trajectory.get(), testable);
      break;

    default:
      throw std::runtime_error(
          "Feeding demo: Unexpected trajectory post processing type!");
  }

  auto future = ada->executeTrajectory(std::move(timedTrajectory));
  try
  {
    future.get();
  }
  catch (const std::exception& e)
  {
    ROS_INFO_STREAM("trajectory execution failed: " << e.what());
    return false;
  }
  return true;
}
};
