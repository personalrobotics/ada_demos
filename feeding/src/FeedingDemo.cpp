#include "feeding/FeedingDemo.hpp"
#include <aikido/constraint/TestableIntersection.hpp>
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
                                          : "trajectory_controller";

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

  // Setting up collisions
  mCollisionFreeConstraint
      = nullptr;


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
void FeedingDemo::moveToStartConfiguration()
{
  auto home
      = getRosParam<std::vector<double>>("/ada/homeConfiguration", mNodeHandle);
  if (mAdaReal)
  {
    // We decided to not move to an initial configuration for now
    // moveArmToConfiguration(Eigen::Vector6d(home.data()));
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

  Eigen::Isometry3d robotPose = createIsometry(
      getRosParam<std::vector<double>>("/ada/baseFramePose", mNodeHandle));
  Eigen::Isometry3d platePose = createIsometry(
      getRosParam<std::vector<double>>("/plate/pose", mNodeHandle));
  auto abovePlateTSR = pr_tsr::getDefaultPlateTSR();
  abovePlateTSR.mT0_w
      = robotPose.inverse() * platePose;
  abovePlateTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAbovePlate};

  abovePlateTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceAbovePlate, verticalToleranceAbovePlate, 0, 0);
  abovePlateTSR.mTw_e.matrix()
      *= mAda->getHand()->getEndEffectorTransform("plate")->matrix();

  bool trajectoryCompleted = moveArmToTSR(abovePlateTSR);
  if (!trajectoryCompleted)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

//==============================================================================
bool FeedingDemo::moveArmToTSR(const aikido::constraint::dart::TSR& tsr)
{
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(tsr);

  auto trajectory = mAda->planToTSR(
      mArmSpace,
      mAda->getArm()->getMetaSkeleton(),
      mAda->getHand()->getEndEffectorBodyNode(),
      goalTSR,
      mCollisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle));

  if (!trajectory) {
    throw std::runtime_error("yes, it's definitely the planToTSR");
  }

  return moveArmOnTrajectory(trajectory);
}

//==============================================================================
bool FeedingDemo::moveWithEndEffectorOffset(
    const Eigen::Vector3d& direction, double length)
{
  auto trajectory = mAda->planToEndEffectorOffset(
      mArmSpace,
      mAda->getArm()->getMetaSkeleton(),
      mAda->getHand()->getEndEffectorBodyNode(),
      mCollisionFreeConstraint,
      direction,
      length,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<double>(
          "/planning/endEffectorOffset/positionTolerance", mNodeHandle),
      getRosParam<double>(
          "/planning/endEffectorOffset/angularTolerance", mNodeHandle));

  return moveArmOnTrajectory(trajectory, RETIME);
}

//==============================================================================
bool FeedingDemo::moveArmToConfiguration(const Eigen::Vector6d& configuration)
{
  auto trajectory = mAda->planToConfiguration(
      mArmSpace,
      mAda->getArm()->getMetaSkeleton(),
      configuration,
      mCollisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle));

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
  if (mCollisionFreeConstraint)
  {
    constraints.push_back(mCollisionFreeConstraint);
  }
  auto testable = std::make_shared<aikido::constraint::TestableIntersection>(
      mArmSpace, constraints);

  aikido::trajectory::TrajectoryPtr timedTrajectory;
  switch (postprocessType)
  {
    case RETIME:
      timedTrajectory = mAda->retimePath(
          mAda->getArm()->getMetaSkeleton(), trajectory.get());
      break;

    case SMOOTH:
      timedTrajectory = mAda->smoothPath(
          mAda->getArm()->getMetaSkeleton(), trajectory.get(), testable);
      break;

    default:
      throw std::runtime_error(
          "Feeding demo: Unexpected trajectory post processing type!");
  }

  auto future = mAda->executeTrajectory(std::move(timedTrajectory));
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
