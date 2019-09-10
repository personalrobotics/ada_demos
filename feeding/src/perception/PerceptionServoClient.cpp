#include "feeding/perception/PerceptionServoClient.hpp"
#include <chrono>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/kunzretimer/KunzRetimer.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/trajectory/util.hpp>
#include <libada/util.hpp>

#include "feeding/util.hpp"

#define THRESHOLD 5.0 // s to wait for good frame

using ada::util::getRosParam;
using aikido::constraint::Satisfied;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::SnapConfigurationToConfigurationPlanner;
using aikido::planner::kunzretimer::computeKunzTiming;
using aikido::planner::vectorfield::planToEndEffectorOffset;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::trajectory::concatenate;
using aikido::trajectory::createPartialTrajectory;
using aikido::trajectory::findTimeOfClosestStateOnTrajectory;
using aikido::trajectory::Interpolated;
using aikido::trajectory::Spline;
using aikido::trajectory::SplinePtr;
using aikido::trajectory::TrajectoryPtr;
using aikido::trajectory::UniqueInterpolatedPtr;
using aikido::trajectory::UniqueSplinePtr;

namespace feeding {

namespace {

Eigen::VectorXd getSymmetricLimits(
    const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits)
{
  assert(
      static_cast<std::size_t>(lowerLimits.size())
      == static_cast<std::size_t>(upperLimits.size()));

  std::size_t limitSize = static_cast<std::size_t>(lowerLimits.size());
  Eigen::VectorXd symmetricLimits(limitSize);
  for (std::size_t i = 0; i < limitSize; ++i)
  {
    symmetricLimits[i] = std::min(-lowerLimits[i], upperLimits[i]);
  }
  return symmetricLimits;
}

} // namespace

//==============================================================================
PerceptionServoClient::PerceptionServoClient(
    const ::ros::NodeHandle* node,
    boost::function<Eigen::Isometry3d(void)> getTransform,
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
        metaSkeletonStateSpace,
    std::shared_ptr<ada::Ada> ada,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::BodyNodePtr bodyNode,
    std::shared_ptr<aikido::control::TrajectoryExecutor> trajectoryExecutor,
    aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
    double perceptionUpdateTime,
    float goalPrecision,
    double planningTimeout,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    bool servoFood,
    std::vector<double> velocityLimits)
  : mNodeHandle(*node, "perceptionServo")
  , mGetTransform(getTransform)
  , mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mAda(std::move(ada))
  , mMetaSkeleton(std::move(metaSkeleton))
  , mBodyNode(bodyNode)
  , mTrajectoryExecutor(trajectoryExecutor)
  , mCollisionFreeConstraint(collisionFreeConstraint)
  , mPerceptionUpdateTime(perceptionUpdateTime)
  , mCurrentTrajectory(nullptr)
  , mGoalPrecision(goalPrecision)
  , mPlanningTimeout(planningTimeout)
  , mEndEffectorOffsetPositionTolerance(endEffectorOffsetPositionTolerance)
  , mEndEffectorOffsetAngularTolerance(endEffectorOffsetAngularTolerance)
  , mServoFood(servoFood)
  , mIsRunning(false)
{
  mNonRealtimeTimer = mNodeHandle.createTimer(
      ros::Duration(mPerceptionUpdateTime),
      &PerceptionServoClient::nonRealtimeCallback,
      this,
      false,
      false);

  // update Max velocity and acceleration
  mMaxAcceleration = getSymmetricLimits(
      mMetaSkeleton->getAccelerationLowerLimits(),
      mMetaSkeleton->getAccelerationUpperLimits());

  mOriginalPose = mBodyNode->getTransform();
  mOriginalConfig = mMetaSkeleton->getPositions();

  mVelocityLimits = Eigen::VectorXd(velocityLimits.size());
  for (std::size_t i = 0; i < velocityLimits.size(); ++i)
    mVelocityLimits[i] = velocityLimits[i];
}

//==============================================================================
PerceptionServoClient::~PerceptionServoClient()
{
  if (mTrajectoryExecutor)
  {
    mTrajectoryExecutor->cancel();
  }

  if (mExec.valid())
  {
    mExec.wait_for(std::chrono::duration<int, std::milli>(1000));
  }

  mNonRealtimeTimer.stop();
  mSub.shutdown();
  ROS_WARN("shutting down perception servo client");
}

//==============================================================================
void PerceptionServoClient::start()
{
  ROS_INFO("Servoclient started");
  mExecutionDone = false;
  mNonRealtimeTimer.start();
  mNotFailed = false;
  mStartTime = std::chrono::system_clock::now();
  mLastSuccess = mStartTime;
}

//==============================================================================
void PerceptionServoClient::stop()
{
  mTimerMutex.lock();
  // Always cancel the executing trajectory when quitting
  mTrajectoryExecutor->cancel();
  mNonRealtimeTimer.stop();
  mIsRunning = false;
  mTimerMutex.unlock();
}

//==============================================================================
bool PerceptionServoClient::wait(double timelimit)
{
  double elapsedTime = 0.0;
  std::chrono::time_point<std::chrono::system_clock> startTime
      = std::chrono::system_clock::now();
  while (elapsedTime < timelimit && !mExecutionDone)
  {

    // sleep a while
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(
                      std::chrono::system_clock::now() - startTime)
                      .count();
  }
  stop();
  if (elapsedTime >= timelimit)
    ROS_INFO_STREAM(
        "Timeout " << timelimit << " reached for PerceptionServoClient");

  return mNotFailed;
}

//==============================================================================
bool PerceptionServoClient::isRunning()
{
  return mIsRunning;
}

//==============================================================================
void PerceptionServoClient::nonRealtimeCallback(const ros::TimerEvent& event)
{
  if (mExecutionDone || !mTimerMutex.try_lock())
    return;

  Eigen::Isometry3d goalPose;
  if (updatePerception(goalPose))
  {
    mLastSuccess = std::chrono::system_clock::now();
    // Generate a new reference trajectory to the goal pose
    mCurrentTrajectory = planToGoalPose(goalPose);
    if (mExecutionDone)
    {
      ROS_WARN_STREAM("Completed");
      mTimerMutex.unlock();
      return;
    }

    if (!mCurrentTrajectory)
    {
      ROS_WARN_STREAM("Failed to get trajectory");
      mTimerMutex.unlock();
      return;
    }
    // Save current pose
    mOriginalPose = mBodyNode->getTransform();
    mOriginalConfig = mMetaSkeleton->getPositions();

    if (mIsRunning && mExec.valid()
        && (mExec.wait_for(std::chrono::duration<int, std::milli>(0))
            != std::future_status::ready))
    {
      ROS_INFO_STREAM("Cancel the current trajectory");
      mTrajectoryExecutor->cancel();
      mExec.wait();
    }

    // Execute the new reference trajectory
    mExec = mTrajectoryExecutor->execute(mCurrentTrajectory);
    mIsRunning = true;
  }
  else
  {
    double sinceLast
        = std::chrono::duration_cast<std::chrono::duration<double>>(
              std::chrono::system_clock::now() - mLastSuccess)
              .count();
    if (sinceLast > THRESHOLD)
    {
      ROS_WARN("Lost perception for too long. Reporting failure...");
      mExecutionDone = true;
      mNotFailed = false;
    }
    else
    {
      ROS_WARN_STREAM("Perception Failed. Since Last: " << sinceLast);
    }
  }
  mTimerMutex.unlock();
}

//==============================================================================
bool PerceptionServoClient::updatePerception(Eigen::Isometry3d& goalPose)
{
  // update new goal Pose
  Eigen::Isometry3d endEffectorTransform(Eigen::Isometry3d::Identity());
  Eigen::Matrix3d rotation(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  endEffectorTransform.linear() = rotation;

  Eigen::Isometry3d pose;
  try
  {
    pose = mGetTransform();
    if (mRemoveRotation)
      removeRotation(pose);
  }
  catch (std::runtime_error& e)
  {
    ROS_WARN_STREAM(e.what());
    return false;
  }

  goalPose = pose * endEffectorTransform;
  std::cout << "Goal Pose " << goalPose.translation().z() << std::endl;
  if (goalPose.translation().z() < -0.1)
  {
    ROS_WARN_STREAM("Food is way too low, z " << goalPose.translation()[2]);
    return false;
  }
  return true;
}

//==============================================================================
SplinePtr PerceptionServoClient::planToGoalPose(
    const Eigen::Isometry3d& goalPose)
{
  // auto oldLimits = setPositionLimits(mMetaSkeleton);
  Eigen::Isometry3d currentPose = mBodyNode->getTransform();
  std::cout << "current Position " << currentPose.translation().transpose()
            << std::endl;
  Eigen::Vector3d goalDirection
      = goalPose.translation() - currentPose.translation();
  std::cout << "Distance " << goalDirection.norm() << std::endl;

  if (goalDirection.norm() < mGoalPrecision)
  {
    ROS_WARN("Visual servoing is finished because goal was position reached.");
    mExecutionDone = true;
    mNotFailed = true;
    return nullptr;
  }
  if (goalDirection[2] > 0 && mServoFood)
  {
    ROS_WARN(
        "Visual servoing is finished because goal is above the current pose");
    mExecutionDone = true;
    mNotFailed = true;
    return nullptr;
  }

  std::cout << "Goal direction " << goalDirection.transpose() << std::endl;
  // ============= Plan from current pose to goal  ==================//
  auto trajToGoal = planEndEffectorOffset(goalDirection);
  if (!trajToGoal)
  {
    ROS_WARN_STREAM("Plan failed");
    return nullptr;
  }

  // ============= Plan from original pose to current pose ==================//
  /*
  auto originalState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->convertPositionsToState(
      mOriginalConfig, originalState);

  Eigen::Vector3d directionFromOldToNew(
      mBodyNode->getTransform().translation() - mOriginalPose.translation());

  UniqueInterpolatedPtr trajFromOldToNew = nullptr;
  std::cout << "Distance from old to new " << directionFromOldToNew.norm()
            << std::endl;

  if (directionFromOldToNew.norm() > 0.001)
  {
    trajFromOldToNew = planToEndEffectorOffset(
        mMetaSkeletonStateSpace,
        *originalState,
        mMetaSkeleton,
        mBodyNode,
        std::make_shared<Satisfied>(mMetaSkeletonStateSpace),
        directionFromOldToNew.normalized(),
        0.0,
        directionFromOldToNew.norm(),
        0.08,
        0.32,
        0.001,
        1e-3,
        1e-2,
        std::chrono::duration<double>(5));
  }
  */

  // ============= Concatenate the two trajectories ==================//
  UniqueSplinePtr timedTraj;
  // if (trajFromOldToNew)
  // {
  //   ROS_INFO_STREAM("Concatenate two trajectories");
  //   auto concatenatedTraj = concatenate(
  //     *dynamic_cast<Interpolated*>(trajFromOldToNew.get()),
  //     *dynamic_cast<Interpolated*>(trajToGoal.get()));
  //   timedTraj = computeKunzTiming(
  //       *dynamic_cast<Interpolated*>(concatenatedTraj.get()),
  //       mVelocityLimits, mMaxAcceleration, 1e-2, 3e-3);
  // }
  // else
  // {
  timedTraj = computeKunzTiming(
      *dynamic_cast<Interpolated*>(trajToGoal.get()),
      mVelocityLimits,
      mMaxAcceleration,
      1e-2,
      3e-3);
  // }

  if (!timedTraj)
  {
    ROS_WARN_STREAM("Concatenation &/ timing failed");
    return nullptr;
  }
  // setPositionLimits(mMetaSkeleton, oldLimits.first, oldLimits.second);

  //  Start from the closest point on the trajectory
  timedTraj = createPartialTimedTrajectoryFromCurrentConfig(timedTraj.get());
  return timedTraj;
}

//==============================================================================
TrajectoryPtr PerceptionServoClient::planEndEffectorOffset(
    const Eigen::Vector3d& goalDirection)
{
  if (goalDirection.norm() < 1e-3)
    return nullptr;

  return mAda->planArmToEndEffectorOffset(
      goalDirection.normalized(),
      std::min(goalDirection.norm(), 0.03),
      mCollisionFreeConstraint,
      mPlanningTimeout,
      mEndEffectorOffsetPositionTolerance,
      mEndEffectorOffsetAngularTolerance);
}

//==============================================================================
UniqueSplinePtr
PerceptionServoClient::createPartialTimedTrajectoryFromCurrentConfig(
    const Spline* trajectory)
{
  double distance;
  auto state = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->convertPositionsToState(
      mMetaSkeleton->getPositions(), state);

  double refTime
      = findTimeOfClosestStateOnTrajectory(*trajectory, state, distance, 0.01);

  if (distance > 1.0)
  {
    ROS_WARN_STREAM("Distance too far " << distance);
    return nullptr;
  }

  std::cout << "Shorted distance " << distance << " at " << refTime
            << std::endl;
  // Start 0.3 sec forward since the robort has been moving.
  refTime += 0.3;
  if(refTime > trajectory->getEndTime()) {
    ROS_WARN_STREAM("Robot already reached end of trajectory.");
    return nullptr;
  }
  auto traj = createPartialTrajectory(*trajectory, refTime);
  if (!traj || traj->getDuration() < 1e-5)
  {
    ROS_WARN_STREAM("Trajectory duration too short.");
    return nullptr;
  }
  return traj;
}

} // namespace feeding
