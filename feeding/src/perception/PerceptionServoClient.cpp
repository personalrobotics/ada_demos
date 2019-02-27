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

static std::string JOINT_STATE_TOPIC_NAME = "/joint_states";

#define THRESHOLD 5.0 // s to wait for good frame

using aikido::trajectory::createPartialTrajectory;
using aikido::trajectory::findTimeOfClosestStateOnTrajectory;
using aikido::trajectory::concatenate;
using aikido::trajectory::SplinePtr;
using dart::dynamics::InverseKinematics;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::SnapConfigurationToConfigurationPlanner;
using aikido::planner::kunzretimer::computeKunzTiming;
using aikido::trajectory::Interpolated;
using aikido::trajectory::Spline;
using ada::util::getRosParam;

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
    float originalDirectionExtension,
    float goalPrecision,
    double planningTimeout,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    std::vector<double> veloctiyLimits)
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
  , mOriginalDirectionExtension(originalDirectionExtension)
  , mGoalPrecision(goalPrecision)
  , mPlanningTimeout(planningTimeout)
  , mEndEffectorOffsetPositionTolerance(endEffectorOffsetPositionTolerance)
  , mEndEffectorOffsetAngularTolerance(endEffectorOffsetAngularTolerance)
  , mVelocityLimits(veloctiyLimits)
{
  mNonRealtimeTimer = mNodeHandle.createTimer(
      ros::Duration(mPerceptionUpdateTime),
      &PerceptionServoClient::nonRealtimeCallback,
      this,
      false,
      false);

  // subscribe to the joint state publisher
  mSub = mNodeHandle.subscribe(
      JOINT_STATE_TOPIC_NAME,
      10,
      &PerceptionServoClient::jointStateUpdateCallback,
      this);

  // update Max velocity and acceleration
  mMaxAcceleration = getSymmetricLimits(
      mMetaSkeleton->getAccelerationLowerLimits(),
      mMetaSkeleton->getAccelerationUpperLimits());

  mCurrentPosition = Eigen::VectorXd::Zero(mMetaSkeleton->getNumDofs());
  mCurrentVelocity = Eigen::VectorXd::Zero(mMetaSkeleton->getNumDofs());

  mOriginalPose = mBodyNode->getTransform();
  mOriginalConfig = mMetaSkeleton->getPositions();
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
  mIsRunning = true;
  mNotFailed = true;
  mStartTime = std::chrono::system_clock::now();
  mLastSuccess = mStartTime;
}

//==============================================================================
void PerceptionServoClient::jointStateUpdateCallback(
    const sensor_msgs::JointState::ConstPtr& msg)
{
  std::lock_guard<std::mutex> currPosVelLock{mJointStateUpdateMutex,
                                             std::adopt_lock};
  std::size_t velDimSize = mCurrentVelocity.size();
  for (std::size_t i = 0; i < velDimSize; i++)
  {
    mCurrentVelocity[i] = msg->velocity[i];
    mCurrentPosition[i] = msg->position[i];
  }
}

//==============================================================================
double PerceptionServoClient::getElapsedTime()
{
  double elapsedTime
      = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now() - mStartTime)
            .count();
  return elapsedTime;
}

//==============================================================================
void PerceptionServoClient::stop()
{
  timerMutex.lock();
  // Always cancel the executing trajectory when quitting
  mTrajectoryExecutor->cancel();
  mNonRealtimeTimer.stop();
  mIsRunning = false;
  timerMutex.unlock();
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
  if (!timerMutex.try_lock())
  {
    return;
  }

  if (mExecutionDone)
  {
    timerMutex.unlock();
    return;
  }

  // check for exceptions (for example the controller cancel the trajectory)
  if (mExec.valid())
  {
    if (mExec.wait_for(std::chrono::duration<int, std::milli>(0))
        == std::future_status::ready)
    {
      try
      {
        mExec.get();
      }
      catch (const std::exception& e)
      {
        ROS_WARN_STREAM(e.what());
        mExecutionDone = true;
        timerMutex.unlock();
        return;
      }
    }
  }

  Eigen::Isometry3d goalPose;
  if (updatePerception(goalPose))
  {
    mLastSuccess = std::chrono::system_clock::now();
    // Generate a new reference trajectory to the goal pose
    auto planningStartTime = std::chrono::steady_clock::now();
    mCurrentTrajectory = planToGoalPoseAndResetMetaSkeleton(goalPose);
    if (!mCurrentTrajectory && mExecutionDone)
    {
      timerMutex.unlock();
      return;
    }

    mTrajectoryExecutor->cancel();

    if (mExec.valid())
    {
      if (mExec.wait_for(std::chrono::duration<int, std::milli>(0))
          == std::future_status::ready)
      {
        try
        {
          mExec.get();
        }
        catch (const std::exception& e)
        {
          ROS_WARN_STREAM(e.what());
          mExecutionDone = true;
          timerMutex.unlock();
          return;
        }
      }
    }

    // Execute the new reference trajectory
    if (mCurrentTrajectory)
    {
      if (mExec.valid())
        mExec.wait();
      mExec = mTrajectoryExecutor->execute(mCurrentTrajectory);
    }
    else
    {
      timerMutex.unlock();
      ROS_WARN_STREAM("cannot find a feasible path");
      return;
    }
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

  timerMutex.unlock();
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
    {
      removeRotation(pose);
    }
  }
  catch (std::runtime_error& e)
  {
    ROS_WARN_STREAM(e.what());
    return false;
  }

  goalPose = pose * endEffectorTransform;

  ROS_INFO_STREAM("goal pose: " << goalPose.translation().transpose().matrix());
  if (goalPose.translation().z() < -0.1)
  {
    ROS_WARN_STREAM("Food is way too low, z " << goalPose.translation()[2]);
    return false;
  }

  if (mHasOriginalDirection)
  {
    goalPose.translation() = goalPose.translation()
                             + originalDirection * mOriginalDirectionExtension;
    ROS_INFO_STREAM(
        "goal pose: " << goalPose.translation().transpose().matrix());
  }
  return true;
}

//==============================================================================
SplinePtr PerceptionServoClient::planToGoalPoseAndResetMetaSkeleton(
    const Eigen::Isometry3d& goalPose)
{
  auto oldLimits = setPositionLimits(mMetaSkeleton);

  auto trajectory = planToGoalPose(goalPose);

  setPositionLimits(mMetaSkeleton, oldLimits.first, oldLimits.second);

  return trajectory;
}

//==============================================================================
SplinePtr PerceptionServoClient::planToGoalPose(
    const Eigen::Isometry3d& goalPose)
{
  Eigen::VectorXd velocityLimits(mVelocityLimits.size());

  for (std::size_t i = 0; i < mVelocityLimits.size(); ++i)
    velocityLimits[i] = mVelocityLimits[i];

  Eigen::Isometry3d currentPose = mBodyNode->getTransform();
  aikido::trajectory::Interpolated* interpolated2 = nullptr;

  Eigen::Vector3d direction2
      = goalPose.translation() - currentPose.translation();
  if (direction2.norm() < mGoalPrecision)
  {
    ROS_INFO("Visual servoing is finished because goal was position reached.");
    mExecutionDone = true;
    return nullptr;
  }

  aikido::trajectory::TrajectoryPtr trajectory2 = nullptr;
  {
    // auto limits = setPositionLimits(mAda->getArm()->getMetaSkeleton());

    auto trajectory2 = mAda->planArmToEndEffectorOffset(
        direction2.normalized(),
        std::min(direction2.norm(), 0.2),
        nullptr,
        mPlanningTimeout,
        mEndEffectorOffsetPositionTolerance,
        mEndEffectorOffsetAngularTolerance);

    // setPositionLimits(
    //     mAda->getArm()->getMetaSkeleton(), limits.first, limits.second);

    if (!trajectory2)
    {
      ROS_INFO_STREAM("[PerceptionServoClient] Failed to plan");
      mExecutionDone = true;
      mNotFailed = false;
      return nullptr;
    }

    interpolated2 = dynamic_cast<aikido::trajectory::Interpolated*>(trajectory2.get());
    if (interpolated2 == nullptr)
      return nullptr;

    auto timedTraj = computeKunzTiming(
        *interpolated2, velocityLimits, mMaxAcceleration, 1e-2, 3e-3);

    if (!timedTraj)
    {
      ROS_WARN_STREAM("[PerceptionServoClient] Failed to retime");
      mExecutionDone = true;
      mNotFailed = false;
      return nullptr;
    }
    return std::move(timedTraj);
  }
}

} // namespace feeding
