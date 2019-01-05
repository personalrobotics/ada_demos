#include "feeding/perception/PerceptionServoClient.hpp"
#include <chrono>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/kunzretimer/KunzRetimer.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/trajectory/util.hpp>

#include "feeding/util.hpp"

static std::string JOINT_STATE_TOPIC_NAME = "/joint_states";

#define THRESHOLD 0.5 // s to wait for good frame

using aikido::trajectory::createPartialTrajectory;
using aikido::trajectory::findTimeOfClosestStateOnTrajectory;
using aikido::trajectory::concatenate;
using aikido::trajectory::SplinePtr;

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
    ::ros::NodeHandle node,
    boost::function<bool(Eigen::Isometry3d&)> getTransform,
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
        metaSkeletonStateSpace,
    std::shared_ptr<ada::Ada> ada,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::BodyNodePtr bodyNode,
    std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor>
        trajectoryExecutor,
    aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
    double perceptionUpdateTime,
    const Eigen::VectorXd& veloctiyLimits,
    float originalDirectionExtension,
    float goalPrecision)
  : mNode(node, "perceptionServo")
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
{
  mNonRealtimeTimer = mNode.createTimer(
      ros::Duration(mPerceptionUpdateTime),
      &PerceptionServoClient::nonRealtimeCallback,
      this,
      false,
      false);

  // subscribe to the joint state publisher
  mSub = mNode.subscribe(
      JOINT_STATE_TOPIC_NAME,
      10,
      &PerceptionServoClient::jointStateUpdateCallback,
      this);

  // update Max velocity and acceleration
  Eigen::VectorXd velocityLowerLimits = mMetaSkeleton->getVelocityLowerLimits();
  Eigen::VectorXd velocityUpperLimits = mMetaSkeleton->getVelocityUpperLimits();
  Eigen::VectorXd accelerationLowerLimits
      = mMetaSkeleton->getAccelerationLowerLimits();
  Eigen::VectorXd accelerationUpperLimits
      = mMetaSkeleton->getAccelerationUpperLimits();
  mMaxVelocity = veloctiyLimits;
  mMaxAcceleration
      = getSymmetricLimits(accelerationLowerLimits, accelerationUpperLimits);

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
  using aikido::planner::vectorfield::computeGeodesicDistance;

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
    auto planningDuration
        = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - planningStartTime);
    // ROS_INFO_STREAM("Planning took " << planningDuration.count() << "
    // millisecs");

    // ROS_INFO("cancel old trajectory");
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
      throw std::runtime_error("cannot find a feasible path");
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
  Eigen::Isometry3d endEffectorTransform = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rotationMatrix(
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  endEffectorTransform.linear() = rotationMatrix;
  bool successful = mGetTransform(goalPose);
  ROS_INFO_STREAM("servo client pose: " << goalPose.matrix());
  goalPose = goalPose * endEffectorTransform;
  ROS_INFO_STREAM("servo client pose: " << goalPose.matrix());
  ROS_INFO_STREAM("goal pose: " << goalPose.translation().transpose().matrix());
  if (goalPose.translation().z() < 0.115)
  {
    ROS_WARN_STREAM("Food is way too low");
    return false;
  }

  if (hasOriginalDirection)
  {
    goalPose.translation() = goalPose.translation()
                             + originalDirection * mOriginalDirectionExtension;
  }
  return successful;
}

//==============================================================================
SplinePtr PerceptionServoClient::planToGoalPoseAndResetMetaSkeleton(
    const Eigen::Isometry3d& goalPose)
{
  auto trajectory = planToGoalPose(goalPose);

  std::vector<int> indices{0, 3, 4, 5};
  auto llimits = mMetaSkeleton->getPositionLowerLimits();
  auto ulimits = mMetaSkeleton->getPositionUpperLimits();

  for (int i = 0; i < indices.size(); ++i)
  {
    llimits(indices[i]) = -dart::math::constantsd::inf();
    ulimits(indices[i]) = dart::math::constantsd::inf();
  }
  mMetaSkeleton->setPositionLowerLimits(llimits);
  mMetaSkeleton->setPositionUpperLimits(ulimits);

  return trajectory;
}

//==============================================================================
SplinePtr PerceptionServoClient::planToGoalPose(
    const Eigen::Isometry3d& goalPose)
{
  using dart::dynamics::InverseKinematics;
  using aikido::statespace::dart::MetaSkeletonStateSaver;
  using aikido::planner::ConfigurationToConfiguration;
  using aikido::planner::SnapConfigurationToConfigurationPlanner;
  using aikido::planner::kunzretimer::computeKunzTiming;
  using aikido::trajectory::Interpolated;
  using aikido::trajectory::Spline;

  Eigen::Isometry3d currentPose = mBodyNode->getTransform();

  aikido::trajectory::Spline* spline1 = nullptr;
  aikido::trajectory::Spline* spline2 = nullptr;

  Eigen::VectorXd currentConfig = mMetaSkeleton->getPositions();
  Eigen::Vector3d direction1
      = currentPose.translation() - mOriginalPose.translation();

  std::vector<int> indices{0, 3, 4, 5};
  std::vector<double> tempLower{-12.56, -12.56, -12.56, -12.56};
  std::vector<double> tempUpper{12.56, 12.56, 12.56, 12.56};
  auto llimits = mMetaSkeleton->getPositionLowerLimits();
  auto ulimits = mMetaSkeleton->getPositionUpperLimits();
  for (int i = 0; i < indices.size(); ++i)
  {
    llimits(indices[i]) = tempLower[i];
    ulimits(indices[i]) = tempUpper[i];
  }
  mMetaSkeleton->setPositionLowerLimits(llimits);
  mMetaSkeleton->setPositionUpperLimits(ulimits);

  auto tempSpace
      = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          mMetaSkeleton.get());

  aikido::trajectory::TrajectoryPtr trajectory1 = nullptr;
  aikido::trajectory::UniqueSplinePtr tSpline1 = nullptr;
  if (direction1.norm() > 1e-2)
  {
    MetaSkeletonStateSaver saver1(mMetaSkeleton);
    mMetaSkeleton->setPositions(mOriginalConfig);

    auto originalState = mMetaSkeletonStateSpace->createState();
    mMetaSkeletonStateSpace->convertPositionsToState(
        mOriginalConfig, originalState);

    auto collisionConstraint = mAda->getArm()->getFullCollisionConstraint(
        tempSpace, mMetaSkeleton, nullptr);
    auto satisfiedConstraint
        = std::make_shared<aikido::constraint::Satisfied>(tempSpace);

    std::chrono::time_point<std::chrono::system_clock> startTime
        = std::chrono::system_clock::now();
    trajectory1 = aikido::planner::vectorfield::planToEndEffectorOffset(
        tempSpace,
        *originalState,
        mMetaSkeleton,
        mBodyNode,
        satisfiedConstraint,
        direction1.normalized(),
        0.0, // direction1.norm() - 0.001,
        direction1.norm() + 0.004,
        0.08,
        0.32,
        0.001,
        1e-3,
        1e-2,
        std::chrono::duration<double>(5));

    if (trajectory1 == nullptr)
      throw std::runtime_error("Failed in finding the first half");
    spline1 = dynamic_cast<aikido::trajectory::Spline*>(trajectory1.get());
  }

  Eigen::Vector3d direction2
      = goalPose.translation() - currentPose.translation();
  if (direction2.norm() < mGoalPrecision)
  {
    ROS_INFO("Visual servoing is finished because goal was position reached.");
    mExecutionDone = true;
    return nullptr;
  }

  aikido::trajectory::TrajectoryPtr trajectory2 = nullptr;
  aikido::trajectory::UniqueSplinePtr tSpline2;
  if (spline1)
  {
    MetaSkeletonStateSaver saver2(mMetaSkeleton);
    auto endState = tempSpace->createState();
    auto endTime = spline1->getEndTime();

    try
    {
      spline1->evaluate(spline1->getEndTime(), endState);
    }
    catch (const std::exception& e)
    {
      ROS_WARN(
          "Trajectory evaluation failed. Printing current config and "
          "direction:");
      ROS_WARN_STREAM(currentConfig);
      ROS_WARN_STREAM(direction1.matrix().transpose());
      ROS_WARN("Retrying...");
      ROS_WARN_STREAM(e.what());
      mExecutionDone = true;
      mNotFailed = false;
      return nullptr;
    }
    tempSpace->setState(mMetaSkeleton.get(), endState);

    auto collisionConstraint = mAda->getArm()->getFullCollisionConstraint(
        tempSpace, mMetaSkeleton, nullptr);
    auto satisfiedConstraint
        = std::make_shared<aikido::constraint::Satisfied>(tempSpace);

    aikido::planner::Planner::Result result;
    std::chrono::time_point<std::chrono::system_clock> startTime
        = std::chrono::system_clock::now();
    trajectory2 = aikido::planner::vectorfield::planToEndEffectorOffset(
        tempSpace,
        *endState,
        mMetaSkeleton,
        mBodyNode,
        satisfiedConstraint,
        direction2.normalized(),
        0.0, // std::min(direction2.norm(), 0.2) - 0.001,
        std::min(direction2.norm(), 0.0) + 0.1,
        0.01,
        0.04,
        0.001,
        1e-3,
        1e-3,
        std::chrono::duration<double>(5),
        &result);

    if (trajectory2 == nullptr)
    {
      std::cout << "RESULT : " << result.getMessage() << std::endl;
      throw std::runtime_error("Failed in finding the second half");
    }
    spline2 = dynamic_cast<aikido::trajectory::Spline*>(trajectory2.get());

    if (spline2 == nullptr)
      return nullptr;

    std::chrono::time_point<std::chrono::system_clock> timingStartTime
        = std::chrono::system_clock::now();
    auto concatenatedTraj = concatenate(*spline1, *spline2);
    if (concatenatedTraj == nullptr)
      return nullptr;

    auto timedTraj = computeKunzTiming(
        *concatenatedTraj, mMaxVelocity, mMaxAcceleration, 1e-2, 9e-3);

    if (timedTraj == nullptr)
      return nullptr;

    currentConfig = mMetaSkeleton->getPositions();
    double refTime
        = findTimeOfClosestStateOnTrajectory(*timedTraj.get(), currentConfig);

    auto partialTimedTraj = createPartialTrajectory(*timedTraj, refTime);

    for (int i = 0; i < indices.size(); ++i)
    {
      llimits(indices[i]) = -dart::math::constantsd::inf();
      ulimits(indices[i]) = dart::math::constantsd::inf();
    }
    mMetaSkeleton->setPositionLowerLimits(llimits);
    mMetaSkeleton->setPositionUpperLimits(ulimits);

    return std::move(partialTimedTraj);
  }
  else
  {
    aikido::trajectory::TrajectoryPtr trajectory2
        = mAda->planArmToEndEffectorOffset(
            direction2.normalized(), std::min(direction2.norm(), 0.2), nullptr);

    if (!hasOriginalDirection)
    {
      originalDirection = direction2.normalized();
      hasOriginalDirection = true;
    }

    if (trajectory2 == nullptr)
    {
      ROS_WARN(
          "Trajectory evaluation failed. Printing current config and "
          "direction:");
      ROS_WARN_STREAM(currentConfig);
      ROS_WARN_STREAM(direction2.matrix().transpose());
      ROS_WARN("Retrying...");
      mExecutionDone = true;
      mNotFailed = false;
      return nullptr;
    }

    spline2 = dynamic_cast<aikido::trajectory::Spline*>(trajectory2.get());
    if (spline2 == nullptr)
      return nullptr;

    auto timedTraj = computeKunzTiming(
        *spline2, mMaxVelocity, mMaxAcceleration, 1e-2, 3e-3);

    if (timedTraj == nullptr)
      return nullptr;

    return std::move(timedTraj);
  }
}

} // namespace feeding
