#include "feeding/PerceptionServoClient.hpp"
#include <chrono>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/kinodynamic/KinodynamicTimer.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include "feeding/util.hpp"

static std::string JOINT_STATE_TOPIC_NAME = "/joint_states";

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
    AdaMover* adaMover,
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
  , mAdaMover(adaMover)
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
    mTrajectoryExecutor->abort();
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
  mStartTime = std::chrono::system_clock::now();
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
  // Always abort the executing trajectory when quitting
  mTrajectoryExecutor->abort();
  mNonRealtimeTimer.stop();
  mIsRunning = false;
  timerMutex.unlock();
}

void PerceptionServoClient::wait(double timelimit)
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

  // check for exceptions (for example the controller aborted the trajectory)
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
    // Generate a new reference trajectory to the goal pose
    auto planningStartTime = std::chrono::steady_clock::now();
    mCurrentTrajectory = planToGoalPose(goalPose);
    if (!mCurrentTrajectory && mExecutionDone)
    {
    timerMutex.unlock();
      return;
    }
    auto planningDuration
        = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - planningStartTime);
    // ROS_INFO_STREAM("Planning took " << planningDuration.count() << " millisecs");

    // ROS_INFO("Aborting old trajectory");
    mTrajectoryExecutor->abort();

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
    // ROS_INFO("Food position didn't change much");
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
  goalPose = goalPose * endEffectorTransform;
  ROS_INFO_STREAM("goal pose: " << goalPose.translation().transpose().matrix());
  if (goalPose.translation().z() < 0.215)
  {
    ROS_WARN_STREAM("Food is way too low");
    return false;
  }

  if (hasOriginalDirection) {
    goalPose.translation() = goalPose.translation() + originalDirection * mOriginalDirectionExtension;
  }
  return successful;
}

//==============================================================================
aikido::trajectory::SplinePtr PerceptionServoClient::planToGoalPose(
    const Eigen::Isometry3d& goalPose)
{
  using dart::dynamics::InverseKinematics;
  using aikido::statespace::dart::MetaSkeletonStateSaver;
  using aikido::planner::ConfigurationToConfiguration;
  using aikido::planner::SnapConfigurationToConfigurationPlanner;
  using aikido::planner::kinodynamic::computeKinodynamicTiming;
  using aikido::trajectory::Interpolated;
  using aikido::trajectory::Spline;

  Eigen::Isometry3d currentPose = mBodyNode->getTransform();

  aikido::trajectory::Spline* spline1 = nullptr;
  aikido::trajectory::Spline* spline2 = nullptr;

  Eigen::VectorXd currentConfig = mMetaSkeleton->getPositions();
  Eigen::Vector3d direction1
      = currentPose.translation() - mOriginalPose.translation();


  aikido::trajectory::TrajectoryPtr trajectory1 = nullptr;
  if (direction1.norm() > 1e-2)
  {
    MetaSkeletonStateSaver saver1(mMetaSkeleton);
    mMetaSkeleton->setPositions(mOriginalConfig);

    // using vectorfield planner directly because Ada seems to update the state
    // otherwise
    // trajectory1 = mAdaMover->planToEndEffectorOffset(direction1.normalized(),
    // direction1.norm());
    auto originalState = mMetaSkeletonStateSpace->createState();
    mMetaSkeletonStateSpace->convertPositionsToState(
        mOriginalConfig, originalState);

    // ROS_INFO_STREAM("Servoing plan to end effector offset 1 state: " << mMetaSkeleton->getPositions().matrix().transpose());
    // ROS_INFO_STREAM("Servoing plan to end effector offset 1 direction: " << direction1.normalized().matrix().transpose() << ",  length: " << direction1.norm());

    auto collisionConstraint = mAdaMover->mAda.getArm()->getFullCollisionConstraint(mMetaSkeletonStateSpace, mMetaSkeleton, nullptr);
    auto satisfiedConstraint = std::make_shared<aikido::constraint::Satisfied>(mMetaSkeletonStateSpace);

    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    trajectory1 = aikido::planner::vectorfield::planToEndEffectorOffset(
        *originalState,
        mMetaSkeletonStateSpace,
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
    // ROS_INFO_STREAM("Planning 1 took " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - startTime).count());

    if (trajectory1 == nullptr)
      throw std::runtime_error("Failed in finding the first half");
    spline1 = dynamic_cast<aikido::trajectory::Spline*>(trajectory1.get());
  }

  Eigen::Vector3d direction2 = goalPose.translation() - currentPose.translation();
  if (direction2.norm() < mGoalPrecision)
  {
    ROS_INFO("Visual servoing is finished because goal was position reached.");
    mExecutionDone = true;
    return nullptr;
  }

  aikido::trajectory::TrajectoryPtr trajectory2 = nullptr;
  if (spline1)
  {
    MetaSkeletonStateSaver saver2(mMetaSkeleton);
    auto endState = mMetaSkeletonStateSpace->createState();
    auto endTime = spline1->getEndTime();
    spline1->evaluate(spline1->getEndTime(), endState);
    mMetaSkeletonStateSpace->setState(mMetaSkeleton.get(), endState);
  
    auto collisionConstraint
        = mAdaMover->mAda.getArm()->getFullCollisionConstraint(mMetaSkeletonStateSpace, mMetaSkeleton, nullptr);
    auto satisfiedConstraint = std::make_shared<aikido::constraint::Satisfied>(mMetaSkeletonStateSpace);

    aikido::planner::Planner::Result result;
    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    trajectory2 = aikido::planner::vectorfield::planToEndEffectorOffset(
        *endState,
        mMetaSkeletonStateSpace,
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
    // ROS_INFO_STREAM("Planning 2 took " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - startTime).count());

    if (trajectory2 == nullptr)
    {
      std::cout << "RESULT : " << result.getMessage() << std::endl;
      throw std::runtime_error("Failed in finding the second half");
    }
    spline2 = dynamic_cast<aikido::trajectory::Spline*>(trajectory2.get());
    if (spline2 == nullptr)
      return nullptr;


    std::chrono::time_point<std::chrono::system_clock> timingStartTime = std::chrono::system_clock::now();
    auto concatenatedTraj = concatenate(*spline1, *spline2);
    if (concatenatedTraj == nullptr)
      return nullptr;

    auto timedTraj = computeKinodynamicTiming(
        *concatenatedTraj, mMaxVelocity, mMaxAcceleration, 1e-2, 9e-3);

    if (timedTraj == nullptr)
      return nullptr;

    currentConfig = mMetaSkeleton->getPositions();
    double refTime
        = findClosetStateOnTrajectory(timedTraj.get(), currentConfig);

    auto partialTimedTraj = createPartialTrajectory(*timedTraj, refTime);
    // ROS_INFO_STREAM("Timing took " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - timingStartTime).count());

    return std::move(partialTimedTraj);
  }
  else
  {
    aikido::trajectory::TrajectoryPtr trajectory2 = mAdaMover->planToEndEffectorOffset(
        direction2.normalized(), 
        std::min(direction2.norm(), 0.2), false);

    if (!hasOriginalDirection) {
      originalDirection = direction2.normalized();
      hasOriginalDirection = true;
    }

    if (trajectory2 == nullptr)
      throw std::runtime_error("Failed in finding the traj");

    spline2 = dynamic_cast<aikido::trajectory::Spline*>(trajectory2.get());
    if (spline2 == nullptr)
      return nullptr;

    auto timedTraj = computeKinodynamicTiming(
        *spline2, mMaxVelocity, mMaxAcceleration, 1e-2, 3e-3);

    if (timedTraj == nullptr)
      return nullptr;

    return std::move(timedTraj);
  } 
}

} // namespace feeding
