#include <chrono>
#include "feeding/PerceptionServoClient.hpp"
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/planner/kinodynamic/KinodynamicTimer.hpp>
#include "feeding/util.hpp"

static std::string JOINT_STATE_TOPIC_NAME = "/joint_states";

namespace feeding {

namespace {

Eigen::VectorXd getSymmetricLimits(
    const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits)
{
  assert( static_cast<std::size_t>(lowerLimits.size())
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
    double goalPoseUpdateTolerance,
    const Eigen::VectorXd& veloctiyLimits)
  : mNode(node, "perceptionServo")
  , mGetTransform(getTransform)
  , mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mAdaMover(adaMover)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mBodyNode(bodyNode)
  , mTrajectoryExecutor(trajectoryExecutor)
  , mCollisionFreeConstraint(collisionFreeConstraint)
  , mPerceptionUpdateTime(perceptionUpdateTime)
  , mGoalPoseUpdateTolerance(goalPoseUpdateTolerance)
  , mCurrentTrajectory(nullptr)
{
  mNonRealtimeTimer = mNode.createTimer(
      ros::Duration(mPerceptionUpdateTime),
      &PerceptionServoClient::nonRealtimeCallback,
      this,
      false,
      false);

  // subscribe to the joint state publisher
  mSub = mNode.subscribe(JOINT_STATE_TOPIC_NAME, 10, &PerceptionServoClient::jointStateUpdateCallback, this);

  // initially set the current pose as the goal pose
  mGoalPose = mBodyNode->getTransform();
  mLastGoalPose = mGoalPose;

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
}

//==============================================================================
PerceptionServoClient::~PerceptionServoClient()
{
  if(mTrajectoryExecutor)
  {
    mTrajectoryExecutor->abort();
  }

  if (mExec.valid())
  {
    auto future_status = mExec.wait_for(std::chrono::duration<int, std::milli>(0));
  }

  mNonRealtimeTimer.stop();
  mSub.shutdown();
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
void PerceptionServoClient::jointStateUpdateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::lock_guard<std::mutex> currPosVelLock{mJointStateUpdateMutex, 
                                                 std::adopt_lock};
  std::size_t velDimSize = mCurrentVelocity.size();
  for(std::size_t i=0; i < velDimSize; i++)
  {
    mCurrentVelocity[i] = msg->velocity[i];
    mCurrentPosition[i] = msg->position[i];  
  }

  // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Joint States updated:   " << mCurrentVelocity[0] << ", "<< mCurrentVelocity[1] << ", "<< mCurrentVelocity[2]);
}

double PerceptionServoClient::getElapsedTime() {
  double elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(
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
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
  if (!timerMutex.try_lock()) {
    return;
  }
  using aikido::planner::vectorfield::computeGeodesicDistance;

  // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Update callback start");
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

  if (updatePerception(mGoalPose))
  {
    // when the difference between new goal pose and previous goal pose is too
    // large
    double angularDistanceRatio = 1.0; // ignore the angular difference
    double geodesicDistance = computeGeodesicDistance(mGoalPose, mLastGoalPose, angularDistanceRatio);
    //ROS_INFO_STREAM("CURRENT GEODESIC DISTANCE IS " << geodesicDistance);
    if(geodesicDistance > mGoalPoseUpdateTolerance || true)
    {
      // Generate a new reference trajectory to the goal pose
      auto start = std::chrono::steady_clock::now();
      // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Planning start");
      mCurrentTrajectory = planToGoalPose(mGoalPose);
      if (!mCurrentTrajectory && mExecutionDone) { return; }
      // Eigen::VectorXd desiredVec(6);
      // mCurrentTrajectory->evaluateDerivative(0, 1, desiredVec);
      // ROS_INFO_STREAM("Timed velocity: " << desiredVec[0] << ", " << desiredVec[1] << ", " << desiredVec[2]);
      // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Planning stop");

      // std::cout << "=======================================" << std::endl;
      // for(double t=mCurrentTrajectory->getStartTime();
      //     t <= mCurrentTrajectory->getEndTime(); t+=0.01)
      // {
      //   Eigen::VectorXd tmpStateVec(mCurrentTrajectory->getStateSpace()->getDimension());
      //   Eigen::VectorXd tmpVelocityVec(mCurrentTrajectory->getStateSpace()->getDimension());
      //   auto tmpState = mCurrentTrajectory->getStateSpace()->createState();
      //   mCurrentTrajectory->evaluate(t, tmpState);
      //   mCurrentTrajectory->getStateSpace()->logMap(tmpState, tmpStateVec);
      //   mCurrentTrajectory->evaluateDerivative(t, 1, tmpVelocityVec);
      //   std::cout << t << " ";
      //   for(std::size_t i=0; i<mCurrentTrajectory->getStateSpace()->getDimension();i++)
      //   {
      //     std::cout << tmpStateVec[i] << " " << tmpVelocityVec[i] << " ";
      //   }
      //   std::cout << std::endl;
      // }
      // std::cout << "============================================================" << std::endl;

      auto duration = std::chrono::duration_cast<std::chrono::milliseconds> 
                            (std::chrono::steady_clock::now() - start);
      ROS_INFO_STREAM("Planning took " << duration.count() << " millisecs");


      // ROS_INFO("Aborting old trajectory");

      start = std::chrono::steady_clock::now();
      mTrajectoryExecutor->abort();
      duration = std::chrono::duration_cast<std::chrono::milliseconds> 
                            (std::chrono::steady_clock::now() - start);
     //  ROS_INFO_STREAM("Abortion took " << duration.count() << " millisecs");
      // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Aborted old trajectory");


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

      // TODO: check whether meta skeleton is automatically updated
      // Execute the new reference trajectory
      if (mCurrentTrajectory)
      {
      // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Waiting for trajectory to finish aborting");
        if (mExec.valid())
          mExec.wait();
      // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Trajectory aborted completely, sending new trajectory");
        mExec = mTrajectoryExecutor->execute(mCurrentTrajectory);
      // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   New trajectory sent");
      }
      else
      {
        timerMutex.unlock();
        throw std::runtime_error("cannot find a feasible path");
      }
    }
    else
    {
      ROS_INFO("Food position didn't change much");
    }

    // updateGoalPose
    mLastGoalPose = mGoalPose;
  }
  // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Update callback stop");
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
  // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Waiting for perception");
  bool successful = mGetTransform(goalPose);
  // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Perception done");
  goalPose = goalPose * endEffectorTransform;
  if (goalPose.translation().z() < 0.215)
  {
    ROS_WARN_STREAM("Food STILL below table!   " << goalPose.matrix());
    return false;
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

  std::cout << "BEFORE PLANNING " << mCurrentVelocity.matrix().transpose() << std::endl;
  Eigen::VectorXd currentVelocities = mMetaSkeleton->getVelocities();

  {
    std::lock_guard<std::mutex> currPosVelLock{mJointStateUpdateMutex, 
                                                 std::adopt_lock};

    currentVelocities = mCurrentVelocity;
  }
  // ROS_INFO_STREAM("Goal pose: " << liftedGoalPose.translation().matrix());

  Eigen::Isometry3d startPose = mBodyNode->getTransform();
  Eigen::Vector3d difference = goalPose.translation() - startPose.translation();
  double length = std::min(difference.norm(), difference.norm());
  if (length < 0.002) {
    ROS_INFO("aborting because end position");
    mExecutionDone = true;
    return nullptr;
  }
  ROS_INFO_STREAM("planToGoalPose, length: " << length << ", norm: " << difference.norm());

  auto traj = mAdaMover->planToEndEffectorOffset(difference.normalized(), length);

  if (!traj) {
    ROS_WARN("Failed to find a solution!");
  }

  if (traj)
  {
    Eigen::VectorXd currentVelocities = mMetaSkeleton->getVelocities();

    {
      std::lock_guard<std::mutex> currPosVelLock{mJointStateUpdateMutex, 
                                                 std::adopt_lock};

      // ROS_INFO_STREAM("Time: " << getElapsedTime() << "    -   Copying joint velocities");
      currentVelocities = mCurrentVelocity;
    }
    // ROS_INFO("Timing new trajectory");
    // ROS_INFO_STREAM("Current velocities " << currentVelocities.transpose());
    /*
    Eigen::VectorXd endVelocities = Eigen::VectorXd::Zero(currentVelocities.rows(), currentVelocities.cols());

    // time trajectory using parabolic timer
    // (trajectory is returned by snap planner)
    auto interpolated = dynamic_cast<const Interpolated*>(traj.get());
    if (interpolated)
    {
      auto timedTraj = createTimedSplineTrajectory(
          *interpolated, currentVelocities, endVelocities, 
          mMaxVelocity, mMaxAcceleration);
      std::cout << "SUCCESSFULLY TIMED THE NEW TRAJ" << std::endl;
      return std::move(timedTraj);
    }
    else
    {
      throw std::runtime_error("trajectory is not interpolated!");
    }*/

    ROS_INFO_STREAM("Current velocities: " << currentVelocities);

    const Interpolated* interpolated = dynamic_cast<const Interpolated*>(traj.get());
    if (interpolated)
    {
      auto timedTraj = computeKinodynamicTiming(*interpolated,
		                                mMaxVelocity,
		                                mMaxAcceleration,
		                                currentVelocities);

      if(timedTraj)
      {
        Eigen::VectorXd initVel(mMaxVelocity.size());
        timedTraj->evaluateDerivative(0.0, 1, initVel);
       
        return std::move(timedTraj);
      }
    }
    else
    {
      const Spline* spline = dynamic_cast<const Spline*>(traj.get());
      if(spline)
      {
        auto timedTraj = computeKinodynamicTiming(*spline,
		                                mMaxVelocity,
		                                mMaxAcceleration,
		                                currentVelocities);
        if(timedTraj)
        {
          Eigen::VectorXd initVel(mMaxVelocity.size());
          timedTraj->evaluateDerivative(0.0, 1, initVel);
      
          return std::move(timedTraj);
        } 
      }
    }
  }

  return nullptr;
}

std::unique_ptr<aikido::trajectory::Spline> timeTrajectoryUsingConstantVelocity(
    const aikido::trajectory::TrajectoryPtr traj,
    std::vector<double>& constantVelocity)
{
  
  return nullptr;
}

} // namespace feeding
