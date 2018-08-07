#include <chrono>
#include "feeding/PerceptionServoClient.hpp"
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/planner/kinodynamic/KinodynamicTimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/constraint/Satisfied.hpp>
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

  mOriginalPose = mBodyNode->getTransform();
  mOriginalConfig = mMetaSkeleton->getPositions();
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
      //     t <= mCurrentTrajectory->getEndTime(); t+=0.04)
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

      ROS_INFO("Aborting old trajectory");

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
  Eigen::Isometry3d currentPose = mBodyNode->getTransform();

  bool useOldPlanning = false;
  if (useOldPlanning) {
    Eigen::Vector3d difference = goalPose.translation() - currentPose.translation();
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
        currentVelocities = mCurrentVelocity;
      }
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
  } else {

    aikido::trajectory::Spline* spline1 = nullptr;
    aikido::trajectory::Spline* spline2 = nullptr;
    aikido::trajectory::TrajectoryPtr trajectory1;
    aikido::trajectory::TrajectoryPtr trajectory2;

    Eigen::VectorXd currentConfig = mMetaSkeleton->getPositions();

    ROS_INFO_STREAM("original: " << mOriginalPose.matrix());
    ROS_INFO_STREAM("current:  " << currentPose.matrix());
    ROS_INFO_STREAM("goal:     " << goalPose.matrix());
    ROS_INFO_STREAM("configuration: " << currentConfig.matrix().transpose());

    Eigen::Vector3d direction1 = currentPose.translation() - mOriginalPose.translation();

    if(direction1.norm()>1e-2)
    {
      ROS_INFO_STREAM("current config: " << mMetaSkeleton->getPositions().matrix().transpose());
      MetaSkeletonStateSaver saver1(mMetaSkeleton);
      mMetaSkeleton->setPositions(mOriginalConfig);
      ROS_INFO_STREAM("current config after setting: " << mMetaSkeleton->getPositions().matrix().transpose());
      auto planning1StartTime = std::chrono::steady_clock::now();
      
      
      // trajectory1 = mAdaMover->planToEndEffectorOffset(direction1.normalized(), direction1.norm());
      auto originalState = mMetaSkeletonStateSpace->createState();
      mMetaSkeletonStateSpace->convertPositionsToState(mOriginalConfig, originalState);
      auto satisfiedConstraint = std::make_shared<aikido::constraint::Satisfied>(mMetaSkeletonStateSpace);
      trajectory1 = aikido::planner::vectorfield::planToEndEffectorOffset(
        *originalState,
        mMetaSkeletonStateSpace,
        mMetaSkeleton,
        mBodyNode,
        satisfiedConstraint,
        direction1.normalized(),
        direction1.norm() - 0.001, direction1.norm() + 0.004,
        0.01, 0.04,
        0.001,
        1e-3, 1e-3,
        std::chrono::duration<double>(5)
      );
      
      
      ROS_INFO_STREAM("current config after planning: " << mMetaSkeleton->getPositions().matrix().transpose());
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                        (std::chrono::steady_clock::now() - planning1StartTime);
      ROS_INFO_STREAM("Planning 1 took " << duration.count() << " millisecs");
      if(trajectory1==nullptr)
        throw std::runtime_error("Failed in finding the first half");
      spline1 = dynamic_cast<aikido::trajectory::Spline*>(trajectory1.get());
    }

    if (spline1) {
      ROS_INFO_STREAM("original config: " << mOriginalConfig.matrix().transpose());
      auto originalState = spline1->getStateSpace()->createState();
      spline1->evaluate(0, originalState);
      Eigen::VectorXd originalStateVec(spline1->getStateSpace()->getDimension());
      spline1->getStateSpace()->logMap(originalState, originalStateVec);
      ROS_INFO_STREAM("Config at beginning of direction 1 trajectory: " << originalStateVec.matrix().transpose());
    }


    Eigen::Vector3d direction2 = goalPose.translation() - currentPose.translation();
    if (direction2.norm() < 0.002) {
      ROS_INFO("aborting because already near goal position");
      mExecutionDone = true;
      return nullptr;
    }

    ROS_INFO_STREAM("direction1: " << direction1.normalized() << "    " << direction1.norm());
    ROS_INFO_STREAM("direction2: " << direction2.normalized() << "    " << direction2.norm());

    {
      MetaSkeletonStateSaver saver2(mMetaSkeleton);
      auto planning2StartTime = std::chrono::steady_clock::now();
      if (spline1) {
        auto endState = mMetaSkeletonStateSpace->createState();
        spline1->evaluate(spline1->getEndTime(), endState);

        auto satisfiedConstraint = std::make_shared<aikido::constraint::Satisfied>(mMetaSkeletonStateSpace);
        trajectory2 = aikido::planner::vectorfield::planToEndEffectorOffset(
          *endState,
          mMetaSkeletonStateSpace,
          mMetaSkeleton,
          mBodyNode,
          satisfiedConstraint,
          direction2.normalized(),
          direction2.norm() - 0.001, direction2.norm() + 0.004,
          0.01, 0.04,
          0.001,
          1e-3, 1e-3,
          std::chrono::duration<double>(5)
        );

        if(trajectory2==nullptr)
          throw std::runtime_error("Failed in finding the second half");
      } else {
        trajectory2 = mAdaMover->planToEndEffectorOffset(direction2.normalized(), direction2.norm());
        if(trajectory2==nullptr)
          throw std::runtime_error("Failed in finding the traj");
      }
      
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                        (std::chrono::steady_clock::now() - planning2StartTime);
      ROS_INFO_STREAM("Planning 2 took " << duration.count() << " millisecs");
      
      spline2 = dynamic_cast<aikido::trajectory::Spline*>(trajectory2.get());
    }

    if(spline2==nullptr)
      return nullptr;

    if(spline1)
    {
      auto concatenatedTraj = concatenate(*spline1, *spline2);
      if(concatenatedTraj==nullptr)
        return nullptr;

      dumpSplinePhasePlot(*spline1, "spline1.txt", 0.01);
      dumpSplinePhasePlot(*spline2, "spline2.txt", 0.01);
      dumpSplinePhasePlot(*concatenatedTraj, "concatenated.txt", 0.01);

      auto timingStartTime = std::chrono::steady_clock::now();
      auto timedTraj = computeKinodynamicTiming(*concatenatedTraj,
	                                              mMaxVelocity,
		                                            mMaxAcceleration,
                                      1e-2,
                                      3e-3);

      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>
                        (std::chrono::steady_clock::now() - timingStartTime);
      ROS_INFO_STREAM("Timing concatenated trajectories took " << duration.count() << " millisecs");

      if(timedTraj==nullptr)
        return nullptr;

      dumpSplinePhasePlot(*timedTraj, "timed.txt", 0.01);



      auto findingClosestStartTime = std::chrono::steady_clock::now();
      double refTime = findClosetStateOnTrajectory(timedTraj.get(),
                                                   currentConfig);
      auto findingClosestDuration = std::chrono::duration_cast<std::chrono::milliseconds>
                      (std::chrono::steady_clock::now() - findingClosestStartTime);
      ROS_INFO_STREAM("current config: " << currentConfig.matrix().transpose());
      ROS_INFO_STREAM("Finding closest state took " << findingClosestDuration.count() << " millisecs");
      



      auto partialTrajectoryStartTime = std::chrono::steady_clock::now();
      auto partialTimedTraj = createPartialTrajectory(*timedTraj, refTime);
      auto partialTrajectoryDuration = std::chrono::duration_cast<std::chrono::milliseconds>
                      (std::chrono::steady_clock::now() - partialTrajectoryStartTime);
      ROS_INFO_STREAM("Creating partial trajectory took " << partialTrajectoryDuration.count() << " millisecs");



      auto tempState = partialTimedTraj->getStateSpace()->createState();
      partialTimedTraj->evaluate(0, tempState);
      Eigen::VectorXd tempStateVec(partialTimedTraj->getStateSpace()->getDimension());
      partialTimedTraj->getStateSpace()->logMap(tempState, tempStateVec);
      ROS_INFO_STREAM("Config at partial trajectory start: " << tempStateVec.matrix().transpose());
      

      dumpSplinePhasePlot(*partialTimedTraj, "partialTimed.txt", 0.01);
      return std::move(partialTimedTraj);
    }
    else
    {
      // dumpSplinePhasePlot(*spline2, "spline2.txt", 0.01);
      
      auto timedTraj = computeKinodynamicTiming(*spline2,
                                                mMaxVelocity,
                                                mMaxAcceleration,
                                      1e-2,
                                      3e-3);

      if(timedTraj==nullptr)
        return nullptr;

      // dumpSplinePhasePlot(*timedTraj, "timed.txt", 0.01);

      return std::move(timedTraj);
    }
  }
}



} // namespace feeding
