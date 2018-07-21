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
    aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
    double perceptionUpdateTime,
    double goalPoseUpdateTolerance)
  : mNode(node, "perceptionServo")
  , mGetTransform(getTransform)
  , mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mAdaMover(adaMover)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mBodyNode(bodyNode)
  , mTrajectoryExecutor(trajectoryExecutor)
  , mCollisionFreeConstraint(collisionFreeConstraint)
  , mViewer(viewer)
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
  mMaxVelocity = getSymmetricLimits(velocityLowerLimits, velocityUpperLimits);
  mMaxAcceleration
      = getSymmetricLimits(accelerationLowerLimits, accelerationUpperLimits);

  mCurrentPosition = Eigen::VectorXd::Zero(mMetaSkeleton->getNumDofs());
  mCurrentVelocity = Eigen::VectorXd::Zero(mMetaSkeleton->getNumDofs());
}

//==============================================================================
PerceptionServoClient::~PerceptionServoClient()
{
  ROS_INFO("PerceptionServoClient::~PerceptionServoClient");
  if(mTrajectoryExecutor)
  {
    mTrajectoryExecutor->abort();
  }
  mNonRealtimeTimer.stop();
  mSub.shutdown();
  // DO NOTHING
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

//==============================================================================
void PerceptionServoClient::start()
{
  ROS_INFO("Servoclient started");
  mExecutionDone = false;
  mNonRealtimeTimer.start();
  mIsRunning = true;
}

//==============================================================================
void PerceptionServoClient::jointStateUpdateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::lock_guard<std::mutex> currPosVelLock{mJointStateUpdateMutex, 
                                                 std::adopt_lock};
  std::size_t velDimSize = msg->velocity.size();
  //std::cout << "jointStateUpdateCallback = SIZE " << velDimSize << " [";
  for(std::size_t i=0; i < velDimSize; i++)
  {
    mCurrentVelocity[i] = msg->velocity[i];
    mCurrentPosition[i] = msg->position[i];  
    //std::cout << msg->velocity[i] << " ";
  }
  //std::cout << std::endl;
}

//==============================================================================
void PerceptionServoClient::stop()
{
  // Always abort the executing trajectory when quitting
  mTrajectoryExecutor->abort();
  mNonRealtimeTimer.stop();
  mIsRunning = false;
  ROS_INFO("stopping done");
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
  ROS_INFO("waiting done");
}

//==============================================================================
bool PerceptionServoClient::isRunning()
{
  return mIsRunning;
}

//==============================================================================
void PerceptionServoClient::nonRealtimeCallback(const ros::TimerEvent& event)
{
  using aikido::planner::vectorfield::computeGeodesicDistance;

  if (mExecutionDone)
  {
    stop();
    return;
  }

  // check for exceptions (for example the controller aborted the trajectory)
  if (mExec.valid())
  {
    //ROS_INFO("mExec valid");
    if (mExec.wait_for(std::chrono::duration<int, std::milli>(0))
        == std::future_status::ready)
    {
      ROS_INFO("future is ready");
      try
      {
        mExec.get();
      }
      catch (const std::exception& e)
      {
        ROS_WARN_STREAM(e.what());
        mExecutionDone = true;
        stop();
        ROS_INFO("timer update aborted");
        return;
      }
    } else {
      ROS_INFO_STREAM("future status: " << std::to_string((uint8_t)mExec.wait_for(std::chrono::duration<int, std::milli>(0))));
    }
  }

  if (updatePerception(mGoalPose))
  {
    // when the difference between new goal pose and previous goal pose is too
    // large
    double angularDistanceRatio = 1.0; // ignore the angular difference
    double geodesicDistance = computeGeodesicDistance(mGoalPose, mLastGoalPose, angularDistanceRatio);
    //ROS_INFO_STREAM("CURRENT GEODESIC DISTANCE IS " << geodesicDistance);
    if(geodesicDistance > mGoalPoseUpdateTolerance )
    {

      // Generate a new reference trajectory to the goal pose
      auto start = std::chrono::steady_clock::now();
      mCurrentTrajectory = planToGoalPose(mGoalPose);
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds> 
                            (std::chrono::steady_clock::now() - start);
      ROS_INFO_STREAM("Planning took " << duration.count() << " millisecs");


 if (mExec.valid())
  {
    //ROS_INFO("mExec valid");
    if (mExec.wait_for(std::chrono::duration<int, std::milli>(0))
        == std::future_status::ready)
    {
      ROS_INFO("future is ready");
      try
      {
        mExec.get();
      }
      catch (const std::exception& e)
      {
        ROS_WARN_STREAM(e.what());
        mExecutionDone = true;
        stop();
        ROS_INFO("timer update aborted");
        return;
      }
    } else {
      ROS_INFO_STREAM("future status: " << std::to_string((uint8_t)mExec.wait_for(std::chrono::duration<int, std::milli>(0))));
    }
  }


      // ROS_INFO("Aborting old trajectory");
      start = std::chrono::steady_clock::now();
      mTrajectoryExecutor->abort();
      duration = std::chrono::duration_cast<std::chrono::milliseconds> 
                            (std::chrono::steady_clock::now() - start);
      // ROS_INFO_STREAM("Aborting took " << duration.count() << " millisecs");
      // TODO: check whether meta skeleton is automatically updated


      // Execute the new reference trajectory
      if (mCurrentTrajectory)
      {
        if (mCurrentTrajectory->getNumSegments() > 0)
        {
          // auto trajMarkerPointer =
          // mViewer.addTrajectoryMarker(mCurrentTrajectory, mMetaSkeleton,
          // *mBodyNode);
          // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        }
        if (mExec.valid())
          mExec.wait();
        // ROS_INFO("Starting trajectory execution");
        mExec = mTrajectoryExecutor->execute(mCurrentTrajectory);
      }
      else
      {
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
}

//==============================================================================
bool PerceptionServoClient::updatePerception(Eigen::Isometry3d& goalPose)
{
  // if (mGetTransform)
  // {
    // update new goal Pose
    Eigen::Isometry3d endEffectorTransform = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rotationMatrix(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    endEffectorTransform.linear() = rotationMatrix;
    bool successful = mGetTransform(goalPose);
    goalPose = goalPose * endEffectorTransform;

    if (goalPose.translation().z() < 0.26)
    {
      ROS_WARN("Food STILL below table!");
      return false;
    }

    // dart::dynamics::SimpleFramePtr goalFrame =
    // std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(),
    // "goalFrame", goalPose);
    // mFrames.push_back(goalFrame);
    // mFrameMarkers.push_back(mViewer.addFrame(goalFrame.get(), 0.07, 0.007));
    // ROS_INFO_STREAM("Goal pose: " << goalPose.matrix());

    return successful;
  // }
  // return false;
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

  Eigen::Vector3d goalTranslation(goalPose.translation().x(), goalPose.translation().y(), goalPose.translation().z());
  Eigen::Isometry3d liftedGoalPose = goalPose;
  liftedGoalPose.translation() = goalTranslation;

  // ROS_INFO_STREAM("Goal pose: " << liftedGoalPose.translation().matrix());

  Eigen::Isometry3d startPose = mBodyNode->getTransform();
  Eigen::Vector3d difference = liftedGoalPose.translation() - startPose.translation();
  // double length = std::min(0.01, difference.norm());
  double length = std::min(difference.norm(), 0.05);
  if (length < 0.002) {
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
        // std::cout << "INIT VEL IS " << initVel.matrix().transpose() << std::endl;
       
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
          // std::cout << "INIT VEL IS " << initVel.matrix().transpose() << std::endl;
      
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
