#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include "feeding/PerceptionServoClient.hpp"


namespace feeding {

namespace {

Eigen::VectorXd getSymmetricLimits(
    const Eigen::VectorXd& lowerLimits,
    const Eigen::VectorXd& upperLimits)
{
  assert(static_cast<std::size_t>(lowerLimits.size()) == static_cast<std::size_t>(upperLimits.size()));

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
    Perception* perception,
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::BodyNodePtr bodyNode,
    std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> trajectoryExecutor,
    double perceptionUpdateTime,
    double goalPoseUpdateTolerance
  ) : mNode(node)
    , mPerception(perception)
    , mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
    , mMetaSkeleton(std::move(metaSkeleton))
    , mBodyNode(bodyNode)
    , mTrajectoryExecutor(trajectoryExecutor)
    , mPerceptionUpdateTime(perceptionUpdateTime)
    , mGoalPoseUpdateTolerance(goalPoseUpdateTolerance)
    , mCurrentTrajectory(nullptr)
{
  mNonRealtimeTimer = mNode.createTimer(
    ros::Duration(mPerceptionUpdateTime), &PerceptionServoClient::nonRealtimeCallback,
    this, false, false);

  // initially set the current pose as the goal pose
  mGoalPose = mBodyNode->getTransform();
  mLastGoalPose = mGoalPose; 

  // update Max velocity and acceleration
  Eigen::VectorXd velocityLowerLimits = mMetaSkeleton->getVelocityLowerLimits();
  Eigen::VectorXd velocityUpperLimits = mMetaSkeleton->getVelocityUpperLimits();
  Eigen::VectorXd accelerationLowerLimits = mMetaSkeleton->getAccelerationLowerLimits();
  Eigen::VectorXd accelerationUpperLimits = mMetaSkeleton->getAccelerationUpperLimits();
  mMaxVelocity = getSymmetricLimits(velocityLowerLimits, velocityUpperLimits);
  mMaxAcceleration = getSymmetricLimits(accelerationLowerLimits, accelerationUpperLimits);
    
}

//==============================================================================
PerceptionServoClient::~PerceptionServoClient()
{
  // DO NOTHING
}


//==============================================================================
void PerceptionServoClient::start()
{
  mNonRealtimeTimer.start();
}

//==============================================================================
void PerceptionServoClient::stop()
{
  // Always abort the executing trajectory when quitting
  mTrajectoryExecutor->abort();
  mNonRealtimeTimer.stop();
}

bool PerceptionServoClient::wait(double timelimit)
{
  double elapsedTime = 0.0;
  std::chrono::time_point<std::chrono::system_clock> startTime
      = std::chrono::system_clock::now();
  while(elapsedTime < timelimit)
  {
    if(mExec.valid())
    {
      return true;
    }

    // sleep a while
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    elapsedTime = std::chrono::duration_cast<std::chrono::duration<double>>(
                      std::chrono::system_clock::now() - startTime)
                      .count();
  }
  
  stop();
  return false;
}

//==============================================================================
void PerceptionServoClient::nonRealtimeCallback(const ros::TimerEvent& event)
{
  using aikido::planner::vectorfield::computeGeodesicDistance;    

  if(updatePerception(mGoalPose))
  {
    // when the difference between new goal pose and previous goal pose is too large
    if( computeGeodesicDistance(mGoalPose, mLastGoalPose, 1.0) > mGoalPoseUpdateTolerance )
    {      
      mTrajectoryExecutor->abort();

      // TODO: check whether meta skeleton is automatically updated

      // Generate a new reference trajectory to the goal pose
      mCurrentTrajectory = planToGoalPose(mGoalPose);
        
      // Execute the new reference trajectory
      if(mCurrentTrajectory)
      {
        mExec = mTrajectoryExecutor->execute(mCurrentTrajectory);
      }
      else
      {
        throw std::runtime_error("cannot find a feasible path");
      }
    }  

    // updateGoalPose
    mLastGoalPose = mGoalPose;
  }
}

//==============================================================================
bool PerceptionServoClient::updatePerception(Eigen::Isometry3d& goalPose)
{
  if(mPerception)
  {
    // update new goal Pose
    return mPerception->perceiveFood(goalPose);
  }
  return false;
}

//==============================================================================
aikido::trajectory::SplinePtr PerceptionServoClient::planToGoalPose(const Eigen::Isometry3d& goalPose)
{  
  using dart::dynamics::InverseKinematics;
  using aikido::statespace::dart::MetaSkeletonStateSaver;
  using aikido::planner::ConfigurationToConfiguration;
  using aikido::planner::SnapConfigurationToConfigurationPlanner;
  using aikido::planner::parabolic::computeParabolicTiming;
  using aikido::trajectory::Interpolated;

  // Save the current state of the space 
  auto saver = MetaSkeletonStateSaver(mMetaSkeleton); 
  DART_UNUSED(saver);

  // Get goal configuration using IK
  auto ik = InverseKinematics::create(mBodyNode.get());
  ik->setDofs(mMetaSkeleton->getDofs());
  ik->getTarget()->setTransform(goalPose);
  
  auto startState = mMetaSkeletonStateSpace->createState();
  auto goalState = mMetaSkeletonStateSpace->createState();
  Eigen::VectorXd startConfig = mMetaSkeleton->getPositions();
  mMetaSkeletonStateSpace->convertPositionsToState(startConfig, startState);
  if(ik->solve(true))
  {
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), goalState);
  }

  // use snap planner to create the path
  SnapConfigurationToConfigurationPlanner planner(mMetaSkeletonStateSpace);
  ConfigurationToConfiguration problem(mMetaSkeletonStateSpace, startState, goalState, nullptr);
  auto traj = planner.plan(problem);

  if(traj)
  {
    // time trajectory using parabolic timer
    // (trajectory is returned by snap planner)
    auto interpolated = dynamic_cast<const Interpolated*>(traj.get());
    if(interpolated)
    {
      return computeParabolicTiming(*interpolated, mMaxVelocity, mMaxAcceleration);
    }
  } 

  return nullptr;
}

} // namespace feeding
