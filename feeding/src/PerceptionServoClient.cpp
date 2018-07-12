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
    aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
    aikido::rviz::WorldInteractiveMarkerViewer& viewer,
    double perceptionUpdateTime,
    double goalPoseUpdateTolerance
  ) : mNode(node)
    , mPerception(perception)
    , mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
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
  ROS_INFO("Servoclient started");
  mExecutionDone = false;
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
  while(elapsedTime < timelimit && !mExecutionDone)
  {

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

  // check for exceptions (for example the controller aborted the trajectory)
  if (mExec.valid()) {
    if (mExec.wait_for(std::chrono::duration<int, std::milli>(0)) == std::future_status::ready) {
      try {
        mExec.get();
      } catch(const std::exception& e) {
        ROS_WARN_STREAM(e.what());
        mExecutionDone = true;
        return;
      }
    }
  }

  if(updatePerception(mGoalPose))
  {
    // when the difference between new goal pose and previous goal pose is too large
    if( computeGeodesicDistance(mGoalPose, mLastGoalPose, 1.0) > mGoalPoseUpdateTolerance )
    {
      ROS_INFO("Sending new Trajectory");
      mTrajectoryExecutor->abort();
      

      // TODO: check whether meta skeleton is automatically updated

      // Generate a new reference trajectory to the goal pose
      mCurrentTrajectory = planToGoalPose(mGoalPose);

      // Execute the new reference trajectory
      if(mCurrentTrajectory)
      {
        if (mCurrentTrajectory->getNumSegments() > 0) {
          // auto trajMarkerPointer = mViewer.addTrajectoryMarker(mCurrentTrajectory, mMetaSkeleton, *mBodyNode);
          // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        }
        if (mExec.valid())
          mExec.wait();
        mExec = mTrajectoryExecutor->execute(mCurrentTrajectory);
      }
      else
      {
        throw std::runtime_error("cannot find a feasible path");
      }
    } else {
      ROS_INFO("Food position didn't change much");
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
    Eigen::Isometry3d endEffectorTransform = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rotationMatrix(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    endEffectorTransform.linear() = rotationMatrix;
    bool successful = mPerception->perceiveFood(goalPose);
    goalPose = goalPose * endEffectorTransform;

    if (goalPose.translation().z() < 0.26) {
      ROS_WARN("Food STILL below table!");
      return false;
    }

    // dart::dynamics::SimpleFramePtr goalFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "goalFrame", goalPose);
    // mFrames.push_back(goalFrame);
    // mFrameMarkers.push_back(mViewer.addFrame(goalFrame.get(), 0.07, 0.007));
    //ROS_INFO_STREAM("Goal pose: " << goalPose.matrix());

    return successful;
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

  auto startState = mMetaSkeletonStateSpace->createState();
  auto goalState = mMetaSkeletonStateSpace->createState();
  Eigen::VectorXd startConfig = mMetaSkeleton->getPositions();
  mMetaSkeletonStateSpace->convertPositionsToState(startConfig, startState);

  // Get goal configuration using IK
  auto ik = InverseKinematics::create(mBodyNode.get());
  ik->setDofs(mMetaSkeleton->getDofs());

  ik->getTarget()->setTransform(goalPose);
  Eigen::VectorXd goalConfig;
  if(ik->solve(goalConfig, true))
  {
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), goalState);
  } else {
    ROS_INFO("unable to solve IK");
    return nullptr;
  }

  // use snap planner to create the path
  SnapConfigurationToConfigurationPlanner planner(mMetaSkeletonStateSpace);
  ConfigurationToConfiguration problem(mMetaSkeletonStateSpace, startState, goalState, nullptr /*mCollisionFreeConstraint*/);
  auto traj = planner.plan(problem);

  if(traj)
  {
    // time trajectory using parabolic timer
    // (trajectory is returned by snap planner)
    auto interpolated = dynamic_cast<const Interpolated*>(traj.get());
    if(interpolated)
    {
      return computeParabolicTiming(*interpolated, mMaxVelocity, mMaxAcceleration);
    } else {
      throw std::runtime_error("trajectory is not interpolated!");
    }
  }

  return nullptr;
}

} // namespace feeding
