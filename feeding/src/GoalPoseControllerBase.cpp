#include <feeding/GoalPoseControllerBase.hpp>

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

using aikido::statespace::dart::MetaSkeletonStateSpace;

const static std::string SERVER_NAME = "set_goal_pose";

namespace feeding
{

//=============================================================================
GoalPoseControllerBase::GoalPoseControllerBase()
{
  using hardware_interface::EffortJointInterface;
  using hardware_interface::PositionJointInterface;
  using hardware_interface::VelocityJointInterface;

  mAdapterFactory.registerFactory<PositionJointInterface, rewd_controllers::JointPositionAdapter>(
      "position");
  mAdapterFactory.registerFactory<VelocityJointInterface, rewd_controllers::JointVelocityAdapter>(
      "velocity");
  mAdapterFactory.registerFactory<EffortJointInterface, rewd_controllers::JointEffortAdapter>(
      "effort");
}

//=============================================================================
GoalPoseControllerBase::~GoalPoseControllerBase() 
{
  // DO NOTHING
}

//=============================================================================
bool GoalPoseControllerBase::initController(
    hardware_interface::RobotHW* robot, ros::NodeHandle& n)
{
  mNodeHandle.reset(new ros::NodeHandle{n});

  using aikido::statespace::dart::MetaSkeletonStateSpace;
  using hardware_interface::JointStateInterface;

  // load the control type from paramter (position, velocity, effort)
  std::string control_type;
  if (!n.getParam("control_type", control_type)) {
    ROS_ERROR("Failed to load 'control_type' parameter.");
    return false;
  }
  if (control_type != "position" && control_type != "velocity" && control_type != "effort") {
    ROS_ERROR_STREAM("Invalid 'control_type' parameter. Must be 'position', 'velocity', or 'effort', but is " << control_type);
    return false;
  }

  // Build up the list of controlled DOFs.
  const auto jointParameters = rewd_controllers::loadJointsFromParameter(n, "joints", control_type);
  if (jointParameters.empty()) return false;

  ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
  for (const auto& param : jointParameters)
    ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

  // Load the URDF as a Skeleton.
  mSkeleton = rewd_controllers::loadRobotFromParameter(n, "robot_description_parameter");
  if (!mSkeleton) return false;

  // Check for zero-mass bodies that will be used incorrectly in calculations
  bool hasZeroMassBody = false;
  for (auto body : mSkeleton->getBodyNodes()) {
    if (body->getMass() <= 0.0) {
      ROS_ERROR_STREAM("Robot link '" << body->getName()
                                      << "' has mass = " << body->getMass());
      hasZeroMassBody = true;
    }
  }
  if (hasZeroMassBody) return false;  // TODO is this actually a problem?

  // Extract the subset of the Skeleton that is being controlled.
  mControlledSkeleton =
      getControlledMetaSkeleton(mSkeleton, jointParameters, "Controlled");
  if (!mControlledSkeleton) return false;

  mControlledSpace =
      std::make_shared<MetaSkeletonStateSpace>(mControlledSkeleton.get());

  // the full skeleton.
  const auto jointStateInterface = robot->get<JointStateInterface>();
  if (!jointStateInterface) {
    ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
    return false;
  }

  mSkeletonUpdater.reset(
      new rewd_controllers::SkeletonJointStateUpdater{mSkeleton, jointStateInterface});

  // Create adaptors to provide a uniform interface to different types.
  const auto numControlledDofs = mControlledSkeleton->getNumDofs();
  mAdapters.resize(numControlledDofs);

  for (size_t idof = 0; idof < numControlledDofs; ++idof) {
    const auto dof = mControlledSkeleton->getDof(idof);
    const auto param = jointParameters[idof];

    auto adapter = mAdapterFactory.create(param.mType, robot, dof);
    if (!adapter) return false;

    // Initialize the adapter using parameters stored on the parameter server.
    ros::NodeHandle adapterNodeHandle = rewd_controllers::createDefaultAdapterNodeHandle(n, dof);
    if (!adapter->initialize(adapterNodeHandle)) return false;

    mAdapters[idof] = std::move(adapter);
  }

  // Initialize buffers to avoid dynamic memory allocation at runtime.
  mDesiredPosition.resize(numControlledDofs);
  mDesiredVelocity.resize(numControlledDofs);
  mDesiredAcceleration.resize(numControlledDofs);
  mDesiredEffort.resize(numControlledDofs);

  // Start the action server. This must be last.
  using std::placeholders::_1;
  mActionServer.reset(new actionlib::ActionServer<Action>{
      n, SERVER_NAME,
      std::bind(&GoalPoseControllerBase::goalCallback, this, _1),
      false});
  mActionServer->start();

  mNonRealtimeTimer = n.createTimer(
      ros::Duration(0.02), &GoalPoseControllerBase::nonRealtimeCallback,
      this, false, false);

  return true;
}

//=============================================================================
void GoalPoseControllerBase::startController(const ros::Time& time)
{
  mSkeletonUpdater->update();

  // Hold the current position.
  mDesiredPosition = mControlledSkeleton->getPositions();
  mDesiredVelocity.setZero();
  mDesiredAcceleration.setZero();

  ROS_DEBUG_STREAM(
      "Initialized desired position: " << mDesiredPosition.transpose());
  ROS_DEBUG_STREAM(
      "Initialized desired velocity: " << mDesiredVelocity.transpose());
  ROS_DEBUG_STREAM(
      "Initialized desired acceleration: " << mDesiredAcceleration.transpose());

  // Reset any internal state in the adapters (e.g. integral windup).
  for (const auto& adapter : mAdapters) adapter->reset();

  ROS_DEBUG("Reset joint adapters.");
  mCurrentTrajectory.set(nullptr);

  mNonRealtimeTimer.start();
}

//=============================================================================
void GoalPoseControllerBase::stopController(const ros::Time& time)
{
  mNonRealtimeTimer.stop();
}

//=============================================================================
void GoalPoseControllerBase::updateStep(const ros::Time& time,
                                               const ros::Duration& period)
{
  // TODO: Make this a member variable to avoid dynamic allocation.
  auto mDesiredState = mControlledSpace->createState();

  std::shared_ptr<TrajectoryContext> context;
  mCurrentTrajectory.get(context);

  if (context && !context->mCompleted.load()) {
    const auto& trajectory = context->mTrajectory;
    const auto timeFromStart = std::min((time - context->mStartTime).toSec(),
                                        trajectory->getEndTime());

    // Evaluate the trajectory at the current time.
    trajectory->evaluate(timeFromStart, mDesiredState);
    mControlledSpace->convertStateToPositions(mDesiredState, mDesiredPosition);
    trajectory->evaluateDerivative(timeFromStart, 1, mDesiredVelocity);
    trajectory->evaluateDerivative(timeFromStart, 2, mDesiredAcceleration);

    // TODO: Check path constraints.
    // TODO: Check goal constraints.

    std::string stopReason;
    bool shouldStopExec = shouldStopExecution(stopReason);

    // Terminate the current trajectory.
    if (timeFromStart >= trajectory->getDuration()) {
      context->mCompleted.store(true);
    } else if (shouldStopExec || mCancelCurrentTrajectory.load()) {
      // TODO: if there is no other work that needs done here, we can get rid of
      // the cancel atomic_bool
      mDesiredVelocity.fill(0.0);
      mDesiredAcceleration.fill(0.0);
      context->mCompleted.store(true);

      if (shouldStopExec) {
        mAbortCurrentTrajectory.store(true);
        mAbortReason = stopReason;
      }
    }
  }

  // Update the state of the Skeleton.
  mSkeletonUpdater->update();
  mActualPosition = mControlledSkeleton->getPositions();
  mActualVelocity = mControlledSkeleton->getVelocities();
  mActualEffort = mControlledSkeleton->getForces();

  // Compute inverse dynamics torques from the set point and store them in the
  // skeleton. These values may be queried by the adapters below.
  mControlledSkeleton->setPositions(mDesiredPosition);
  mControlledSkeleton->setVelocities(mDesiredVelocity);
  mControlledSkeleton->setAccelerations(mDesiredAcceleration);

  mSkeleton->computeInverseDynamics();
  mDesiredEffort = mControlledSkeleton->getForces();

  // Restore the state of the Skeleton from JointState interfaces. These values
  // may be used by the adapters below.
  mControlledSkeleton->setPositions(mActualPosition);
  mControlledSkeleton->setVelocities(mActualVelocity);

  for (size_t idof = 0; idof < mAdapters.size(); ++idof) {
    mAdapters[idof]->update(time, period, mActualPosition[idof],
                            mDesiredPosition[idof], mActualVelocity[idof],
                            mDesiredVelocity[idof], mDesiredEffort[idof]);
  }
}

//=============================================================================
void GoalPoseControllerBase::goalCallback(GoalHandle goalHandle)
{
  const auto goal = goalHandle.getGoal();
  ROS_INFO_STREAM("Received trajectory "
                  << goalHandle.getGoalID().id);

  Eigen::Isometry3d goalPose;
  // set the goal pose and linear velocity
  // use tf::poseTFToEigen to isometry3d
  
  // create a trajectory from the current pose to the goal pose
  auto trajectory = createTrajectory(goalPose);

  // time the trajectory as the reference 
  const auto newContext = std::make_shared<TrajectoryContext>();
  newContext->mStartTime = ros::Time::now();
  newContext->mTrajectory = trajectory;
  newContext->mGoalHandle = goalHandle;

  std::lock_guard<std::mutex> newGoalLock{mNewGoalPoseRequestsMutex};
  mCurrentTrajectory.set(newContext);

  
}

//=============================================================================
aikido::trajectory::SplinePtr GoalPoseControllerBase::createTrajectory(
    Eigen::Isometry3d& goalPose)
{
  // create a trajectory in the metaSkeleton state space 
  // from the current configuration to a goal configuration defined by goal pose
  return nullptr;
}

//=============================================================================
void GoalPoseControllerBase::nonRealtimeCallback(
    const ros::TimerEvent& event)
{

}

//=============================================================================
bool GoalPoseControllerBase::shouldStopExecution(std::string& reason)
{ 
  return false; 
}

} // namespace feeding
