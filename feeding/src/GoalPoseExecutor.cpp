#include <thread>
#include "dart/dart.hpp"
#include "aikido/control/ros/util.hpp"
#include "aikido/control/ros/RosTrajectoryExecutionException.hpp"
#include "feeding/GoalPoseExecutor.hpp"

namespace feeding {

GoalPoseExecutor::GoalPoseExecutor(::ros::NodeHandle node,
    const std::string& serverName,
    const std::chrono::milliseconds& connectionTimeout,
    const std::chrono::milliseconds& connectionPollingPeriod)
  : mNode{std::move(node)}
  , mCallbackQueue{}
  , mClient{mNode, serverName, &mCallbackQueue}
  , mConnectionTimeout{connectionTimeout}
  , mConnectionPollingPeriod{connectionPollingPeriod}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  // DO NOTHING
}

//==============================================================================
GoalPoseExecutor::~GoalPoseExecutor()
{
  // DO NOTHING
}

//==============================================================================
std::future<void> GoalPoseExecutor::execute(const Eigen::Isometry3d& goalPose)
{

  ada_demos::SetGoalPoseGoal goal;

  bool waitForServer
      = aikido::control::ros::waitForActionServer<ada_demos::SetGoalPoseAction,
                            std::chrono::milliseconds,
                            std::chrono::milliseconds>(
          mClient,
          mCallbackQueue,
          mConnectionTimeout,
          mConnectionPollingPeriod);

  if (!waitForServer)
    throw std::runtime_error("Unable to connect to action server.");

  // keep the connection till 

  // send the goal to the action server
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning

  
  if (mInProgress)
    throw std::runtime_error("Executor is running");
  
  mPromise.reset(new std::promise<void>());
  mInProgress = true;
  mGoalHandle = mClient.sendGoal(
      goal,
      boost::bind(&GoalPoseExecutor::transitionCallback, this, _1));

  // start a timer to update goal pose
  mNonRealtimeTimer = mNode.createTimer(
      ros::Duration(0.02), &GoalPoseExecutor::nonRealtimeCallback,
      this, false, false);
  mNonRealtimeTimer.start();

  return mPromise->get_future(); 
}

//==============================================================================
void GoalPoseExecutor::transitionCallback(GoalHandle handle)
{
  std::cout << "GoalPoseExecutor::transitionCallback" << std::endl;

  // This function assumes that mMutex is locked.

  using GoalPoseExecutionException = aikido::control::ros::RosTrajectoryExecutionException;
  using actionlib::TerminalState;
  using Result = ada_demos::SetGoalPoseAction;

  if (handle.getCommState() == actionlib::CommState::DONE)
  {
    std::stringstream message;
    bool isSuccessful = true;

    // Check the status of the actionlib call. Note that the actionlib call can
    // succeed, even if execution failed.
    const auto terminalState = handle.getTerminalState();
    if (terminalState != TerminalState::SUCCEEDED)
    {
      message << "Action " << terminalState.toString();

      const auto terminalMessage = terminalState.getText();
      if (!terminalMessage.empty())
        message << " (" << terminalMessage << ")";

      mPromise->set_exception(
          std::make_exception_ptr(
              GoalPoseExecutionException(message.str(), terminalState)));

      isSuccessful = false;
    }
    else
    {
      message << "Execution failed.";
    }

    // Check the status of execution. This is only possible if the actionlib
    // call succeeded.
    const auto result = handle.getResult();
    if (result && result->success == false)
    {

      mPromise->set_exception(
          std::make_exception_ptr( std::runtime_error(message.str()) ) );
      isSuccessful = false;
    }

    if (isSuccessful)
    {
      mPromise->set_value();
      // stop the timer
      mNonRealtimeTimer.stop();
    }

    mInProgress = false;
  }  
}

//==============================================================================
void GoalPoseExecutor::nonRealtimeCallback(const ros::TimerEvent& event)
{
  // if the current goal is different, send a new goal
  bool sendGoal = false;

  std::cout << "GoalPoseExecutor::nonRealtimeCallback" << std::endl;

  if(sendGoal)
  {
    ada_demos::SetGoalPoseGoal goal;
    mGoalHandle = mClient.sendGoal(
      goal,
      boost::bind(&GoalPoseExecutor::transitionCallback, this, _1));
  }
}

}
