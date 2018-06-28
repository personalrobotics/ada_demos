#include <thread>
#include "aikido/control/ros/util.hpp"
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
}

GoalPoseExecutor::~GoalPoseExecutor()
{
}

std::future<void> GoalPoseExecutor::execute(const Eigen::Isometry3d& goalPose)
{
  bool waitForServer
      = aikido::control::ros::waitForActionServer<ada_demos::SetGoalPoseAction,
                            std::chrono::milliseconds,
                            std::chrono::milliseconds>(
          mClient,
          mCallbackQueue,
          mConnectionTimeout,
          mConnectionPollingPeriod);

}

}
