#ifndef FEEDING_GOALPOSEEXECUTOR_HPP_
#define FEEDING_GOALPOSEEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <mutex>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/action_client.h>
#include "ada_demos/SetGoalPoseAction.h"

namespace feeding
{
class GoalPoseExecutor
{
public:
  GoalPoseExecutor(::ros::NodeHandle node,
    const std::string& serverName,
    const std::chrono::milliseconds& connectionTimeout,
    const std::chrono::milliseconds& connectionPollingPeriod);
  ~GoalPoseExecutor();

  std::future<void> execute(const Eigen::Isometry3d& goalPose);

protected:
  using GoalPoseActionClient = actionlib::ActionClient<ada_demos::SetGoalPoseAction>;
  using GoalHandle = GoalPoseActionClient::GoalHandle;

  double mLinearVelocity; 
  Eigen::Isometry3d mGoalPose;
  void transitionCallback(GoalHandle handle);

  void nonRealtimeCallback(const ros::TimerEvent& event);

private:

  ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  GoalPoseActionClient mClient;
  GoalHandle mGoalHandle;
  ::ros::Timer mNonRealtimeTimer;

  std::chrono::milliseconds mConnectionTimeout;
  std::chrono::milliseconds mConnectionPollingPeriod;

  bool mInProgress;
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mInProgress, mPromise
  mutable std::mutex mMutex;
};

}  // namespace feeding

#endif  // FEEDING_GOALPOSEEXECUTOR_HPP_
