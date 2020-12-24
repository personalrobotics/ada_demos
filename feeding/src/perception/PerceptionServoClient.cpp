#include "feeding/perception/PerceptionServoClient.hpp"

#include <chrono>

#include <libada/util.hpp>

#include "feeding/util.hpp"

namespace feeding {

using CartVelocityResult = ada::CartVelocityResult;

//==============================================================================
PerceptionServoClient::PerceptionServoClient(
      const ::ros::NodeHandle* node,
      std::function<std::unique_ptr<Eigen::Isometry3d>(void)> getPerception,
      std::shared_ptr<ada::Ada> ada,
      ros::Duration perceptionUpdateTime,
      ros::Duration perceptionTimeout,
      double goalPrecision,
      double approachVelocity,
      std::shared_ptr<FTThresholdHelper> ftThresholdHelper)
  : mNodeHandle(*node, "perceptionServo")
  , mGetPerception(getPerception)
  , mAda(std::move(ada))
  , mExecutionDone(true)
  , mWatchdogFed(false)
  , mPromise(nullptr)
  , mGoalPrecision(goalPrecision)
  , mApproachVelocity(approachVelocity)
  , mFTThresholdHelper(ftThresholdHelper)
{
  mNonRealtimeTimer = mNodeHandle.createTimer(
      perceptionUpdateTime,
      &PerceptionServoClient::nonRealtimeCallback,
      this,
      false,
      false);

  mNonRealtimeWatchdog = mNodeHandle.createTimer(
      perceptionTimeout,
      &PerceptionServoClient::nonRealtimeWatchdog,
      this,
      false,
      false);
}

//==============================================================================
PerceptionServoClient::~PerceptionServoClient()
{
  stop();
  mNonRealtimeTimer.stop();
  mNonRealtimeWatchdog.stop();
}

//==============================================================================
std::future<CartVelocityResult> PerceptionServoClient::start()
{
  if(!mExecutionDone.load()) {
    ROS_WARN_STREAM("Cancelling previous ServoClient");
    stop();
  }

  mPromise.reset(new std::promise<CartVelocityResult>());
  // Invalid until first velocity command
  mFuture = std::future<CartVelocityResult>();
  std::future<CartVelocityResult> ret = mPromise->get_future();
  mWatchdogFed.store(true);

  if (!mAda->setVelocityControl(true, mFTThresholdHelper.get())) {
    mPromise->set_value(CartVelocityResult::kCVR_INVALID);
    return ret;
  }

  // Swap to proper ftThresholdServer
  if (mFTThresholdHelper) {
    mFTThresholdHelper->swapTopic(
      "/move_until_touch_cartvel_controller/set_forcetorque_threshold", 
      true);
  }

  mExecutionDone.store(false);
  mNonRealtimeTimer.start();
  mNonRealtimeWatchdog.start();
  ROS_INFO("Servoclient started");

  return ret;
}

//==============================================================================
void PerceptionServoClient::stop(CartVelocityResult result)
{
  // Only stop if we are running
  if(!mExecutionDone.exchange(true)) {
    ROS_INFO("Stopping ServoClient");

    mAda->setVelocityControl(false, mFTThresholdHelper.get());

    // Swap to proper ftThresholdServer
    if (mFTThresholdHelper) {
      mFTThresholdHelper->swapTopic("", true);
    }

    mPromise->set_value(result);
  }
}

//==============================================================================
void PerceptionServoClient::nonRealtimeWatchdog(const ros::TimerEvent& event) {
  // If the watchdog hasn't been fed by perception
  // Stop the visual servoing
  if(!mWatchdogFed.exchange(false)) {
    ROS_WARN("ServoClient: Watchdog timed out.");
    stop(CartVelocityResult::kCVR_TIMEOUT);
  }
}

//==============================================================================
void PerceptionServoClient::nonRealtimeCallback(const ros::TimerEvent& event)
{
  // If Execution is Done, don't continue
  if(mExecutionDone.load()) return;

  // Pre-perception velocity command check
  std::future_status status;
  if(mFuture.valid()) {
    status = mFuture.wait_for(std::chrono::milliseconds(1));
    if (status == std::future_status::ready) {
      auto result = mFuture.get();
      if (result != CartVelocityResult::kCVR_SUCCESS) {
        ROS_WARN_STREAM("Velocity command returned non-success: " << result);
        stop(result);
        return;
      }
    } // mFuture ready
  } // mFuture valid

  // Run perception
  Eigen::Isometry3d goalPose;
  auto ret = mGetPerception();
  if (ret == nullptr) {
    ROS_WARN_STREAM("ServoClient: No perception.");
    return;
  }
  goalPose = *ret;

  // Feed watchdog
  mWatchdogFed.store(true);

  // Set up velocity command
  Eigen::Isometry3d currentPose = mAda->getHand()->getEndEffectorBodyNode()->getTransform();
  Eigen::Vector3d vectorToGoalPose
      = goalPose.translation() - currentPose.translation();

  if (vectorToGoalPose.norm() < mGoalPrecision)
  {
    ROS_INFO_STREAM("ServoClient Done, reached target.");
    stop(CartVelocityResult::kCVR_SUCCESS);
  }

  // Rescale by velocity
  vectorToGoalPose = vectorToGoalPose / vectorToGoalPose.norm() * mApproachVelocity;


  // Check if previous velocity command failed
  // If so, stop visual servoing
  if(mFuture.valid()) {
    status = mFuture.wait_for(std::chrono::milliseconds(1));
    if (status == std::future_status::ready) {
      auto result = mFuture.get();
      if (result != CartVelocityResult::kCVR_SUCCESS) {
        ROS_WARN_STREAM("Velocity command returned non-success: " << result);
        stop(result);
        return;
      }
    } // mFuture ready
  } // mFuture valid

  // Execute new velocity command
  // TODO: have duration be 1s or watchdog, whichever is longer
  mFuture = mAda->moveArmCommandVelocity(vectorToGoalPose, Eigen::Vector3d(0.0, 0.0, 0.0), ros::Duration(10.0));
}

} // namespace feeding
