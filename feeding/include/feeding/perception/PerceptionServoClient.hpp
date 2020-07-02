#ifndef FEEDING_PERCEPTIONSERVOCLIENT_HPP_
#define FEEDING_PERCEPTIONSERVOCLIENT_HPP_

#include <mutex>
#include <ros/ros.h>

#include <libada/Ada.hpp>

#include "feeding/FTThresholdHelper.hpp"

namespace feeding {

class PerceptionServoClient
{
public:
  using CartVelocityResult = ada::CartVelocityResult;

  PerceptionServoClient(
      const ::ros::NodeHandle* node,  // For creating timer
      std::function<std::unique_ptr<Eigen::Isometry3d>(void)> getPerception, // Actual perception function
      std::shared_ptr<ada::Ada> ada, // Robot pointer, used for sending velocity commands
      ros::Duration perceptionUpdateTime, // How often to run perception 
      ros::Duration perceptionTimeout, // How long to keep running without perception
      double goalPrecision, // In m, defines sphere around goal point
      double approachVelocity, // In m/s, how fast to approach goal
      std::shared_ptr<FTThresholdHelper> ftThresholdHelper); // Set to use FT sensor

  virtual ~PerceptionServoClient();

  std::future<CartVelocityResult> start();

  void stop(CartVelocityResult result = CartVelocityResult::kCVR_CANCELLED);

protected:

  void nonRealtimeCallback(const ros::TimerEvent& event);

  void nonRealtimeWatchdog(const ros::TimerEvent& event);

  // Perception Update Loop
  ros::Timer mNonRealtimeTimer;

  // Watchdog loop
  ros::Timer mNonRealtimeWatchdog;

  // Execution variables
  std::atomic<bool> mExecutionDone;
  std::atomic<bool> mWatchdogFed;
  std::shared_ptr<std::promise<CartVelocityResult>> mPromise;
  std::future<CartVelocityResult> mFuture;

  // Mutex between update loop
  std::mutex mTimerMutex;

  // Robot pointer
  std::shared_ptr<ada::Ada> mAda;

  // See Constructor Doc
  ::ros::NodeHandle mNodeHandle;
  std::function<std::unique_ptr<Eigen::Isometry3d>(void)> mGetPerception;
  double mGoalPrecision;
  double mApproachVelocity;
  std::shared_ptr<FTThresholdHelper> mFTThresholdHelper;
};
} // namespace feeding

#endif // FEEDING_PERCEPTIONSERVOCLIENT_HPP_
