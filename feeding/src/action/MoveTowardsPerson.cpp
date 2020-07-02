#include "feeding/action/MoveTowardsPerson.hpp"

#include <libada/util.hpp>

#include "feeding/perception/Perception.hpp"
#include "feeding/perception/PerceptionServoClient.hpp"

namespace feeding {
namespace action {

bool moveTowardsPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    FeedingDemo* feedingDemo,
    double distanceFromPerson,
    double velocityLimit)
{
  ROS_INFO_STREAM("Move towards person");

  PerceptionServoClient servoClient(
      nodeHandle,
      std::bind(&Perception::perceiveFace, perception.get()),
      ada,
      ros::Duration(0.1), // Update trajectory at 10Hz
      ros::Duration(0.5), // Stop if we can't see the face
      distanceFromPerson,
      velocityLimit,
      feedingDemo->getFTThresholdHelper());

  auto future = servoClient.start();
  
  // Time-out after 10s
  std::future_status status = future.wait_for(std::chrono::seconds(10));
  if(status != std::future_status::ready) {
    // Cancel servoing
    servoClient.stop();
    ROS_WARN_STREAM("Servoing took too long.");
    return false;
  }

  auto result = future.get();
  return (result == ada::CartVelocityResult::kCVR_SUCCESS);
}
} // namespace action
} // namespace feeding
