#include "feeding/action/MoveInto.hpp"

#include <libada/util.hpp>
#include "feeding/perception/PerceptionServoClient.hpp"
#include "feeding/util.hpp"

namespace feeding {
namespace action {

bool moveInto(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Perception>& perception,
    const ::ros::NodeHandle* nodeHandle,
    std::shared_ptr<FTThresholdHelper> ftThresholdHelper,
    double velocityLimit,
    const std::string mFoodUid,
    Eigen::Vector3d foodOffset)
{
  ROS_INFO_STREAM("Move into " + mFoodUid);

  PerceptionServoClient servoClient(
      nodeHandle,
      std::bind(&Perception::perceiveFoodByUID, perception.get(), mFoodUid, foodOffset),
      ada,
      ros::Duration(0.5), // Update trajectory at 2Hz
      ros::Duration(15.0), // Don't stop if we cannot see food
      0.0, // Don't stop until force threshold met
      velocityLimit,
      ftThresholdHelper);

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
  return (result == ada::CartVelocityResult::kCVR_FORCE);
}

} // namespace action
} // namespace feeding
