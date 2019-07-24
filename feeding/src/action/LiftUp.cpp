#include "feeding/action/LiftUp.hpp"
#include <libada/util.hpp>
#include "feeding/util.hpp"

namespace feeding {
namespace action {

const static std::vector<std::string> trajectoryController{"rewd_trajectory_controller"};
const static std::vector<std::string> ftTrajectoryController{"move_until_touch_topic_controller"};
bool liftUp(
    const std::shared_ptr<::ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double planningTimeout,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper,
    double upDist)
{

  ROS_INFO_STREAM("Lift Up");

  Eigen::Vector3d backDirection(0, 0, 1);

  bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
                          backDirection,
                          upDist,
                          collisionFree,
                          planningTimeout,
                          endEffectorOffsetPositionTolerance,
                          endEffectorOffsetAngularTolerance);

  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
  if (ftThresholdHelper)
  {
    ROS_WARN_STREAM("Start FT, stop Traj Controller");
    bool result = ada->switchControllers(ftTrajectoryController, trajectoryController);
    if (!result)
    {
      ROS_WARN_STREAM("Failed to switch; continue with traj controller");
    }
    else
    {
      ftThresholdHelper->setThresholds(STANDARD_FT_THRESHOLD);
    }
  }
  return trajectoryCompleted;
}

} // namespace feeding
} // namespace action
