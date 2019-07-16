#include "feeding/action/TouchTable.hpp"
#include <libada/util.hpp>
#include "feeding/util.hpp"

namespace feeding {
namespace action {

const static std::vector<std::string> trajectoryController{"rewd_trajectory_controller"};
const static std::vector<std::string> ftTrajectoryController{"move_until_touch_topic_controller"};
bool touchTable(
    const std::shared_ptr<::ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double planningTimeout,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper)
{

  ROS_INFO_STREAM("Move Down");

  if (ftThresholdHelper)
  {
    ROS_WARN_STREAM("Stop FT, start Traj Controller");
    ada->getTrajectoryExecutor()->cancel();
    bool result = ada->switchControllers(trajectoryController, ftTrajectoryController);
    if (!result)
    {
      ROS_WARN_STREAM("Failed to switch; continue with FT controller");
      ftThresholdHelper->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD);
    }
  }

  Eigen::Vector3d direction(0, 0, -1);
  double length = 0.1;

  bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
      direction,
      length,
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
  direction[2] = 1;
  length = 0.05;

  return ada->moveArmToEndEffectorOffset(
            direction,
            length,
            collisionFree,
            planningTimeout,
            endEffectorOffsetPositionTolerance,
            endEffectorOffsetAngularTolerance);
}

} // namespace feeding
} // namespace action
