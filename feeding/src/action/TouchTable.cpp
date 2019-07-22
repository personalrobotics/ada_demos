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
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper,
    std::vector<double> velocityLimits,
    float angle)
{

  ROS_INFO_STREAM("Move Down");

  bool rotate;
  if (angle != 0.0) 
  {
    Eigen::VectorXd twists(6);
    twists << 0.0, 0.0, angle, 0.0, 0.0, 0.0;

    ROS_INFO_STREAM("Rotate forque");
    rotate = ada->moveArmWithEndEffectorTwist(
                    twists,
                    1,
                    collisionFree,
                    planningTimeout,
                    endEffectorOffsetPositionTolerance,
                    endEffectorOffsetAngularTolerance,
                    velocityLimits);
  } else {
    rotate = true;
  }

  if (rotate) {
    ftThresholdHelper->setThresholds(1, 1); // For stopping traj when touch the table

    Eigen::Vector3d direction(0, 0, -1);
    double length = 0.04; // maximum go-down distance

    bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
                                  direction,
                                  length,
                                  collisionFree,
                                  planningTimeout,
                                  endEffectorOffsetPositionTolerance,
                                  endEffectorOffsetAngularTolerance);
    
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
    return trajectoryCompleted;
  } else {
    return false;
  }
}

} // namespace feeding
} // namespace action
