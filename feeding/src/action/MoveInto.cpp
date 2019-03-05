#include "feeding/action/PutDownFork.hpp"

#include <libada/util.hpp>

#include "feeding/TargetItem.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/perception/PerceptionServoClient.hpp"
#include "feeding/util.hpp"

namespace feeding {
namespace action {

bool moveInto(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Perception>& perception,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const ::ros::NodeHandle* nodeHandle,
    TargetItem item,
    double planningTimeout,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    const Eigen::Vector3d& endEffectorDirection,
    std::shared_ptr<FTThresholdHelper> ftThresholdHelper)
{
  ROS_INFO_STREAM("Move into " + TargetToString.at(item));

  if (item != FOOD && item != FORQUE)
    throw std::invalid_argument(
        "MoveInto[" + TargetToString.at(item) + "] not supported");

  if (item == TargetItem::FORQUE)
    return ada->moveArmToEndEffectorOffset(
        Eigen::Vector3d(0, 1, 0),
        0.01,
        collisionFree,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance);

  // if (perception)
  // {
  //   ROS_INFO("Servoing into food");

  //   int numDofs = ada->getArm()->getMetaSkeleton()->getNumDofs();
  //   std::vector<double> velocityLimits(numDofs, 0.2);

  //   PerceptionServoClient servoClient(
  //       nodeHandle,
  //       boost::bind(&Perception::getTrackedFoodItemPose, perception.get()),
  //       ada->getArm()->getStateSpace(),
  //       ada,
  //       ada->getArm()->getMetaSkeleton(),
  //       ada->getHand()->getEndEffectorBodyNode(),
  //       ada->getTrajectoryExecutor(),
  //       nullptr,
  //       1.0,
  //       0.002,
  //       planningTimeout,
  //       endEffectorOffsetPositionTolerance,
  //       endEffectorOffsetAngularTolerance,
  //       true, // servoFood
  //       velocityLimits);
  //   servoClient.start();

  //   return servoClient.wait(15.0);
  // }
  // else

  std::cout << "endEffectorDirection " << endEffectorDirection.transpose() << std::endl;
  {
    double length = 0.05;
    int numDofs = ada->getArm()->getMetaSkeleton()->getNumDofs();
    // Collision constraint is not set because f/t sensor stops execution.

    auto result = ada->moveArmToEndEffectorOffset(
        endEffectorDirection,
        length,
        nullptr,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance);
    ROS_INFO_STREAM(" Execution result: " << result);
  }

  return true;
}

} // namespace feeding
} // namespace action