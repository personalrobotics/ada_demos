#include "feeding/action/PutDownFork.hpp"

#include <libada/util.hpp>

#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/TargetItem.hpp"
#include "feeding/util.hpp"


namespace feeding {
namespace action {

bool moveInto(
  const std::shared_ptr<ada::Ada>& ada,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  TargetItem item,
  double planningTimeout,
  double endEffectorOffsetPositionTolerenace,
  double endEffectorOffsetAngularTolerance,
  const Eigen::Vector3d& endEffectorDirection,
  std::shared_ptr<FTThresholdHelper> ftThresholdHelper)
{
  // using aikido::control::ros::RosTrajectoryExecutor;

  ada::util::waitForUser("Move into " + TargetToString.at(item), ada);

  if (ftThresholdHelper)
    ftThresholdHelper->setThresholds(GRAB_FOOD_FT_THRESHOLD);

  if (item != FOOD && item != FORQUE)
    throw std::invalid_argument(
        "MoveInto[" + TargetToString.at(item) + "] not supported");

  if (item == TargetItem::FORQUE)
    return ada->moveArmToEndEffectorOffset(
        Eigen::Vector3d(0, 1, 0), 0.01, collisionFree,
    planningTimeout,
    endEffectorOffsetPositionTolerenace,
    endEffectorOffsetAngularTolerance);

  // TODO: fix
  /*
  if (mAdaReal && mPerception && mVisualServo)
  {
    ROS_INFO("Servoing into food");
    auto rosExecutor
        = std::dynamic_pointer_cast<RosTrajectoryExecutor>(
          ada->getTrajectoryExecutor());

    if (rosExecutor == nullptr)
    {
      throw std::runtime_error("no ros executor");
    }

    int numDofs = ada->getArm()->getMetaSkeleton()->getNumDofs();
    Eigen::VectorXd velocityLimits = Eigen::VectorXd::Ones(numDofs) * 0.2;

    PerceptionServoClient servoClient(
        mNodeHandle,
        boost::bind(&Perception::getTrackedFoodItemPose, mPerception.get()),
        mArmSpace,
        mAda,
        mAda->getArm()->getMetaSkeleton(),
        mAda->getHand()->getEndEffectorBodyNode(),
        rosExecutor,
        CollisionFreePtr,
        0.1,
        velocityLimits,
        0.1,
        0.002);
    servoClient.start();

    return servoClient.wait(10000.0);
  }
  */

  double length = 0.025;

  for(int i = 0; i < 2; ++i)
  {
    // Collision constraint is not set because f/t sensor stops execution.
    auto result = ada->moveArmToEndEffectorOffset(
        endEffectorDirection, length, nullptr,
        planningTimeout,
        endEffectorOffsetPositionTolerenace,
        endEffectorOffsetAngularTolerance);
    ROS_INFO_STREAM(" Execution result: " << result);
  }

  return true;
}


} // namespace feeding
} // namespace action
