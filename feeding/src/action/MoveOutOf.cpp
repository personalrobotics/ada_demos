#include "feeding/action/MoveOutOf.hpp"

#include <libada/util.hpp>

#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/util.hpp"

namespace feeding {
namespace action {

void moveOutOf(
  const std::shared_ptr<::ada::Ada>& ada,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  TargetItem item,
  double length,
  double planningTimeout,
  double endEffectorOffsetPositionTolerance,
  double endEffectorOffsetAngularTolerance,
  bool ignoreCollision,
  const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper)
{
  if (item != FOOD && item != PLATE && item != FORQUE)
    throw std::invalid_argument(
        "MoveOutOf[" + TargetToString.at(item) + "] not supported");

  ada::util::waitForUser("Move Out of " + TargetToString.at(item), ada);
  if (ftThresholdHelper)
    ftThresholdHelper->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD);

  Eigen::Vector3d direction(0, 0, 1);

  // TODO: move to the caller
  // if (item == PLATE)
  //   length = getRosParam<double>("/feedingDemo/heightOutOfPlate", mNodeHandle);
  // else if (item == FOOD)
  //   length = getRosParam<double>("/feedingDemo/moveOutofFood", mNodeHandle);
  // else
  // {
  //   length = 0.04;
    // direction = Eigen::Vector3d(0, -1, 0);
  // }
  if (item != PLATE && item != FOOD)
  {
    direction = Eigen::Vector3d(0, -1, 0);
  }

  for(int i = 0; i < 3; ++i)
  {
    if (ignoreCollision)
    {
      bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
        direction, length, nullptr, //mCollisionFreeConstraint,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance);
    }
    else
    {
      bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
        direction, length, collisionFree,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance);
    }
  }

  if (ftThresholdHelper)
    ftThresholdHelper->setThresholds(STANDARD_FT_THRESHOLD);
  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
}


} // namespace feeding
} // namespace action
