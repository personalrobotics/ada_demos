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
    Eigen::Vector3d direction,
    double planningTimeout,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper)
{
  ada::util::waitForUser("Move Out of " + TargetToString.at(item), ada);
  if (ftThresholdHelper)
    ftThresholdHelper->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD);

  // Do it three times
  for (int i = 0; i < 3; ++i)
  {
    bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
        direction,
        length,
        collisionFree,
        planningTimeout,
        endEffectorOffsetPositionTolerance,
        endEffectorOffsetAngularTolerance);

    // trajectoryCompleted might be false because the forque hit the food
    // along the way and the trajectory was aborted
  }

  if (ftThresholdHelper)
    ftThresholdHelper->setThresholds(STANDARD_FT_THRESHOLD);
}

} // namespace feeding
} // namespace action
