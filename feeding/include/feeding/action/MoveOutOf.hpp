#ifndef FEEDING_ACTION_MOVEOUTOF_HPP_
#define FEEDING_ACTION_MOVEOUTOF_HPP_

#include <libada/Ada.hpp>
#include "feeding/Workspace.hpp"
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/TargetItem.hpp"

namespace feeding {
namespace action {

void moveOutOf(
  const std::shared_ptr<ada::Ada>& ada,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  TargetItem item,
  double length,
  double planningTimeout,
  double endEffectorOffsetPositionTolerance,
  double endEffectorOffsetAngularTolerance,
  bool ignoreCollision,
  const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper);

}
}

#endif