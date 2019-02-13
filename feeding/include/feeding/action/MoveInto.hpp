#ifndef FEEDING_ACTION_MOVEINTO_HPP_
#define FEEDING_ACTION_MOVEINTO_HPP_

#include <libada/Ada.hpp>
#include "feeding/Workspace.hpp"
#include "feeding/TargetItem.hpp"

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
  std::shared_ptr<FTThresholdHelper> ftThresholdHelper
  );

} // namespace action
} // namespace feeding
#endif