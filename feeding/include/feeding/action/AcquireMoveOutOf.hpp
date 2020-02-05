#ifndef FEEDING_ACTION_ACQUIREMOVEOUTOF_HPP_
#define FEEDING_ACTION_ACQUIREMOVEOUTOF_HPP_

#include <libada/Ada.hpp>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/TargetItem.hpp"
#include "feeding/Workspace.hpp"

namespace feeding {
namespace action {

void acquireMoveOutOf(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    TargetItem item,
    double length,
    Eigen::Vector3d direction,
    double planningTimeout,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper,
    double liftoffAngle);
}
} // namespace feeding

#endif