#ifndef FEEDING_ACTION_KINOVSCOOP_HPP_
#define FEEDING_ACTION_KINOVSCOOP_HPP_

#include <libada/Ada.hpp>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/Workspace.hpp"
#include "feeding/perception/Perception.hpp"
#include "feeding/FeedingDemo.hpp"

namespace feeding {
namespace action {

bool PushOnFood(
    const std::shared_ptr<ada::Ada>& ada,
    std::string push_direction,
    double length,
    const Eigen::Isometry3d& T0_w,
    Eigen::Isometry3d Tw_e,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double horizontalTolerance,
    double verticalTolerance,
    double rotationTolerance,
    double tiltTolerance,
    double planningTimeout,
    int maxNumTrials,
    const std::vector<double>& velocityLimits
);
} // namespace action
} // namespace feeding

#endif
