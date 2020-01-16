#ifndef FEEDING_ACTION_KINOVSCOOP_HPP_
#define FEEDING_ACTION_KINOVSCOOP_HPP_

#include <libada/Ada.hpp>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/Workspace.hpp"
#include "feeding/perception/Perception.hpp"
#include "feeding/FeedingDemo.hpp"

namespace feeding {
namespace action {

bool push(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double timelimit,
    double positionTolerance,
    double angularTolerance,
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper,
    std::vector<double> velocityLimits,
    float angle
);
} // namespace action
} // namespace feeding

#endif