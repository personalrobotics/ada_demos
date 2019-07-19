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
    std::vector<double> velocityLimits,
    float angle,
    double pushDist
);
} // namespace action
} // namespace feeding

#endif