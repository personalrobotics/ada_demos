#ifndef FEEDING_ACTION_POLYSCOOP_HPP_
#define FEEDING_ACTION_POLYSCOOP_HPP_

#include <libada/Ada.hpp>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/Workspace.hpp"
#include "feeding/perception/Perception.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/action/ScoopHelper.hpp"
#include "feeding/action/PolyTraj.hpp"

namespace feeding {
namespace action {

bool PolyScoop(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double hight,
    double theta,
    double minima,
    double direction, 
    int demotype,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits);
}
}

#endif