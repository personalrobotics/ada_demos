#ifndef FEEDING_ACTION_MOVEINFRONTOFPERSON_HPP_
#define FEEDING_ACTION_MOVEINFRONTOFPERSON_HPP_

#include <libada/Ada.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/Workspace.hpp"

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

bool moveInFrontOfPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    std::vector<double> velocityLimits,
    FeedingDemo* feedingDemo = nullptr);
}
} // namespace feeding

#endif