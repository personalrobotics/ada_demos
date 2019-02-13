#ifndef FEEDING_ACTION_MOVETOPERSON_HPP_
#define FEEDING_ACTION_MOVETOPERSON_HPP_

#include <libada/Ada.hpp>
#include "feeding/Workspace.hpp"

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

/// Moves the forque towards the person.
/// This function does not throw an exception if the trajectory is aborted,
/// because we expect that.
bool moveToPerson(
  const std::shared_ptr<ada::Ada>& ada,
  double distanceToPerson,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  double planningTimeout,
  double endEffectorOffsetPositionTolerenace,
  double endEffectorOffsetAngularTolerance);
}
}

#endif
