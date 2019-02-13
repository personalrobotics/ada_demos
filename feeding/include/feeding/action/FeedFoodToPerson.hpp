#ifndef FEEDING_ACTION_FEEDFOODTOPERSON_HPP_
#define FEEDING_ACTION_FEEDFOODTOPERSON_HPP_

#include <libada/Ada.hpp>
#include "feeding/Workspace.hpp"

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

void feedFoodToPerson(
  const std::shared_ptr<ada::Ada>& ada,
  const std::shared_ptr<Workspace>& workspace,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  const Eigen::Isometry3d& plate,
  const Eigen::Isometry3d& plateEndEffectorTransform,
  std::chrono::milliseconds waitAtPerson,
  bool tilted,
  double heightAbovePlate,
  double horizontalToleranceAbovePlate,
  double verticalToleranceAbovePlate,
  double rotationToleranceAbovePlate,
  double distanceToPerson,
  double horizontalToleranceForPerson,
  double verticalToleranceForPerson,
  double planningTimeout,
  int maxNumTrials,
  std::vector<double> velocityLimits);
}
}

#endif
