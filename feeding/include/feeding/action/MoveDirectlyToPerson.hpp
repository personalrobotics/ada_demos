#ifndef FEEDING_ACTION_MOVEDIRECTLYTO_HPP_
#define FEEDING_ACTION_MOVEDIRECTLYTO_HPP_

#include <libada/Ada.hpp>

namespace feeding {
namespace action {

bool moveDirectlyToPerson(
  const std::shared_ptr<ada::Ada>& ada,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  Eigen::Isometry3d personPose,
  bool tilted,
  double distanceToPerson,
  double horizontalTolerance,
  double verticalTolerance,
  double planningTimeout,
  int maxNumTrials,
  std::vector<double> velocityLimits);

} // namespace action
} // namespace feeding

#endif
