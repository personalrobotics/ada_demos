#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveAbove.hpp"

namespace feeding {
namespace action {

bool moveAbovePlate(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double heightAbovePlate,
    double horizontalTolerance,
    double verticalTolerance,
    double rotationTolerance,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits)
{
  return moveAbove(
      ada,
      collisionFree,
      plate,
      plateEndEffectorTransform,
      horizontalTolerance,
      verticalTolerance,
      rotationTolerance,
      0.0, // tilt tolereance
      planningTimeout,
      maxNumTrials,
      velocityLimits);
}

} // namespace action
} // namespace feeding
