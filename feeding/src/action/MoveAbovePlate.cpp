#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveAbove.hpp"

namespace feeding {
namespace action {

bool moveAbovePlate(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double horizontalTolerance,
    double verticalTolerance,
    double rotationTolerance,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits)
{

  // Eigen::VectorXd config(6);
  // config << -1.21321, 3.05786, 1.33674, -2.66234, -1.65711, -0.00439555;
  // bool success = ada->moveArmToConfiguration(config, collisionFree, 2.0);
  // if (!success)
    return moveAbove(
        ada,
        collisionFree,
        plate,
        plateEndEffectorTransform,
        horizontalTolerance,
        verticalTolerance,
        M_PI,
        0.03,
        planningTimeout,
        maxNumTrials,
        velocityLimits);
  // else
  //   return success;
}

} // namespace action
} // namespace feeding
