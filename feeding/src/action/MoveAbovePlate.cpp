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

  Eigen::VectorXd config(6);
  // is this the config of the point above the plate? change it to test.
  config << -2.15583, 3.0954,  1.61802,  -2.45501,  -2.04492, -4.73983 ;
  bool success = ada->moveArmToConfiguration(config, collisionFree, 2.0);
  // what's the difference between ada->moveArmToConfiguration and moveAbove
  if (!success) {
    std::cout <<"Try moveAbove"<<std::endl;
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
  }
  else
    return success;
}

} // namespace action
} // namespace feeding
