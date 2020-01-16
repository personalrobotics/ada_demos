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

  // // Hardcoded pose
  // Eigen::VectorXd config(6);
  // // config << -2.11666, 3.34967, 2.04129, -2.30031, -2.34026, 2.9545;
  // config << -1.4614, 3.0954,  1.61802,  -2.45501,  -2.04492, -2.3529; // kyle's pushing config 
  // bool success = ada->moveArmToConfiguration(config, collisionFree, 2.0);
  bool success = false;
  if (!success) {
    std::cout <<"Try moveAbove"<<std::endl;
    return moveAbove(
        ada,
        collisionFree,
        plate,
        plateEndEffectorTransform,
        horizontalTolerance,
        verticalTolerance,
        rotationTolerance, // rotationTolerance should be small enough.
        0.03,
        planningTimeout,
        maxNumTrials,
        velocityLimits);
  }
  // if (!success)
  //   return moveAbove(
  //       ada,
  //       collisionFree,
  //       plate,
  //       plateEndEffectorTransform,
  //       horizontalTolerance,
  //       verticalTolerance,
  //       M_PI,
  //       0.03,
  //       planningTimeout,
  //       maxNumTrials,
  //       velocityLimits);
  else
    return success;
}

} // namespace action
} // namespace feeding
