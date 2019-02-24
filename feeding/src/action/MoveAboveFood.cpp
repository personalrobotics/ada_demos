#include "feeding/action/MoveAboveFood.hpp"
#include <aikido/constraint/dart/TSR.hpp>
#include <libada/util.hpp>
#include "feeding/AcquisitionAction.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/util.hpp"

using aikido::constraint::dart::TSR;

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

bool moveAboveFood(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    std::string foodName,
    const Eigen::Isometry3d& foodTransform,
    float rotateAngle,
    TiltStyle tiltStyle,
    double heightAboveFood,
    double horizontalTolerance,
    double verticalTolerance,
    double rotationTolerance,
    double tiltTolerance,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits,
    FeedingDemo* feedingDemo)
{
  ada::util::waitForUser(
      "Rotate forque to angle " + std::to_string(rotateAngle), ada);

  Eigen::Isometry3d target;
  Eigen::Isometry3d eeTransform
      = *ada->getHand()->getEndEffectorTransform("food");
  Eigen::AngleAxisd rotation
      = Eigen::AngleAxisd(-rotateAngle, Eigen::Vector3d::UnitZ());

  if (tiltStyle == TiltStyle::NONE)
  {
    target = foodTransform;
    eeTransform.linear() = eeTransform.linear() * rotation;
    eeTransform.translation()[2] = -heightAboveFood;
  }
  else if (tiltStyle == TiltStyle::VERTICAL)
  {
    target = removeRotation(foodTransform);
    eeTransform.linear()
        = eeTransform.linear() * rotation
          * Eigen::AngleAxisd(-M_PI * 0.5, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(M_PI + 0.5, Eigen::Vector3d::UnitX());
    eeTransform.translation()[2] = heightAboveFood;
  }
  else // angled
  {
    target = removeRotation(foodTransform);
    eeTransform.linear()
        = eeTransform.linear() * rotation
          * Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(M_PI * 5.0 / 6.0, Eigen::Vector3d::UnitX());
    eeTransform.translation()
        = Eigen::Vector3d{-sin(M_PI * 0.25) * heightAboveFood * 0.5,
                          0,
                          cos(M_PI * 0.25) * heightAboveFood * 0.5};
  }

  std::cout << "Tilt Style " << tiltStyle << std::endl;
  std::cout << "Tilt Tolerance "  << tiltTolerance << std::endl;

  return moveAbove(
      ada,
      collisionFree,
      target,
      eeTransform,
      horizontalTolerance,
      verticalTolerance,
      rotationTolerance,
      0.0,
      planningTimeout,
      maxNumTrials,
      velocityLimits,
      feedingDemo);
}

} // namespace feeding
} // namespace action
