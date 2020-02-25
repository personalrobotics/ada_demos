#include <aikido/constraint/dart/TSR.hpp>
#include <libada/util.hpp>
#include "feeding/AcquisitionAction.hpp"
#include "feeding/action/AcquireMoveAboveFood.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/util.hpp"

using aikido::constraint::dart::TSR;

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

bool acquireMoveAboveFood(
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
    double incidentAngle, // need to incorporate this incident angle!
    int maxNumTrials,
    std::vector<double> velocityLimits,
    FeedingDemo* feedingDemo)
{
  Eigen::Isometry3d target;
  Eigen::Isometry3d eeTransform
      = *ada->getHand()->getEndEffectorTransform("food");
  Eigen::AngleAxisd rotation
      = Eigen::AngleAxisd(-rotateAngle, Eigen::Vector3d::UnitZ());

  std::cout << "Rotation angle: " << rotateAngle << std::endl;
  std::cout<< "Incident angle: " << incidentAngle << std::endl;

    // taken from demo/humanStudy branch
    target = removeRotation(foodTransform);
    eeTransform.linear() 
      = eeTransform.linear() * rotation
          * Eigen::AngleAxisd(-incidentAngle, Eigen::Vector3d::UnitX());
    eeTransform.translation()
      = Eigen::AngleAxisd(rotateAngle, Eigen::Vector3d::UnitZ()) // Take into account action rotation
        * Eigen::Vector3d{0,
                        -sin(M_PI * 0.25) * heightAboveFood * 0.7,
                        cos(M_PI * 0.25) * heightAboveFood * 0.9};
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

} // namespace action
} // namespace feeding
