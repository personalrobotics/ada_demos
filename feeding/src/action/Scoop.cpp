#include "feeding/action/KinovaScoop.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"
#include "feeding/AcquisitionAction.hpp"

#include "aikido/trajectory/util.hpp"
#include "aikido/robot/util.hpp"
#include <aikido/trajectory/Interpolated.hpp>
#include <libada/util.hpp>

static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;
using aikido::trajectory::concatenate;
using aikido::trajectory::Interpolated;
using aikido::trajectory::TrajectoryPtr;

using ada::util::getRosParam;

namespace feeding {
namespace action {

// bool moveWithEndEffectorTwist(
//     const Eigen::Vector6d& twists,
//     double duration,
//     bool respectCollision)
// {
//    // temporarily disabling
//   return mAda->moveArmWithEndEffectorTwist(
//     Eigen::Vector6d(getRosParam<std::vector<double>>("/scoop/twist1",
//     mNodeHandle).data()),
//     respectCollision ? mCollisionFreeConstraint : nullptr,
//     duration,
//     getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
//     getRosParam<double>(
//           "/planning/endEffectorTwist/positionTolerance", mNodeHandle),
//     getRosParam<double>(
//           "/planning/endEffectorTwist/angularTolerance", mNodeHandle));

// }

// void scoop(const std::shared_ptr<ada::Ada>& ada)
// {
//   //  temporarily disabling
//   std::vector<std::string> twists{
//     "/scoop/twist1", "/scoop/twist2", "/scoop/twist3"};

//   for (const auto & param : twists)
//   {
//     auto success =
//       ada->moveWithEndEffectorTwist(
//         Eigen::Vector6d(getRosParam<std::vector<double>>(param,
//         mNodeHandle).data()));
//     if (!success)
//     {
//       ROS_ERROR_STREAM("Failed to execute " << param << std::endl);
//       throw std::runtime_error("Failed to execute scoop");
//     }
//   }
// }
 bool scoop(const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double height,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits) {
    // move above plate
  ROS_INFO_STREAM("Move above plate");
  bool abovePlaceSuccess = moveAbovePlate(
      ada,
      collisionFree,
      plate,
      plateEndEffectorTransform,
      horizontalToleranceAbovePlate,
      verticalToleranceAbovePlate,
      rotationToleranceAbovePlate,
      planningTimeout,
      maxNumTrials,
      velocityLimits);

  if (!abovePlaceSuccess)
  {
    talk("Sorry, I'm having a little trouble moving. Mind if I get a little help?");
    ROS_WARN_STREAM("Move above plate failed. Please restart");
    return false;
  }
  else
  {
    std::cout <<"Move above Place Success"<<std::endl;
    talk("Move above Place Success", true);
  }


 }


} // namespace feeding
} // namespace action
