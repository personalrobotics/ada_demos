#include "feeding/action/Push.hpp"
#include "feeding/AcquisitionAction.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <libada/util.hpp>
#include "aikido/robot/util.hpp"
#include "aikido/trajectory/util.hpp"

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <math.h>

using ada::util::waitForUser;
static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

using namespace std;
using namespace Eigen;
using aikido::trajectory::concatenate;
using aikido::trajectory::Interpolated;
using aikido::trajectory::TrajectoryPtr;
using State = aikido::statespace::dart::MetaSkeletonStateSpace::State;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using ada::util::createIsometry;

namespace feeding {
namespace action {

//==============================================================================
bool push(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double timelimit,
    double positionTolerance,
    double angularTolerance,
    std::vector<double> velocityLimits,
    float angle,
    double pushDist)
{
  float xOff = cos(angle); // angle - M_PI * 0.5) * 0.05;
  float yOff = sin(angle); //angle - M_PI * 0.5) * 0.05;

  if (pushDist < 0) {
      xOff *= -1;
      yOff *= -1;
      pushDist *= -1;
  }

  ROS_INFO_STREAM("Push forque");
  return ada->moveArmToEndEffectorOffset(
                              Eigen::Vector3d(-xOff, -yOff, 0),
                              pushDist,
                              collisionFree,
                              timelimit,
                              positionTolerance,
                              angularTolerance,
                              velocityLimits);
}

} // namespace action
} // namespace action
