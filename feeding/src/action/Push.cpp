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
const static std::vector<std::string> trajectoryController{"rewd_trajectory_controller"};
const static std::vector<std::string> ftTrajectoryController{"move_until_touch_topic_controller"};

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
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper,
    std::vector<double> velocityLimits,
    float angle)
{
  float xOff = cos(angle + M_PI * 0.5); // angle - M_PI * 0.5) * 0.05;
  float yOff = sin(angle + M_PI * 0.5); //angle - M_PI * 0.5) * 0.05;
  double pushDist = 0.1;
  if (ftThresholdHelper) {
      ftThresholdHelper->setThresholds(4, 4); // For stopping traj when touch the table
  }
  ROS_INFO_STREAM("Push forque");

  Eigen::VectorXd twists(6);
  twists << 0.0, 0.0, 0.0, -xOff, -yOff, 0;

  // bool trajectoryCompleted = ada->moveArmWithEndEffectorTwist(
  //                           twists,
  //                           1,
  //                           collisionFree,
  //                           timelimit,
  //                           positionTolerance,
  //                           angularTolerance,
  //                           velocityLimits);

  bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
                                Eigen::Vector3d(-xOff, -yOff, 0),
                                pushDist,
                                collisionFree,
                                timelimit,
                                positionTolerance,
                                angularTolerance,
                                velocityLimits);
  if (ftThresholdHelper)
  {
    ROS_WARN_STREAM("Stop FT, start Traj Controller");
    ada->getTrajectoryExecutor()->cancel();
    bool result = ada->switchControllers(trajectoryController, ftTrajectoryController);
    if (!result)
    {
    ROS_WARN_STREAM("Failed to switch; continue with FT controller");
    ftThresholdHelper->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD);
    }
  }

  return true;
}

} // namespace action
} // namespace action
