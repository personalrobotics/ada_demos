#include <ros/ros.h>

#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/PushOnFood.hpp"
#include "feeding/action/Push.hpp"
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

using ada::util::createIsometry;
using ada::util::getRosParam;

namespace feeding {
namespace action {

//==============================================================================
bool PushOnFood(
    const std::shared_ptr<ada::Ada>& ada,
    std::string push_direction,
    double length,
    const Eigen::Isometry3d& T0_w,
    Eigen::Isometry3d Tw_e,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double horizontalTolerance,
    double verticalTolerance,
    double rotationTolerance,
    double tiltTolerance,
    double planningTimeout,
    int maxNumTrials,
    const std::vector<double>& velocityLimits)
{
  double pushDist = 0.08;
  double downDist = 0.03;
  if (push_direction == "left_push")
  {
    Tw_e.translation()[0] = -length;
    Tw_e.translation()[2] = downDist;
    Tw_e.linear() = Tw_e.linear() * 
    Eigen::Matrix3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
    
    ros::Duration(0.5).sleep(); // sleep for half a second
    
    bool aboveFoodSuccess = moveAbove(
      ada,
      collisionFree,
      T0_w,
      Tw_e,
      horizontalTolerance,
      verticalTolerance,
      rotationTolerance,
      tiltTolerance,
      planningTimeout,
      maxNumTrials,
      velocityLimits);

    bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
                                Eigen::Vector3d(1, 0, 0),
                                pushDist,
                                collisionFree,
                                planningTimeout,
                                0.001,
                                0.01,
                                velocityLimits);
  }  
  if (push_direction == "right_push")
  {
    Tw_e.translation()[0] = -length;
    Tw_e.linear() = Tw_e.linear() * 
    Eigen::Matrix3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));

    bool aboveFoodSuccess = moveAbove(
      ada,
      collisionFree,
      T0_w,
      Tw_e,
      horizontalTolerance,
      verticalTolerance,
      rotationTolerance,
      tiltTolerance,
      planningTimeout,
      maxNumTrials,
      velocityLimits);

    bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
                                Eigen::Vector3d(1, 0, 0),
                                pushDist,
                                collisionFree,
                                planningTimeout,
                                0.001,
                                0.01,
                                velocityLimits);
  }    
  if (push_direction == "up_push")
  {
    Tw_e.translation()[0] = -length;
    Tw_e.linear() = Tw_e.linear() * 
    Eigen::Matrix3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));

    bool aboveFoodSuccess = moveAbove(
      ada,
      collisionFree,
      T0_w,
      Tw_e,
      horizontalTolerance,
      verticalTolerance,
      rotationTolerance,
      tiltTolerance,
      planningTimeout,
      maxNumTrials,
      velocityLimits);

    bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
                                Eigen::Vector3d(1, 0, 0),
                                pushDist,
                                collisionFree,
                                planningTimeout,
                                0.001,
                                0.01,
                                velocityLimits);
  }  
  if (push_direction == "down_push")
  {
    Tw_e.translation()[0] = -length;
    Tw_e.linear() = Tw_e.linear() * 
    Eigen::Matrix3d(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));

    bool aboveFoodSuccess = moveAbove(
      ada,
      collisionFree,
      T0_w,
      Tw_e,
      horizontalTolerance,
      verticalTolerance,
      rotationTolerance,
      tiltTolerance,
      planningTimeout,
      maxNumTrials,
      velocityLimits);

    bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
                                Eigen::Vector3d(1, 0, 0),
                                pushDist,
                                collisionFree,
                                planningTimeout,
                                0.001,
                                0.01,
                                velocityLimits);
  }  

  return true;
}

} // namespace action
} // namespace action
