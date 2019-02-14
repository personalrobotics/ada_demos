#ifndef FEEDING_ACTION_FEEDFOODTOPERSON_HPP_
#define FEEDING_ACTION_FEEDFOODTOPERSON_HPP_

#include <libada/Ada.hpp>
#include "feeding/perception/Perception.hpp"
#include "feeding/Workspace.hpp"

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

void feedFoodToPerson(
  const std::shared_ptr<ada::Ada>& ada,
  const std::shared_ptr<Workspace>& workspace,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  const std::shared_ptr<Perception>& perception,
  ros::NodeHandle nodeHandle,
  const Eigen::Isometry3d& plate,
  const Eigen::Isometry3d& plateEndEffectorTransform,
  const Eigen::Isometry3d& personPose,
  std::chrono::milliseconds waitAtPerson,
  double heightAbovePlate,
  double horizontalToleranceAbovePlate,
  double verticalToleranceAbovePlate,
  double rotationToleranceAbovePlate,
  double distanceToPerson,
  double horizontalToleranceForPerson,
  double verticalToleranceForPerson,
  double planningTimeout,
  int maxNumTrials,
  double endEffectorOffsetPositionTolerenace,
  double endEffectorOffsetAngularTolerance,
  std::vector<double> velocityLimits,
  const Eigen::Vector3d* tiltOffset);
}
}

#endif
