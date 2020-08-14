#include "feeding/action/MoveTowardsPerson.hpp"

#include <libada/util.hpp>

#include "feeding/perception/Perception.hpp"

namespace feeding {
namespace action {

bool moveTowardsPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    double distanceToPerson,
    double planningTimeout,
    double endEffectorOffsetPositionTolerenace,
    double endEffectorOffsetAngularTolerance)
{
  ROS_INFO_STREAM("Move towards person");

  int numDofs = ada->getArm()->getMetaSkeleton()->getNumDofs();
  // FAST
  std::vector<double> velocityLimits(numDofs, 0.3);
  // SLOW
  // std::vector<double> velocityLimits(numDofs, 0.1);

  /*

  PerceptionServoClient servoClient(
      nodeHandle,
      boost::bind(&Perception::perceiveFace, perception.get()),
      ada->getArm()->getStateSpace(),
      ada,
      ada->getArm()->getMetaSkeleton(),
      ada->getHand()->getEndEffectorBodyNode(),
      ada->getTrajectoryExecutor(),
      collisionFree,
      0.2,
      0.015,
      planningTimeout,
      endEffectorOffsetPositionTolerenace,
      endEffectorOffsetAngularTolerance,
      false, // not food
      velocityLimits);
  servoClient.start();
  return servoClient.wait(10);
  */

  // Read Person Pose
  bool seePerson = false;
  Eigen::Isometry3d personPose;
  while (!seePerson)
  {
    try
    {
      personPose = perception->perceiveFace();
      seePerson = true;
    }
    catch (...)
    {
      ROS_WARN_STREAM("No Face Detected!");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
  }

  Eigen::Isometry3d currentPose
      = ada->getHand()->getEndEffectorBodyNode()->getTransform();

  // Plan from current to goal pose
  Eigen::Vector3d vectorToGoalPose
      = personPose.translation() - currentPose.translation();
  vectorToGoalPose.y() -= distanceToPerson;
  auto length = vectorToGoalPose.norm();
  vectorToGoalPose.normalize();

  ROS_WARN_STREAM("Angular Tolerance: " << endEffectorOffsetAngularTolerance);
  ROS_WARN_STREAM("Pose Tolerance: " << endEffectorOffsetPositionTolerenace);
  ROS_WARN_STREAM("Offset: " << distanceToPerson);
  ROS_WARN_STREAM("Goal Pose: " << vectorToGoalPose);

  if (!ada->moveArmToEndEffectorOffset(
          vectorToGoalPose,
          length,
          nullptr,
          planningTimeout,
          endEffectorOffsetPositionTolerenace,
          endEffectorOffsetAngularTolerance,
          velocityLimits))
  {
    ROS_WARN_STREAM("Execution failed");
    return false;
  }
  return true;
}
} // namespace action
} // namespace feeding
