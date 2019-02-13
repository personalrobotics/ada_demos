#include "feeding/action/MoveToPerson.hpp"

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

bool moveToPerson(
  const std::shared_ptr<ada::Ada>& ada,
  double distanceToPerson,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  double planningTimeout,
  double endEffectorOffsetPositionTolerenace,
  double endEffectorOffsetAngularTolerance);
{
  waitForUser("Move towards person");

  if (!mAdaReal)
  {
    return ada->moveArmToEndEffectorOffset(
        Eigen::Vector3d(0, 1, 0),
        distanceToPerson * 0.4,
        collisionFree,
        planningTimeout,
        endEffectorOffsetPositionTolerenace,
        endEffectorOffsetAngularTolerance);
  }

  /* TODO
  std::shared_ptr<aikido::control::TrajectoryExecutor> executor
      = ada->getTrajectoryExecutor();
  std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> rosExecutor
      = std::dynamic_pointer_cast<aikido::control::ros::RosTrajectoryExecutor>(
          executor);

  if (rosExecutor == nullptr)
  {
    throw std::runtime_error("no ros executor");
  }

  int numDofs = ada->getArm()->getMetaSkeleton()->getNumDofs();
  Eigen::VectorXd velocityLimits = Eigen::VectorXd::Zero(numDofs);
  for (int i = 0; i < numDofs; i++)
    velocityLimits[i] = 0.2;

  feeding::PerceptionServoClient servoClient(
      mNodeHandle,
      boost::bind(&Perception::perceiveFace, mPerception.get()),
      mArmSpace,
      ada,
      ada->getArm()->getMetaSkeleton(),
      ada->getHand()->getEndEffectorBodyNode(),
      rosExecutor,
      mCollisionFreeConstraint,
      0.2,
      velocityLimits,
      0,
      0.06);
  servoClient.start();
  return servoClient.wait(30);
  */
}

}
}
