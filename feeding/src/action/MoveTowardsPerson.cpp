#include "feeding/action/MoveTowardsPerson.hpp"

#include <libada/util.hpp>
#include "feeding/perception/PerceptionServoClient.hpp"

namespace feeding {
namespace action {

bool moveTowardsPerson(
  const std::shared_ptr<ada::Ada>& ada,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  const std::shared_ptr<Perception>& perception,
  ros::NodeHandle nodeHandle,
  double distanceToPerson,
  double planningTimeout,
  double endEffectorOffsetPositionTolerenace,
  double endEffectorOffsetAngularTolerance)
{
  ada::util::waitForUser("Move towards person", ada);

  int numDofs = ada->getArm()->getMetaSkeleton()->getNumDofs();
  Eigen::VectorXd velocityLimits = Eigen::VectorXd::Zero(numDofs);
  for (int i = 0; i < numDofs; i++)
    velocityLimits[i] = 0.2;

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
      velocityLimits,
      0,
      0.06);
  servoClient.start();
  return servoClient.wait(30);

}

}
}
