#include "feeding/action/MoveTowardsPerson.hpp"

#include <libada/util.hpp>
#include "feeding/perception/PerceptionServoClient.hpp"

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
  ada::util::waitForUser("Move towards person", ada);

  int numDofs = ada->getArm()->getMetaSkeleton()->getNumDofs();
  std::vector<double> velocityLimits(numDofs, 0.2);

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
      0,
      0.06,
      planningTimeout,
      endEffectorOffsetPositionTolerenace,
      endEffectorOffsetAngularTolerance,
      velocityLimits);
  servoClient.start();
  return servoClient.wait(30);
}
}
}
