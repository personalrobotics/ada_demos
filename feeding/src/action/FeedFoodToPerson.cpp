#include "feeding/action/FeedFoodToPerson.hpp"
#include <libada/util.hpp>
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveTowardsPerson.hpp"

namespace feeding {
namespace action {

//==============================================================================
void feedFoodToPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Workspace>& workspace,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
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
    const Eigen::Vector3d* tiltOffset)
{
  auto moveIFOPerson = [&] {
    return moveInFrontOfPerson(
        ada,
        collisionFree,
        personPose,
        distanceToPerson,
        horizontalToleranceForPerson,
        verticalToleranceForPerson,
        planningTimeout,
        maxNumTrials,
        velocityLimits);
  };

  bool moveIFOSuccess = false;
  bool moveSuccess = false;
  for (std::size_t i = 0; i < 2; ++i)
  {
    moveIFOSuccess = moveIFOPerson();
    if (!moveIFOSuccess)
    {
      ROS_WARN_STREAM("Failed to move in front of person, retry");
      continue;
    }
    else
      break;
  }

  std::cout << "IFO Current pose \n" <<
    ada->getMetaSkeleton()->getPositions().transpose() << std::endl;

  if (moveIFOSuccess)
  {
    nodeHandle->setParam("/feeding/facePerceptionOn", true);

    ROS_INFO_STREAM("Move towards person");
    moveSuccess = moveTowardsPerson(
        ada,
        nullptr,
        perception,
        nodeHandle,
        distanceToPerson,
        planningTimeout,
        endEffectorOffsetPositionTolerenace,
        endEffectorOffsetAngularTolerance);
    nodeHandle->setParam("/feeding/facePerceptionOn", false);
  }

  if (moveIFOSuccess && moveSuccess)
  {
    // ===== EATING =====
    ROS_WARN("Human is eating");
    std::this_thread::sleep_for(waitAtPerson);

    // Backward
    ada::util::waitForUser("Move backward", ada);
    Eigen::Vector3d goalDirection(0, -1, 0);
    ada->moveArmToEndEffectorOffset(
      goalDirection.normalized(),
      0.1,
      nullptr,
      planningTimeout,
      endEffectorOffsetPositionTolerenace,
      endEffectorOffsetAngularTolerance);
  }

  // ===== BACK TO PLATE =====
  ada::util::waitForUser("Move back to plate", ada);

  // TODO: add a back-out motion and then do move above plate with
  // collisionFree.
  moveAbovePlate(
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
}
}
}