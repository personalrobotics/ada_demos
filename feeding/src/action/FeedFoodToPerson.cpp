#include "feeding/action/FeedFoodToPerson.hpp"
#include <libada/util.hpp>
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
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
  auto moveIFOPerson = [&]
  {
    return moveInFrontOfPerson(
      ada,
      nullptr, // collisionFree,
      workspace->getPersonPose(), // TODO why not personPose
      distanceToPerson,
      horizontalToleranceForPerson,
      verticalToleranceForPerson,
      planningTimeout,
      maxNumTrials,
      velocityLimits);
  };

  bool moveSuccess = false;

  for (std::size_t i = 0; i < 2; ++i)
  {
    moveIFOPerson();
    nodeHandle->setParam("/feeding/facePerceptionOn", true);

    ada::util::waitForUser("Move towards person", ada);

    moveSuccess = moveTowardsPerson(
      ada,
      collisionFree,
      perception,
      nodeHandle,
      distanceToPerson,
      planningTimeout,
      endEffectorOffsetPositionTolerenace,
      endEffectorOffsetAngularTolerance);
    nodeHandle->setParam("/feeding/facePerceptionOn", false);

    if (moveSuccess)
      break;
    ROS_INFO_STREAM("Moved failed, backing up and retrying");
  }

  if (!moveSuccess)
  {
    ROS_INFO_STREAM("Servoing failed. Falling back to direct movement...");
    moveIFOPerson();
    moveDirectlyToPerson(
      ada,
      collisionFree,
      personPose,
      distanceToPerson,
      horizontalToleranceForPerson,
      verticalToleranceForPerson,
      planningTimeout,
      maxNumTrials,
      velocityLimits,
      tiltOffset);
  }

  // ===== EATING =====
  ROS_WARN("Human is eating");
  std::this_thread::sleep_for(waitAtPerson);
  ungrabAndDeleteFood(ada, workspace);
  ada::util::waitForUser("Move away from person", ada);

  // ===== BACK TO PLATE =====
  ada::util::waitForUser("Move back to plate", ada);

  moveAbovePlate(
    ada,
    collisionFree,
    plate,
    plateEndEffectorTransform,
    heightAbovePlate,
    horizontalToleranceAbovePlate,
    verticalToleranceAbovePlate,
    rotationToleranceAbovePlate,
    planningTimeout,
    maxNumTrials,
    velocityLimits
    );
}

}
}