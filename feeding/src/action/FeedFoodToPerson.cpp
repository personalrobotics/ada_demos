#include "feeding/action/FeedFoodToPerson.hpp"
#include <libada/util.hpp>
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
namespace feeding {
namespace action {

//==============================================================================
void feedFoodToPerson(
  const std::shared_ptr<ada::Ada>& ada,
  const std::shared_ptr<Workspace>& workspace,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  const Eigen::Isometry3d& plate,
  const Eigen::Isometry3d& plateEndEffectorTransform,
  std::chrono::milliseconds waitAtPerson,
  bool tilted,
  double heightAbovePlate,
  double horizontalToleranceAbovePlate,
  double verticalToleranceAbovePlate,
  double rotationToleranceAbovePlate,
  double distanceToPerson,
  double horizontalToleranceForPerson,
  double verticalToleranceForPerson,
  double planningTimeout,
  int maxNumTrials,
  std::vector<double> velocityLimits)
{

  bool moveSuccess = false;

  moveInFrontOfPerson(
    ada,
    collisionFree,
    workspace->getPersonPose(),
    distanceToPerson,
    horizontalToleranceForPerson,
    verticalToleranceForPerson,
    planningTimeout,
    maxNumTrials,
    velocityLimits
    );
  /*
  for (std::size_t i = 0; i < 2; ++i)
  {
    moveInFrontOfPerson();
    nodeHandle.setParam("/feeding/facePerceptionOn", true);

    waitForUser("Move towards person");

    moveSuccess = moveTowardsPerson();
    nodeHandle.setParam("/feeding/facePerceptionOn", false);

    if (moveSuccess)
      break;
    ROS_INFO_STREAM("Moved failed, backing up and retrying");
  }

  if (!moveSuccess) {
    ROS_INFO_STREAM("Servoing failed. Falling back to direct movement...");
    moveInFrontOfPerson();
    moveDirectlyToPerson(tilted);
  }*/

  // ===== EATING =====
  // ROS_WARN("Human is eating");
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