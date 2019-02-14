#include "feeding/action/Skewer.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"

#include <libada/util.hpp>


static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};


namespace feeding {
namespace action {

//==============================================================================
void skewer(
  const std::shared_ptr<ada::Ada>& ada,
  const std::shared_ptr<Workspace>& workspace,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  const std::shared_ptr<Perception>& perception,
  const std::string& foodName,
  const Eigen::Isometry3d& plate,
  const Eigen::Isometry3d& plateEndEffectorTransform,
  const std::unordered_map<std::string, double>& foodSkeweringForces,
  double heightAbovePlate,
  double horizontalToleranceAbovePlate,
  double verticalToleranceAbovePlate,
  double rotationToleranceAbovePlate,
  double heightAboveFood,
  double horizontalToleranceForFood,
  double verticalToleranceForFood,
  double rotationToleranceForFood,
  double tiltToleranceForFood,
  double moveOutofFoodLength,
  double endEffectorOffsetPositionTolerance,
  double endEffectorOffsetAngularTolerance,
  std::chrono::milliseconds waitTimeForFood,
  double planningTimeout,
  int maxNumTrials,
  std::vector<double> velocityLimits,
  const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper)
{
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
    velocityLimits);

  auto itemWithActionScore = detectAndMoveAboveFood(
    ada,
    collisionFree,
    perception,
    foodName,
    heightAboveFood,
    horizontalToleranceForFood,
    verticalToleranceForFood,
    rotationToleranceForFood,
    tiltToleranceForFood,
    planningTimeout,
    maxNumTrials,
    velocityLimits
    );

  for(std::size_t i = 0; i < 3; ++i)
  {
    auto tiltStyle = itemWithActionScore->action.tiltStyle;
    ROS_INFO_STREAM(
        "Getting " << foodName << "with " << foodSkeweringForces.at(foodName)
                               << "N with angle mode " << tiltStyle);

    Eigen::Vector3d endEffectorDirection(0, 0, -1);
    if (tiltStyle == TiltStyle::ANGLED)
    {
      endEffectorDirection = Eigen::Vector3d(0.1, 0, -0.18);
      endEffectorDirection.normalize();

    }
    // ===== INTO FOOD =====
    ada::util::waitForUser("Move forque into food", ada);
    moveInto(
      ada,
      collisionFree,
      TargetItem::FOOD,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance,
      endEffectorDirection,
      ftThresholdHelper);
    grabFood(ada, workspace);
    std::this_thread::sleep_for(waitTimeForFood);

    // ===== OUT OF FOOD =====

    Eigen::Vector3d direction(0, 0, 1);
    bool ignoreCollision = true;
    moveOutOf(
      ada,
      nullptr,
      TargetItem::FOOD,
      moveOutofFoodLength,
      direction,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance,
      ftThresholdHelper
    );

    std::this_thread::sleep_for(waitTimeForFood);
    if (getUserInputWithOptions(optionPrompts, "Did I succeed?") == 1)
    {
      ROS_INFO_STREAM("Successful");
      return;
    }
    ungrabAndDeleteFood(ada, workspace);
    ROS_INFO_STREAM("Try again.");
  }

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
