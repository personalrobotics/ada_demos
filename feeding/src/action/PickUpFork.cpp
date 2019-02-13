#include "feeding/action/PickUpFork.hpp"
#include "feeding/action/MoveAboveForque.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util/util.hpp"
#include "feeding/TargetItem.hpp"

namespace feeding {
namespace action {

void pickUpFork(
  const std::shared_ptr<ada::Ada>& ada,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  double forkHolderAngle,
  std::vector<double> forkHolderTranslation,
  const Eigen::Isometry3d& plate,
  double heightAbovePlate,
  double horizontalToleranceAbovePlate,
  double verticalToleranceAbovePlate,
  double rotationToleranceAbovePlate,
  double endEffectorOffsetPositionTolerance,
  double endEffectorOffsetAngularTolerance,
  double planningTimeout,
  int maxNumTrials,
  std::vector<double> velocityLimits,
  std::shared_ptr<FTThresholdHelper> ftThresholdHelper)
{
  ada->openHand();
  moveAboveForque(
    ada,
    collisionFree,
    forkHolderAngle,
    forkHolderTranslation,
    planningTimeout,
    maxNumTrials);

  Eigen::Vector3d endEffectorDirection(0, 0, -1);
  moveInto(
    ada,
    collisionFree,
    TargetItem::FORQUE,
    planningTimeout,
    endEffectorOffsetPositionTolerance,
    endEffectorOffsetAngularTolerance,
    endEffectorDirection,
    ftThresholdHelper);

  std::vector<std::string> optionPrompts{"(1) close", "(2) leave-as-is"};
  auto input = getUserInputWithOptions(optionPrompts, "Close Hand?");

  if (input == 1)
  {
    ada->closeHand();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  moveOutOf(
    ada,
    collisionFree,
    TargetItem::FORQUE,
    planningTimeout,
    endEffectorOffsetPositionTolerance,
    endEffectorOffsetAngularTolerance,
    ftThresholdHelper
  );

  moveAbovePlate(
    ada,
    collisionFree,
    plate,
    heightAbovePlate,
    horizontalToleranceAbovePlate,
    verticalToleranceAbovePlate,
    rotationToleranceAbovePlate,
    endEffectorOffsetPositionTolerance,
    endEffectorOffsetAngularTolerance,
    planningTimeout,
    maxNumTrials,
    velocityLimits
  );
}


} // namespace feeding
} // namespace action
