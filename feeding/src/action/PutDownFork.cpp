#include "feeding/action/PutDownFork.hpp"
#include "feeding/action/MoveAbove.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/TargetItem.hpp"

namespace feeding {
namespace action {

void putDownFork(
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
  ada->closeHand();
  moveAboveForque(
    ada,
    collisionFree,
    forkHolderAngle,
    forkHolderTranslation,
    planningTimeout,
    maxNumTrials);
  moveInto(TargetItem::FORQUE);

  ada->openHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

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
