#include "feeding/action/MoveAbove.hpp"
#include <libada/util.hpp>
#include "feeding/util.hpp"
using aikido::constraint::dart::TSR;
using ada::util::createBwMatrixForTSR;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::CollisionFreePtr;

// Contains motions which are mainly TSR actions
namespace feeding {

namespace action {


bool moveAbove(
    const std::shared_ptr<::ada::Ada>& ada,
    const CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& targetTransform,
    const Eigen::Isometry3d& endEffectorTransform,
    double horizontalTolerance,
    double verticalTolerance,
    double rotationTolerance,
    double tiltTolerance,
    double planningTimeout,
    int maxNumTrials,
    const std::vector<double>& velocityLimits,
    FeedingDemo* feedingDemo)
{
  TSR target;

  target.mT0_w = targetTransform;
  target.mBw = createBwMatrixForTSR(
      horizontalTolerance,
      horizontalTolerance,
      verticalTolerance,
      0,
      tiltTolerance,
      rotationTolerance);

  target.mTw_e.matrix() = endEffectorTransform.matrix();

  try
  {
    auto trajectoryCompleted = ada->moveArmToTSR(
        target,
        collisionFree,
        planningTimeout,
        maxNumTrials,
        getConfigurationRanker(ada),
        velocityLimits,
        ::ada::TrajectoryPostprocessType::KUNZ);
    return trajectoryCompleted;
  }
  catch (...)
  {
    ROS_WARN("Error in trajectory completion!");
    return false;
  }
}

} // namespace action
} // namespace feeding
