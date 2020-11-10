#include "feeding/action/MoveAbove.hpp"
#include <libada/util.hpp>
#include "feeding/util.hpp"
using ada::util::createBwMatrixForTSR;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::constraint::dart::TSR;

// Contains motions which are mainly TSR actions
namespace feeding {

namespace action {

bool moveAbove(
    const std::shared_ptr<::ada::Ada>& ada,
    const CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& targetTransform, // T0_w: the pose of TSR frame in world frame 0.
    const Eigen::Isometry3d& endEffectorTransform, // Tw_e: the pose of EE in TSR frame w.
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
  target.mBw = createBwMatrixForTSR( // Bw: tolerance Matrix
      horizontalTolerance,
      horizontalTolerance,
      verticalTolerance,
      0,
      tiltTolerance,
      rotationTolerance);

  target.mTw_e.matrix() = endEffectorTransform.matrix();
  // if (feedingDemo && feedingDemo->getViewer())
  // {
  //   feedingDemo->getViewer()->addTSRMarker(target);
  //   std::cout << "Check TSR" << std::endl;
  //   int n;
  //   std::cin >> n;
  // }

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

    std::cout << "MoveAbove Current pose \n"
              << ada->getMetaSkeleton()->getPositions().transpose()
              << std::endl;

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
