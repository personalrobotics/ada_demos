#include "feeding/action/MoveToPerson.hpp"
#include "feeding/util.hpp"
#include <libada/util.hpp>

using aikido::constraint::dart::TSR;
using ada::util::createBwMatrixForTSR;

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

bool moveDirectlyToPerson(
  const std::shared_ptr<ada::Ada>& ada,
  const aikido::constraint::dart::CollisionFreePtr& collisionFree,
  Eigen::Isometry3d personPose,
  bool tilted,
  double distanceToPerson,
  double horizontalTolerance,
  double verticalTolerance,
  double planningTimeout,
  int maxNumTrials,
  std::vector<double> velocityLimits)
{
  // double distanceToPerson = 0.02;
  // double horizontalToleranceNearPerson = getRosParam<double>(
  //     "/planning/tsr/horizontalToleranceNearPerson", mNodeHandle);
  // double verticalToleranceNearPerson = getRosParam<double>(
  //     "/planning/tsr/verticalToleranceNearPerson", mNodeHandle);

  // Eigen::Isometry3d personPose = createIsometry(
      // getRosParam<std::vector<double>>("/study/personPose", mNodeHandle));
  if (tilted)
  {
    std::vector<double> tiltOffsetVector
        = getRosParam<std::vector<double>>("/study/tiltOffset", mNodeHandle);
    Eigen::Vector3d tiltOffset{
        tiltOffsetVector[0], tiltOffsetVector[1], tiltOffsetVector[2]};
    personPose.translation() += tiltOffset;
  }

  TSR personTSR;
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  if (tilted)
  {
    personTSR.mBw = createBwMatrixForTSR(
        horizontalToleranceNearPerson,
        verticalToleranceNearPerson,
        -M_PI / 4,
        M_PI / 4);
    Eigen::Isometry3d eeTransform
        = ada->getHand()->getEndEffectorTransform("person");
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                Eigen::AngleAxisd(M_PI * -0.25, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(M_PI * 0.25, Eigen::Vector3d::UnitX()));
    personTSR.mTw_e.matrix() *= eeTransform.matrix();
  }
  else
  {
    personTSR.mBw = createBwMatrixForTSR(
        horizontalToleranceNearPerson, verticalToleranceNearPerson, 0, 0);
    personTSR.mTw_e.matrix()
        *= ada->getHand()->getEndEffectorTransform("person")->matrix();
  }

  if (!ada->moveArmToTSR(
      personTSR,
      collisionFree,
      planningTimeout,
      maxNumTrials,
      util::getConfigurationRanker(ada),
      velocityLimits,
      ada::TrajectoryPostprocessType::KUNZ))
  {
    ROS_WARN_STREAM("Execution failed");
  }
}

}
}
