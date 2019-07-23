#include "feeding/action/MoveInFrontOfPerson.hpp"
#include <libada/util.hpp>
#include "feeding/util.hpp"

using aikido::constraint::dart::TSR;
using ada::util::createBwMatrixForTSR;

using aikido::constraint::dart::TSR;

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

bool moveInFrontOfPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& workspacePersonPose,
    double distanceToPerson,
    double horizontalToleranceForPerson,
    double verticalToleranceForPerson,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits,
    FeedingDemo* feedingDemo)
{
  ROS_INFO_STREAM("move in front of person");

  // hardcoded pose in front of person
  Eigen::VectorXd moveIFOPose(6);
  moveIFOPose << -2.30252, 4.23221, 3.84109, -4.65546, 3.94225, 4.26543;

  bool success = ada->moveArmToConfiguration(moveIFOPose, collisionFree, 2.0);
  if (success)
    return true;

  TSR personTSR;
  Eigen::Isometry3d personPose = Eigen::Isometry3d::Identity();
  personPose.translation() = workspacePersonPose.translation();
  personPose.linear()
      = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  personTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceForPerson,
      horizontalToleranceForPerson,
      verticalToleranceForPerson,
      0,
      0,
      0);
  personTSR.mTw_e.matrix()
      *= ada->getHand()->getEndEffectorTransform("person")->matrix();

  return ada->moveArmToTSR(
      personTSR,
      collisionFree,
      planningTimeout,
      maxNumTrials,
      getConfigurationRanker(ada),
      velocityLimits);
}
}
}
