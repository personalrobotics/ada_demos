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

  // if(feedingDemo)
  // {
  //   feedingDemo->getViewer()->addTSRMarker(personTSR);
  //   int n;
  //   std::cin >> n;
  // }

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
