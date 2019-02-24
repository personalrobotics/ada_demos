#include "feeding/action/MoveDirectlyToPerson.hpp"
#include <libada/util.hpp>
#include "feeding/util.hpp"

using aikido::constraint::dart::TSR;
using ada::util::createBwMatrixForTSR;

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

// TODO: This gets to MoveInfrontOfPerson pose. need moveInto?
bool moveDirectlyToPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& personPose,
    double distanceToPerson,
    double horizontalToleranceForPerson,
    double verticalToleranceForPerson,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits,
    const Eigen::Vector3d* tiltOffset,
    FeedingDemo* feedingDemo)
{
  Eigen::Isometry3d person(personPose);
  if (tiltOffset)
  {
    person.translation() += *tiltOffset;
  }

  TSR personTSR;
  personTSR.mT0_w = person;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  if (tiltOffset)
  {
    personTSR.mBw = createBwMatrixForTSR(
        horizontalToleranceForPerson,
        horizontalToleranceForPerson,
        verticalToleranceForPerson,
        0,
        M_PI / 4,
        -M_PI / 4);
    Eigen::Isometry3d eeTransform
        = ada->getHand()->getEndEffectorTransform("person").get();
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
        horizontalToleranceForPerson,
        horizontalToleranceForPerson,
        verticalToleranceForPerson,
        0,
        0,
        0);
    personTSR.mTw_e.matrix()
        *= ada->getHand()->getEndEffectorTransform("person")->matrix();
  }

  if(feedingDemo)
  {
    feedingDemo->getViewer()->addTSRMarker(personTSR);
    std::cout << "check person TSR" << std::endl;
    int n;
    std::cin >> n;
  }

  if (!ada->moveArmToTSR(
          personTSR,
          collisionFree,
          planningTimeout,
          maxNumTrials,
          getConfigurationRanker(ada),
          velocityLimits,
          ada::TrajectoryPostprocessType::KUNZ))
  {
    ROS_WARN_STREAM("Execution failed");
    return false;
  }
  return true;
}
}
}
