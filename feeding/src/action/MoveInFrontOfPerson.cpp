#include "feeding/action/MoveInFrontOfPerson.hpp"

#include <libada/util.hpp>

#include "feeding/util.hpp"

using ada::util::createBwMatrixForTSR;
using aikido::constraint::dart::TSR;

using aikido::constraint::dart::TSR;

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

bool moveInFrontOfPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    std::vector<double> velocityLimits,
    FeedingDemo* feedingDemo)
{
  ROS_INFO_STREAM("move in front of person");

  // hardcoded pose in front of person
  Eigen::VectorXd moveIFOPose(6);

  // Wheelchair
  moveIFOPose << -2.30252, 4.23221, 3.84109, -4.65546, 3.94225, 4.26543;

  // Tripod
  // moveIFOPose << -1.81753, 4.32404, 4.295815, 3.12878, 1.89724, -0.61526;

  // Participant
  // moveIFOPose << -2.30293, 4.04904, 3.63059, 1.62787, -2.34089, -2.01773;

  // Participant Tripod
  // moveIFOPose << -1.81752, 4.60286, 4.64300, -3.05122, 1.89743, -0.61493;

  return ada->moveArmToConfiguration(
      moveIFOPose, collisionFree, 2.0, velocityLimits);
}
} // namespace action
} // namespace feeding
