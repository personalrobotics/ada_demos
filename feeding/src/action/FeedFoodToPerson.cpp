#include "feeding/action/FeedFoodToPerson.hpp"
#include <libada/util.hpp>
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveTowardsPerson.hpp"
#include "feeding/util.hpp"
#include "std_msgs/String.h"

namespace feeding {
namespace action {

//==============================================================================
void feedFoodToPerson(
    int trialType,
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Workspace>& workspace,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const aikido::constraint::dart::CollisionFreePtr&
        collisionFreeWithWallFurtherBack,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    const Eigen::Isometry3d& personPose,
    std::chrono::milliseconds waitAtPerson,
    double heightAbovePlate,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    double distanceToPerson,
    double horizontalToleranceForPerson,
    double verticalToleranceForPerson,
    double planningTimeout,
    int maxNumTrials,
    double endEffectorOffsetPositionTolerenace,
    double endEffectorOffsetAngularTolerance,
    std::vector<double> velocityLimits,
    const Eigen::Vector3d* tiltOffset)
{
  talk("Great! Bringing the food to you!", true);
  auto moveIFOPerson = [&] {
    return moveInFrontOfPerson(
        ada,
        collisionFree,
        personPose,
        distanceToPerson,
        horizontalToleranceForPerson,
        verticalToleranceForPerson,
        planningTimeout,
        maxNumTrials,
        velocityLimits);
  };

  bool moveIFOSuccess = false;
  bool moveSuccess = false;
  for (std::size_t i = 0; i < 2; ++i)
  {
    moveIFOSuccess = moveIFOPerson();
    if (!moveIFOSuccess)
    {
      ROS_WARN_STREAM("Failed to move in front of person, retry");
      talk("Sorry, I'm having a little trouble moving. Let me try again.");
      continue;
    }
    else
      break;
  }

  if (moveIFOSuccess)
  {
    nodeHandle->setParam("/feeding/facePerceptionOn", true);

    ROS_INFO_STREAM("Move towards person");
    talk("Ready? Make sure I can see your face.");

    // TODO: Consult Ethan to Fix Wrong partial start time issue
    // Send message to web interface to indicate skweweing finished
    ros::NodeHandle timingHandle;
    ros::Publisher timingPub = timingHandle.advertise<std_msgs::String>("/timing_done", 1, true);
    std_msgs::String msg;
    msg.data = "timing done";
    timingPub.publish(msg);

    if (trialType != AUTO && trialType != TR_AUTO) {
      talk("Let me know when you are ready to eat! ");
      std::string feedAngle = getFeedAngleFromAlexa();
    }

    moveSuccess = moveTowardsPerson(
        ada,
        collisionFreeWithWallFurtherBack,
        perception,
        nodeHandle,
        distanceToPerson,
        planningTimeout,
        endEffectorOffsetPositionTolerenace,
        endEffectorOffsetAngularTolerance);
    nodeHandle->setParam("/feeding/facePerceptionOn", false);
  }

  if (moveIFOSuccess)
  {
    // ===== EATING =====
    ROS_WARN("Human is eating");
    std::this_thread::sleep_for(waitAtPerson);
    

    // Backward
    ada::util::waitForUser("Move backward", ada);
    talk("Let me get out of your way.", true);
    Eigen::Vector3d goalDirection(0, -1, 0);
    bool success = ada->moveArmToEndEffectorOffset(
        goalDirection.normalized(),
        0.1,
        nullptr, // collisionFreeWithWallFurtherBack,
        planningTimeout,
        endEffectorOffsetPositionTolerenace * 2,
        endEffectorOffsetAngularTolerance * 2);

    ROS_INFO_STREAM("Backward " << success << std::endl);
  }

  // ===== BACK TO PLATE =====
  ROS_INFO_STREAM("Move back to plate");

  // TODO: add a back-out motion and then do move above plate with
  // collisionFree.
  talk("And now back to the plate.", true);
  moveAbovePlate(
      ada,
      collisionFree,
      plate,
      plateEndEffectorTransform,
      horizontalToleranceAbovePlate,
      verticalToleranceAbovePlate,
      rotationToleranceAbovePlate,
      planningTimeout,
      maxNumTrials,
      velocityLimits);
}
} // namespace action
} // namespace feeding
