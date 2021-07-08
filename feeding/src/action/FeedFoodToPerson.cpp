#include "feeding/action/FeedFoodToPerson.hpp"

#include <libada/util.hpp>

#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveTowardsPerson.hpp"
#include "feeding/util.hpp"

using ada::util::createBwMatrixForTSR;
using ada::util::getRosParam;
using aikido::constraint::dart::TSR;

namespace feeding {
namespace action {

static const std::vector<std::string> optionPrompts{"(1) tilt", "(2) no tilt"};
//==============================================================================
void feedFoodToPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    ros::Duration waitAtPerson,
    FeedingDemo* feedingDemo,
    const Eigen::Vector6d& jointVelocityLimits,
    // Visual Servoing Params
    double servoVelocityLimit,
    double distanceFromPerson,
    // Returning to plate params
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    // Tilting params
    const Eigen::Vector3d* tiltOffset,
    double horizontalToleranceForPerson,
    double verticalToleranceForPerson,
    double planningTimeout,
    int maxNumTrials)
{

  // Step 1: Move In Front Of Person
  auto moveIFOPerson = [&] {
    return moveInFrontOfPerson(
        ada,
        collisionFree,
        jointVelocityLimits,
        feedingDemo);
  };
  if (!moveIFOPerson())
  {
    ROS_WARN_STREAM("Failed to move in front of person");
    talk("Sorry, I'm having a little trouble moving to you.");
    return;
  }

  // HS: Send message to web interface to indicate skewer finished
  publishActionDoneToWeb((ros::NodeHandle*)nodeHandle);

  // HS: Ask for Tilt Override
  auto overrideTiltOffset = tiltOffset;

  if (!getRosParam<bool>("/humanStudy/autoTransfer", *nodeHandle))
  {
    talk("Should I tilt the food item?", false);
    std::string done = "";
    std::string actionTopic;
    nodeHandle->param<std::string>(
        "/humanStudy/actionTopic", actionTopic, "/study_action_msgs");
    done = getInputFromTopic(actionTopic, *nodeHandle, false, -1);

    if (done == "tilt_the_food" || done == "tilt")
    {
      std::vector<double> tiltOffsetVector
          = getRosParam<std::vector<double>>("/study/tiltOffset", *nodeHandle);
      auto tiltOffsetEigen = Eigen::Vector3d(
          tiltOffsetVector[0], tiltOffsetVector[1], tiltOffsetVector[2]);
      overrideTiltOffset = &tiltOffsetEigen;
    }
    else if (done == "continue")
    {
      overrideTiltOffset = nullptr;
    }
    else
    {
      if (getUserInputWithOptions(optionPrompts, "Not valid, should I tilt??")
          == 1)
      {
        std::vector<double> tiltOffsetVector = getRosParam<std::vector<double>>(
            "/study/tiltOffset", *nodeHandle);
        auto tiltOffsetEigen = Eigen::Vector3d(
            tiltOffsetVector[0], tiltOffsetVector[1], tiltOffsetVector[2]);
        overrideTiltOffset = &tiltOffsetEigen;
      }
      else
      {
        overrideTiltOffset = nullptr;
      }
    }
  }

  publishTransferDoneToWeb((ros::NodeHandle*)nodeHandle);

  // HS: Check autoTiming, and if false, wait for topic
  if (!getRosParam<bool>("/humanStudy/autoTiming", *nodeHandle))
  {
    talk("Let me know when you are ready.", false);
    std::string done = "";
    while (done != "continue")
    {
      std::string actionTopic;
      nodeHandle->param<std::string>(
          "/humanStudy/actionTopic", actionTopic, "/study_action_msgs");
      done = getInputFromTopic(actionTopic, *nodeHandle, false, -1);
    }
  }
  else
  {
    nodeHandle->setParam("/feeding/facePerceptionOn", true);
    talk("Open your mouth when ready.", false);
    // TODO: Add mouth-open detection.
    while (true)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (perception->isMouthOpen())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (perception->isMouthOpen())
        {
          break;
        }
      }
    }
    nodeHandle->setParam("/feeding/facePerceptionOn", false);

    if (getRosParam<bool>("/humanStudy/createError", *nodeHandle))
    {
      // Wait an extra 5 seconds
      ROS_WARN_STREAM("Error Requested for Timing!");
      talk("Calculating...");
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
  }

  // HS: Check for Error in Bite Transfer
  if (getRosParam<bool>("/humanStudy/autoTransfer", *nodeHandle)
      && getRosParam<bool>("/humanStudy/createError", *nodeHandle))
  {
    ROS_WARN_STREAM("Error Requested for Transfer!");
    // Erroneous Transfer
    Eigen::VectorXd moveErrorPose(6);
    moveErrorPose << -2.78470, 4.57978, 4.86316, -3.05050, 1.89748, -0.6150;
    ada->moveArmToConfiguration(moveErrorPose, nullptr, 2.0, jointVelocityLimits);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    talk("Oops, let me try that again.", true);
    moveIFOPerson();
  }

  // Step 2: Enable Face Detection
  nodeHandle->setParam("/feeding/facePerceptionOn", true);

  // If tilting, position further away from face.
  if (overrideTiltOffset)
  {
    distanceFromPerson += overrideTiltOffset->norm();
  }

  // Step 3: Execute visual Servoing
  ROS_INFO_STREAM("Move towards person");
  bool moveSuccess = moveTowardsPerson(
      ada,
      perception,
      nodeHandle,
      feedingDemo,
      distanceFromPerson,
      servoVelocityLimit);

  // No longer nead face perception
  nodeHandle->setParam("/feeding/facePerceptionOn", false);

  // Do not tilt if servoing fails (e.g. because user is touching fork)
  if (!moveSuccess) {
    ROS_WARN_STREAM("Unsuccessful servoing (FT threshold?), we probably shouldn't tilt.");
    overrideTiltOffset = nullptr;
  } else {
    ROS_WARN_STREAM("Visual servoing completed!");
  }

  // Step 4: Execute Tilt
  if (overrideTiltOffset != nullptr)
  {

    Eigen::Isometry3d person
        = ada->getHand()->getEndEffectorBodyNode()->getTransform();
    person.translation() += *overrideTiltOffset;

    TSR personTSR;
    personTSR.mT0_w = person;

    personTSR.mBw = createBwMatrixForTSR(
        horizontalToleranceForPerson,
        horizontalToleranceForPerson,
        verticalToleranceForPerson,
        0,
        M_PI / 8,
        M_PI / 8);
    Eigen::Isometry3d eeTransform = Eigen::Isometry3d::Identity();
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                Eigen::AngleAxisd(M_PI * -0.25, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(M_PI * 0.25, Eigen::Vector3d::UnitX()));
    personTSR.mTw_e.matrix() *= eeTransform.matrix();

    // Actually execute movement
    // if (feedingDemo && feedingDemo->getViewer())
    // {
    //   feedingDemo->getViewer()->addTSRMarker(personTSR);
    //   std::cout << "Check TSR" << std::endl;
    //   int n;
    //   std::cin >> n;
    // }
    Eigen::Vector6d slowerVelocity = Eigen::Vector6d(jointVelocityLimits);
    double slowFactor = (jointVelocityLimits[0] > 0.5) ? 2.0 : 1.0;
    slowerVelocity /= slowFactor;

    talk("Tilting, hold tight.", true);

    ada->moveArmToTSR(
        personTSR,
        nullptr,
        planningTimeout,
        maxNumTrials,
        getConfigurationRanker(ada),
        slowerVelocity);
  }

  // Step 5: EATING
  ROS_WARN("Human is eating");
  talk("Ready to eat!");
  ROS_WARN("Waiting for person...");
  waitAtPerson.sleep();

  // Step 6: Return to fronto of person
  ada::util::waitForUser("Move backward", ada);
  talk("Let me get out of your way.", true);
  if (!moveInFrontOfPerson(ada, nullptr, jointVelocityLimits, feedingDemo)) {
    ROS_WARN_STREAM("Failed to move in front of person");
    talk("Sorry, I'm having a little trouble moving away from you.");
    return;
  }

  // Step 7: Back above the plate
  ROS_INFO_STREAM("Move back to plate");
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
      jointVelocityLimits);

  publishTimingDoneToWeb((ros::NodeHandle*)nodeHandle);
}

} // namespace action
} // namespace feeding
