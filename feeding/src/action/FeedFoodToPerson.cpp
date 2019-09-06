#include "feeding/action/FeedFoodToPerson.hpp"
#include <libada/util.hpp>
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveDirectlyToPerson.hpp"
#include "feeding/action/MoveInFrontOfPerson.hpp"
#include "feeding/action/MoveTowardsPerson.hpp"
#include "feeding/util.hpp"

using ada::util::getRosParam;
using ada::util::createBwMatrixForTSR;
using aikido::constraint::dart::TSR;

namespace feeding {
  namespace action {
    //==============================================================================
    void feedFoodToPerson(
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
        publishTimingDoneToWeb();
        nodeHandle->setParam("/feeding/facePerceptionOn", true);

        ROS_INFO_STREAM("Move towards person");
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

  // Ask for Tilt Override
      auto overrideTiltOffset = tiltOffset;

      if (!getRosParam<bool>("/humanStudy/autoTransfer", *nodeHandle)) {
        talk("Should I tilt the food item?", false);
        std::string done = "";
        std::string actionTopic;
        nodeHandle->param<std::string>("/humanStudy/actionTopic", actionTopic, "/study_action_msgs");
        done = getInputFromTopic(actionTopic, *nodeHandle, true, -1);

        if (done == "tilt_the_food") {
          std::vector<double> tiltOffsetVector
          = getRosParam<std::vector<double>>("/study/tiltOffset", *nodeHandle);
          auto tiltOffsetEigen = Eigen::Vector3d(
              tiltOffsetVector[0], tiltOffsetVector[1], tiltOffsetVector[2]);
          overrideTiltOffset = &tiltOffsetEigen;
        } else {
          overrideTiltOffset = nullptr;
        }
      }

  // Execute Tilt
      if (overrideTiltOffset != nullptr) {
        Eigen::Isometry3d person = Eigen::Isometry3d::Identity();
        person.translation() += *overrideTiltOffset;

        TSR personTSR;
        personTSR.mT0_w = person;

        personTSR.mBw = createBwMatrixForTSR(
          horizontalToleranceForPerson,
          horizontalToleranceForPerson,
          verticalToleranceForPerson,
          0,
          M_PI / 4,
          -M_PI / 4);
        Eigen::Isometry3d eeTransform = Eigen::Isometry3d::Identity();
        eeTransform.linear()
        = eeTransform.linear() 
        *Eigen::Matrix3d(
          Eigen::AngleAxisd(M_PI * -0.25, Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(M_PI * 0.25, Eigen::Vector3d::UnitX()));
        personTSR.mTw_e.matrix() *= eeTransform.matrix();

    // Actually execute movement
        ada->moveArmToTSR(
          personTSR,
          collisionFree,
          planningTimeout,
          maxNumTrials,
          getConfigurationRanker(ada),
          velocityLimits);
      }

      if (moveIFOSuccess)
      {
    // ===== EATING =====
        ROS_WARN("Human is eating");
        talk("Ready to eat!");
        std::this_thread::sleep_for(waitAtPerson);

    // Backward
        ada::util::waitForUser("Move backward", ada);
        publishTransferDoneToWeb();
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
