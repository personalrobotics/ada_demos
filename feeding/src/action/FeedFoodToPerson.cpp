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
      const Eigen::Vector3d* tiltOffset,
      FeedingDemo* feedingDemo)
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

      // Check autoTiming, and if false, wait for topic
      if (!getRosParam<bool>("/humanStudy/autoTiming", *nodeHandle)) {
        talk("Let me know when you are ready.", false);
        std::string done = "";
        while (done != "continue") {
            std::string actionTopic;
            nodeHandle->param<std::string>("/humanStudy/actionTopic", actionTopic, "/study_action_msgs");
            done = getInputFromTopic(actionTopic, *nodeHandle, false, -1);
        }
      } else {
        nodeHandle->setParam("/feeding/facePerceptionOn", true);
        talk("Open your mouth when ready.", false);
        // TODO: Add mouth-open detection.
        while(!perception->isMouthOpen()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        nodeHandle->setParam("/feeding/facePerceptionOn", false);

        if(getRosParam<bool>("/humanStudy/createError", *nodeHandle)) {
          // Wait an extra 5 seconds
          ROS_WARN_STREAM("Error Requested for Timing!");
          talk("Calculating...");
          std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        }
      }

      if (moveIFOSuccess)
      {
        publishTimingDoneToWeb();

        if(getRosParam<bool>("/humanStudy/autoTransfer", *nodeHandle) &&
          getRosParam<bool>("/humanStudy/createError", *nodeHandle)) {
          ROS_WARN_STREAM("Error Requested for Transfer!");
          // Erroneous Transfer
          moveDirectlyToPerson(
            ada,
            collisionFreeWithWallFurtherBack,
            personPose,
            distanceToPerson,
            horizontalToleranceForPerson,
            verticalToleranceForPerson,
            planningTimeout,
            maxNumTrials,
            velocityLimits,
            nullptr,
            feedingDemo
            );
          std::this_thread::sleep_for(std::chrono::milliseconds(3000));
          talk("Oops, let me try that again.", true);
          moveIFOSuccess = moveInFrontOfPerson(
            ada,
            collisionFreeWithWallFurtherBack,
            personPose,
            distanceToPerson,
            horizontalToleranceForPerson,
            verticalToleranceForPerson,
            planningTimeout,
            maxNumTrials,
            velocityLimits);
        }

        nodeHandle->setParam("/feeding/facePerceptionOn", true);

        ROS_INFO_STREAM("Move towards person");
        moveSuccess = moveTowardsPerson(
          ada,
          nullptr,
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
        done = getInputFromTopic(actionTopic, *nodeHandle, false, -1);

        if (done == "tilt_the_food" || done == "tilt") {
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
        Eigen::Isometry3d person = ada->getHand()->getEndEffectorBodyNode()->getTransform();
        person.translation() += *overrideTiltOffset;

        TSR personTSR;
        personTSR.mT0_w = person;

        personTSR.mBw = createBwMatrixForTSR(
          horizontalToleranceForPerson,
          horizontalToleranceForPerson,
          verticalToleranceForPerson,
          0,
          M_PI / 4,
          M_PI / 4);
        Eigen::Isometry3d eeTransform = Eigen::Isometry3d::Identity();
        eeTransform.linear()
        = eeTransform.linear() 
        *Eigen::Matrix3d(
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
        std::vector<double> slowerVelocity;
        double slowFactor = (velocityLimits[0] > 0.5) ? 2.0 : 1.0;
        for (int i=0; i<velocityLimits.size(); i++) {
          slowerVelocity.push_back(velocityLimits[i] / slowFactor);
        }
        talk("Tilting, hold tight.", true);
        ada->moveArmToTSR(
          personTSR,
          nullptr, //collisionFreeWithWallFurtherBack,
          planningTimeout,
          maxNumTrials,
          getConfigurationRanker(ada),
          slowerVelocity);
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
