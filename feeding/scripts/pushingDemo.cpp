#include <ros/ros.h>
#include <libada/util.hpp>
#include <sstream>
#include "std_msgs/String.h"

#include "feeding/FeedingDemo.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveAboveFood.hpp"
#include "feeding/action/MoveAbove.hpp"

#include "feeding/util.hpp"
#include "feeding/FTThresholdHelper.hpp"

#include "feeding/action/Push.hpp"
#include "feeding/action/TouchTable.hpp"
#include "feeding/action/LiftUp.hpp"
#include "feeding/action/MoveAboveAndScoop.hpp"
#include "feeding/action/PolyScoop.hpp"

#include "feeding/FoodItem.hpp"

#include "aikido/perception/shape_conversions.hpp"
#include <yaml-cpp/exceptions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using ada::util::waitForUser;
using ada::util::getRosParam;

namespace feeding {

void pushingDemo(
  FeedingDemo& feedingDemo, 
  std::shared_ptr<Perception>& perception, 
  ros::NodeHandle nodeHandle)
{
  // TODO: positioning the hand above the plate
  ROS_INFO_STREAM("========== Pushing DEMO ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();
  
  std::shared_ptr<FTThresholdHelper> FTThresholdHelper = feedingDemo.getFTThresholdHelper();

  while (true)
  {
    waitForUser("Start?", ada);

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ROS_INFO_STREAM("Running Pushing demo");

    // ===== MOVE ABOVE PLATE =====
    ROS_INFO_STREAM("Move above plate");
    bool abovePlaceSuccess = action::moveAbovePlate(
        ada,
        collisionFree,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters["horizontalTolerance"],
        feedingDemo.mPlateTSRParameters["verticalTolerance"],
        feedingDemo.mPlateTSRParameters["rotationTolerance"],
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits);

    if (!abovePlaceSuccess)
    {
      ROS_WARN_STREAM("Move above plate failed. Please restart");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "Move above Place Success" << std::endl;
    }

    //--------------------------------------------------------------------------------------------------------
    // Detect Food Item 

    waitForUser("Ready to Detect?", ada);

    bool useSimFood = getRosParam<bool>("/pushingDemo/useSimFood", nodeHandle);

    bool pushFlag = false;
    std::string foodName = "carrot";
    Eigen::Isometry3d foodTransform;
    std::string push_direction;
    const AcquisitionAction* mAction;

    if (!useSimFood)
    {
        std::vector<std::unique_ptr<FoodItem>> candidateItems;
        while (true)
        {
          // Perception returns the list of good candidates, any one of them is good.
          // Multiple candidates are preferrable since planning may fail.
          candidateItems = perception->perceiveFood(foodName);
          if (candidateItems.size() == 0)
          {
            ROS_WARN_STREAM(
                "Failed to detect any food. Please place food on the plate.");
          }
          else
            break;
          std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // choose the first food in candidateItems and set it to be tracked.
        auto& item = candidateItems[0];
        mAction = item->getAction();
        foodTransform = item->getPose();
        // Q: is the pose there same as the pose that we received directly from MarkerArray.

        perception->setFoodItemToTrack(item.get());
        
        // tell whether should we do push?
        // push_direction = item->getYamlNode()["push_direction"].as<std::string>();
        // push_direction = "left";
        push_direction = getRosParam<std::string>("/pushingDemo/push_direction", nodeHandle);

    }
    else
    {
        visualization_msgs::MarkerArrayConstPtr markerMessage
        = ros::topic::waitForMessage<visualization_msgs::MarkerArray>(
            "/food_detector/marker_array", nodeHandle, ros::Duration(getRosParam<double>("/perception/timeoutSeconds", nodeHandle)));     
        auto markerTransform = markerMessage->markers[0];
               
        foodTransform = aikido::perception::convertROSPoseToEigen(markerTransform.pose);
        std::cout << foodTransform.translation() << std::endl;
        auto mYamlNode = YAML::Load(markerTransform.text);
        push_direction = mYamlNode["push_direction"].as<std::string>();
    }

    //--------------------------------------------------------------------------------------------------------
    //  PUSH FOOD 
    if (push_direction == "left" || push_direction == "right" || push_direction == "up" || push_direction == "down")
      pushFlag = true;
    else
      std::cout << "No pushing" << std::endl;   

    if (pushFlag)
    {
      ROS_INFO_STREAM("Detect and Move above food");

      // bool moveAboveSuccessful = action::moveAboveFood(
      //       ada,
      //       collisionFree,
      //       foodName,
      //       foodTransform,
      //       mAction->getRotationAngle(),
      //       mAction->getTiltStyle(),
      //       feedingDemo.mPlateTSRParameters["height"],
      //       feedingDemo.mPlateTSRParameters["horizontalTolerance"],
      //       feedingDemo.mPlateTSRParameters["verticalTolerance"],
      //       feedingDemo.mPlateTSRParameters["rotationTolerance"],
      //       feedingDemo.mFoodTSRParameters["tiltTolerance"],
      //       feedingDemo.mPlanningTimeout,
      //       feedingDemo.mMaxNumTrials,
      //       feedingDemo.mVelocityLimits);

      Eigen::Isometry3d backTransform
        = feedingDemo.getAda()->getHand()->getEndEffectorTransform("plate").get();
      // = feedingDemo.getPlateEndEffectorTransform();
      backTransform.translation() = Eigen::Vector3d(-0.03, 0.03, feedingDemo.mPlateTSRParameters["height"]);

      bool moveAboveSuccessful = action::moveAbove(
              ada,
              feedingDemo.getCollisionConstraint(),
              foodTransform,
              backTransform,
              0.01,
              0.01,
              0.1,
              0.00,
              feedingDemo.mPlanningTimeout,
              feedingDemo.mMaxNumTrials,
              feedingDemo.mVelocityLimits);


      ROS_INFO_STREAM("Down to plate");
      // std::shared_ptr<FTThresholdHelper> FTThresholdHelper = nullptr;
      float angle = 0;
      // nodeHandle.getParam("/feedingDemo/pushingAngle", angle);

      double downLength = getRosParam<double>("/pushingDemo/downLength", nodeHandle);
      double pushLength = getRosParam<double>("/pushingDemo/pushLength", nodeHandle);
      double liftLength = getRosParam<double>("/pushingDemo/liftLength", nodeHandle);

      std::cout << "length to go down = "<< downLength << std::endl;   
      std::cout << "length to push = "<< pushLength << std::endl;   
      std::cout << "length to lift up = "<< liftLength << std::endl;   

      waitForUser("0. Start ???? ", ada);

      // ===== Move Back a little bit.
      // bool trajectoryCompleted = ada->moveArmToEndEffectorOffset(
      //                               Eigen::Vector3d(0, 1, 0),
      //                               0.03,
      //                               collisionFree,
      //                               feedingDemo.mPlanningTimeout,
      //                               feedingDemo.mPlateTSRParameters["verticalTolerance"],
      //                               feedingDemo.mPlateTSRParameters["rotationTolerance"],
      //                               feedingDemo.mVelocityLimits);
      // if (!trajectoryCompleted)
      // {
      //   ROS_WARN_STREAM("Move Back failed. Please restart");
      //   exit(EXIT_FAILURE);
      // }
      // else
      // {
      //   waitForUser("1. Success to Move Back ^_^ !!!", ada);
      // }   

      // ===== DOWN TO PLATE =====
      // bool moveDownSuccess = action::touchTable(
      //                         ada,
      //                         collisionFree,
      //                         feedingDemo.mPlanningTimeout,
      //                         feedingDemo.mPlateTSRParameters["verticalTolerance"],
      //                         feedingDemo.mPlateTSRParameters["rotationTolerance"],
      //                         FTThresholdHelper,
      //                         feedingDemo.mVelocityLimits,
      //                         angle,
      //                         downLength);

      Eigen::VectorXd twists(6);
      twists << 0.0, 0.0, 0.5 * M_PI, 0.0, 0.0, -downLength;
      bool moveDownSuccess = ada->moveArmWithEndEffectorTwist(
                                twists,
                                1,
                                collisionFree,
                                feedingDemo.mPlanningTimeout,
                                feedingDemo.mPlateTSRParameters["verticalTolerance"],
                                feedingDemo.mPlateTSRParameters["rotationTolerance"],
                                feedingDemo.mVelocityLimits);

      if (!moveDownSuccess)
      {
        ROS_WARN_STREAM("Move down failed. Please restart");
        exit(EXIT_FAILURE);
      }
      else
      {
          waitForUser("2. Success to Move Down ^_^ !!!", ada);
      }

      ROS_INFO_STREAM("Push");
      // bool pushSuccess = action::push(
      //                         ada,
      //                         collisionFree,
      //                         feedingDemo.mPlanningTimeout,
      //                         feedingDemo.mPlateTSRParameters["verticalTolerance"],
      //                         feedingDemo.mPlateTSRParameters["rotationTolerance"],
      //                         FTThresholdHelper,
      //                         feedingDemo.mVelocityLimits,
      //                         angle);

      // bool pushSuccess = ada->moveArmToEndEffectorOffset(
      //                               Eigen::Vector3d(0, -1, 0),
      //                               0.1,
      //                               collisionFree,
      //                               feedingDemo.mPlanningTimeout,
      //                               feedingDemo.mPlateTSRParameters["verticalTolerance"],
      //                               feedingDemo.mPlateTSRParameters["rotationTolerance"],
      //                               feedingDemo.mVelocityLimits);

      Eigen::VectorXd twists2(6);
      // twists2 << 0.0, 0.0, 0.0, 0, -0.1, 0;
      twists2 << 0.0, 0.0, 0.0, pushLength, 0, 0;

      bool pushSuccess = ada->moveArmWithEndEffectorTwist(
                                twists2,
                                1,
                                collisionFree,
                                feedingDemo.mPlanningTimeout,
                                feedingDemo.mPlateTSRParameters["verticalTolerance"],
                                feedingDemo.mPlateTSRParameters["rotationTolerance"],
                                getRosParam<std::vector<double>>("/pushingDemo/pushVelocityLimits", nodeHandle));
                                // feedingDemo.mVelocityLimits);

      if (!pushSuccess)
      {
        ROS_WARN_STREAM("Push failed. Please restart");
        exit(EXIT_FAILURE);
      }
      else
      {
        waitForUser("3. Success to Push ^_^ !!!", ada);
      }      


      Eigen::VectorXd twists3(6);
      twists3 << 0.0, 0.0, 0.0, 0.0, 0.0, liftLength;

      bool trajectoryCompleted = ada->moveArmWithEndEffectorTwist(
                                twists3,
                                1,
                                collisionFree,
                                feedingDemo.mPlanningTimeout,
                                feedingDemo.mPlateTSRParameters["verticalTolerance"],
                                feedingDemo.mPlateTSRParameters["rotationTolerance"],
                                feedingDemo.mVelocityLimits);

      // action::liftUp(
      //               ada,
      //               collisionFree,
      //               feedingDemo.mPlanningTimeout,
      //               feedingDemo.mPlateTSRParameters["verticalTolerance"],
      //               feedingDemo.mPlateTSRParameters["rotationTolerance"],
      //               FTThresholdHelper,
      //               feedingDemo.mVelocityLimits,
      //               0.05);
      
      waitForUser("4. Success to liftUp ^_^ !!!", ada);

    }
    
    //--------------------------------------------------------------------------------------------------------
    //  Scoop Food

    else
    {
      // do scooping or skewering and nodeHandle.setParam("/pushingDemo/allow_push_img", True) 
      // TODO:
      // 1. MoveAboveFood for scooping
      // action::moveAboveAndScoop(ada, item, feedingDemo, nodeHandle);
      char input_flag;      
      std::cout<<"Scoop or not? "<< std::endl;
      std::cin >> input_flag;

      if ((input_flag == 'Y') || (input_flag == 'y'))
      {
          // std::cout << "height = " << feedingDemo.mPlateTSRParameters["height"] << std::endl;
          // moveAboveAndScoop(ada, foodTransform, feedingDemo, nodeHandle);

          ScoopHelper scoopHelper = ScoopHelper(nodeHandle, foodTransform);
          // double scoop_mode = feedingDemo.mScoopParameters["mode"];
          // double height = feedingDemo.mScoopParameters["height"];
          // double minima = feedingDemo.mScoopParameters["minima"];
          // double alpha = feedingDemo.mScoopParameters["alpha"];
          // double delta = feedingDemo.mScoopParameters["delta"];
          // double beta = feedingDemo.mScoopParameters["beta"];

          double height
           = getRosParam<double>("/scoopDemo/height", nodeHandle);
          double minima
           = getRosParam<double>("/scoopDemo/minima", nodeHandle);
          double alpha
           = getRosParam<double>("/scoopDemo/alpha", nodeHandle);
          double delta 
           = getRosParam<double>("/scoopDemo/delta", nodeHandle);
          double beta
           = getRosParam<double>("/scoopDemo/beta", nodeHandle);
          double scoop_mode
           = getRosParam<int>("/scoopDemo/mode", nodeHandle);    

          height *= alpha;

          double theta = scoopHelper.getTheta2();
          double direction = scoopHelper.getDirection2();

      //--------------------------------------------------------------------------------------------------------
      // Move to the initial position above the food for scooping

          ROS_INFO_STREAM("Move to the initial position above the food for scooping");

          bool aboveFoodSuccess = action::moveAbove(
              ada,
              feedingDemo.getCollisionConstraint(),
              scoopHelper.getTransform(),
              feedingDemo.getFoodEndEffectorTransform(scoop_mode, height, minima, theta, direction, delta),
              0.01,
              0.01,
              0.1,
              0.00,
              feedingDemo.mPlanningTimeout,
              feedingDemo.mMaxNumTrials,
              feedingDemo.mVelocityLimits);

          if (!aboveFoodSuccess)
              ROS_WARN_STREAM("Move above plate failed. Please restart");
          else
              std::cout <<"Move above Place Success"<<std::endl;

      //--------------------------------------------------------------------------------------------------------
      // Excecute Scooping    

          waitForUser("Execute Scooping?", ada); 

          action::PolyScoop(
          ada,
          feedingDemo.getCollisionConstraint(),
          height,
          theta,
          minima,
          direction,
          scoop_mode,
          feedingDemo.mPlateTSRParameters["horizontalTolerance"],
          feedingDemo.mPlateTSRParameters["verticalTolerance"],
          feedingDemo.mPlateTSRParameters["rotationTolerance"],
          feedingDemo.mEndEffectorOffsetPositionTolerance,
          feedingDemo.mEndEffectorOffsetAngularTolerance,
          feedingDemo.mPlanningTimeout,
          feedingDemo.mMaxNumTrials,
          feedingDemo.mVelocityLimits);
          
          nodeHandle.setParam("/pushingDemo/allow_push_img", true);
          workspace.reset(); // what does it use for?    
      }
    }
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}

}; // namespace feeding