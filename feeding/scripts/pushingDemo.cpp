
#include <ros/ros.h>
#include <libada/util.hpp>
#include <sstream>
#include "std_msgs/String.h"

#include "feeding/FeedingDemo.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/Push.hpp"
#include "feeding/util.hpp"
#include "feeding/action/TouchTable.hpp"
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/action/LiftUp.hpp"

using ada::util::waitForUser;

namespace feeding {

void pushingDemo(FeedingDemo& feedingDemo, ros::NodeHandle nodeHandle)
{
  // TODO: positioning the hand above the plate
  ROS_INFO_STREAM("========== Pushing DEMO ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();
  
  std::shared_ptr<FTThresholdHelper> FTThresholdHelper = feedingDemo.getFTThresholdHelper();

  ros::Publisher image_pub = nodeHandle.advertise<std_msgs::String>("/save_image", 1, true);
  std::stringstream ss;

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
      // talk("Sorry, I'm having a little trouble moving. Mind if I get a little
      // help?");
      ROS_WARN_STREAM("Move above plate failed. Please restart");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "Move above Place Success" << std::endl;
      std::cout << "Taking Picture from above" << std::endl;
      std_msgs::String msg;
      ss.str(std::string());
      ss << "1_start";
      msg.data = ss.str();
      image_pub.publish(msg);
      // talk("Move above Place Success", true);
    }
    
    // ===== DOWN TO PLATE =====
    waitForUser("Going Down", ada);

    ROS_INFO_STREAM("Down to plate");
    // std::shared_ptr<FTThresholdHelper> FTThresholdHelper = nullptr;
    float angle;
    nodeHandle.getParam("/feedingDemo/pushingAngle", angle);
    bool moveDownSuccess = action::touchTable(
                            ada,
                            collisionFree,
                            feedingDemo.mPlanningTimeout,
                            feedingDemo.mPlateTSRParameters["verticalTolerance"],
                            feedingDemo.mPlateTSRParameters["rotationTolerance"],
                            FTThresholdHelper,
                            feedingDemo.mVelocityLimits,
                            angle);
    if (!moveDownSuccess)
    {
      ROS_WARN_STREAM("Move down failed. Please restart");
      exit(EXIT_FAILURE);
    }
    else
    {
      bool liftUpSuccess = action::liftUp(
                              ada,
                              collisionFree,
                              feedingDemo.mPlanningTimeout,
                              feedingDemo.mPlateTSRParameters["verticalTolerance"],
                              feedingDemo.mPlateTSRParameters["rotationTolerance"],
                              FTThresholdHelper,
                              0.008);

      std::cout << "Move down Success" << std::endl;
      std::cout << "Taking Picture before pushing" << std::endl;
      std_msgs::String msg;
      ss.str(std::string());
      ss << "2_before_pushing";
      msg.data = ss.str();
      image_pub.publish(msg);
    }

    // ===== PUSH FOOD =====
    waitForUser("Push?", ada);

    
    double pushDist;
    nodeHandle.getParam("/feedingDemo/pushDist", pushDist);

    
    ROS_INFO_STREAM("Push");
    bool pushSuccess = action::push(
                            ada,
                            collisionFree,
                            feedingDemo.mPlanningTimeout,
                            feedingDemo.mPlateTSRParameters["verticalTolerance"],
                            feedingDemo.mPlateTSRParameters["rotationTolerance"],
                            FTThresholdHelper,
                            feedingDemo.mVelocityLimits,
                            angle);

    if (!pushSuccess)
    {
      ROS_WARN_STREAM("Push failed. Please restart");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "Push Success" << std::endl;
      std::cout << "Taking Picture after pushing" << std::endl;
      std_msgs::String msg;
      ss.str(std::string());
      ss << "3_after_pushing";
      msg.data = ss.str();
      image_pub.publish(msg);
    }

    bool liftUpSuccess = action::liftUp(
                            ada,
                            collisionFree,
                            feedingDemo.mPlanningTimeout,
                            feedingDemo.mPlateTSRParameters["verticalTolerance"],
                            feedingDemo.mPlateTSRParameters["rotationTolerance"],
                            FTThresholdHelper,
                            0.03);

    std::cout << "Taking Picture above wall" << std::endl;
    std_msgs::String mg;
    ss.str(std::string());
    ss << "4_above_wall";
    mg.data = ss.str();
    image_pub.publish(mg);

    // ===== MOVE ABOVE PLATE =====
    ROS_INFO_STREAM("Move above plate, again");
    bool backAbovePlaceSuccess = action::moveAbovePlate(
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

    if (!backAbovePlaceSuccess)
    {
      ROS_WARN_STREAM("Move above plate failed. Please restart");
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "Move back above Success" << std::endl;
      std::cout << "Taking Picture after finishing" << std::endl;
      std_msgs::String msg;
      ss.str(std::string());
      ss << "5_finish";
      msg.data = ss.str();
      image_pub.publish(msg);
    }

    workspace.reset();
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
}; // namespace feeding
