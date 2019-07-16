
#include <ros/ros.h>
#include <libada/util.hpp>
#include <sstream>
#include "std_msgs/String.h"

#include "feeding/FeedingDemo.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/Push.hpp"
#include "feeding/util.hpp"
#include "feeding/action/TouchTable.hpp"

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
    waitForUser("next step?", ada);

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
      ss << "up_before";
      msg.data = ss.str();
      image_pub.publish(msg);
      // talk("Move above Place Success", true);
    }
    
    // ===== DOWN TO PLATE =====
    ROS_INFO_STREAM("Down to plate");
    // ftThresholdHelper.setThresholds(1.0, 1.0); // For stopping traj when touch the table
    // bool moveDownSuccess = action::touchTable(
    //                         ada,
    //                         collisionFree,
    //                         feedingDemo.mPlanningTimeout,
    //                         feedingDemo.mPlateTSRParameters["verticalTolerance"],
    //                         feedingDemo.mPlateTSRParameters["rotationTolerance"],
    //                         FTThresholdHelper);
    // if (!moveDownSuccess)
    // {
    //   ROS_WARN_STREAM("Move down failed. Please restart");
    //   exit(EXIT_FAILURE);
    // }
    // else
    // {
    //   std::cout << "Move down Success" << std::endl;
    //   std::cout << "Taking Picture before pushing" << std::endl;
      // std_msgs::String msg;
      // ss.str(std::string());
      // ss << "down_before";
      // msg.data = ss.str();
      // image_pub.publish(msg);
    // }

    float angle;
    double pushDist;
    nodeHandle.getParam("/feedingDemo/pushingAngle", angle);
    nodeHandle.getParam("/feedingDemo/pushDist", pushDist);

    // ===== PUSH FOOD =====
    ROS_INFO_STREAM("Push");
    bool pushSuccess = action::push(
                            ada,
                            collisionFree,
                            feedingDemo.mPlanningTimeout,
                            feedingDemo.mPlateTSRParameters["verticalTolerance"],
                            feedingDemo.mPlateTSRParameters["rotationTolerance"],
                            feedingDemo.mVelocityLimits,
                            angle,
                            pushDist);

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
      ss << "down_after";
      msg.data = ss.str();
      image_pub.publish(msg);
    }

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
      ss << "up_after";
      msg.data = ss.str();
      image_pub.publish(msg);
    }

    workspace.reset();
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
}; // namespace feeding
