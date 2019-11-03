
#include <ros/ros.h>
#include <libada/util.hpp>
#include "std_msgs/String.h"
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/KinovaScoop.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveAbove.hpp"

using ada::util::waitForUser;
using ada::util::getRosParam;

namespace feeding {

// class MarkersListener
// {
// private:    
//     // ros::Subscriber sub;

//     visualization_msgs::Marker marker;
//     Eigen::Isometry3d FoodTransform; // transformation from camera frame to food item frame
//     int seq;

// public:

//     //MarkersListener(ros::NodeHandle n) 
//     MarkersListener() 
//     {
//         seq = 0;
//         std::cout << "Contruct a MarkerArray Listener" << std::endl;
//        // sub = n.subscribe("/food_detector/marker_array", 100, callback);
//     }

//     void callback(visualization_msgs::MarkerArray marker_array) 
//     {
//         // std::cout << "call back" << std::endl;
//         // if (sizeof(marker_array) == 0)
//         // return;
//         if (marker_array.markers.empty())
//         {   
//             // std::cout << "empty" << std::endl;
//             return;
//         }
//         marker = marker_array.markers[0]; 
//         // std::cout <<"Marker is in "<< marker.header.frame_id.c_str() << std::endl;
//         double tx, ty, tz, rx, ry, rz, rw;
//         tx = marker.pose.position.x; 
//         ty = marker.pose.position.y;
//         tz = marker.pose.position.z; 
//         rx = marker.pose.orientation.x;
//         ry = marker.pose.orientation.y;
//         rz = marker.pose.orientation.z;
//         rw = marker.pose.orientation.w; 
//         Eigen::Quaterniond q(rw, rx, ry, rz);
//         FoodTransform.translation() = Eigen::Vector3d(tx, ty, tz); 
//         FoodTransform.linear() = q.matrix();
//         // if (seq == 0) 
//         // {
//         //     PrintInfo("food", FoodTransform)
//         // }
//         seq++;
//     }

//     Eigen::Isometry3d getTranform()
//     {
//         return FoodTransform;
//     }

//     visualization_msgs::Marker getMarker()
//     {
//         return marker;
//     }

// };

void kinovaScoopDemo(
    FeedingDemo& feedingDemo,
    ros::NodeHandle nodeHandle)
{
    ROS_INFO_STREAM("========== Kinova Scoop DEMO ==========");
    // MarkersListener ml = MarkersListener();
    // std::string foodDetectorTopicName = getRosParam<std::string>(
    //     "/perception/foodDetectorTopicName", nodeHandle);
    // double PerceptionTimeout
    //     = getRosParam<double>("/perception/timeoutSeconds", nodeHandle);
    // ros::Subscriber sub = nodeHandle.subscribe(foodDetectorTopicName, 100, &MarkersListener::callback, &ml);
    // // visualization_msgs::MarkerArrayConstPtr markerMessage = 
    //     ros::topic::waitForMessage<visualization_msgs::MarkerArray>(foodDetectorTopicName, nodeHandle, ros::Duration(PerceptionTimeout));
    // Eigen::Isometry3d target = ml.getTranform();
    // PrintInfo("target", target);

    auto ada = feedingDemo.getAda();
    auto workspace = feedingDemo.getWorkspace();
    auto collisionFree = feedingDemo.getCollisionConstraint();
    auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();
    PrintInfo("plate", plate);

//--------------------------------------------------------------------------------------------------------
    // move above plate
    ROS_INFO_STREAM("Move above plate");
    bool abovePlaceSuccess = action::moveAbovePlate(
    ada,
    collisionFree,
    plate,
    feedingDemo.getPlateEndEffectorTransform(),
    feedingDemo.mPlateTSRParameters["horizontalTolerance"],
    feedingDemo.mPlateTSRParameters["verticalTolerance"],
    0.1, 
    // feedingDemo.mPlateTSRParameters["rotationTolerance"],
    feedingDemo.mPlanningTimeout,
    feedingDemo.mMaxNumTrials,
    feedingDemo.mVelocityLimits);

    if (!abovePlaceSuccess)
    {
        talk("Sorry, I'm having a little trouble moving. Mind if I get a little help?");
        ROS_WARN_STREAM("Move above plate failed. Please restart");
    }
    else
    {
        std::cout <<"Move above Place Success"<<std::endl;
        talk("Move above Place Success", true);
    }


//--------------------------------------------------------------------------------------------------------
    // Eigen::Isometry3d eeTransform;
    // eeTransform = feedingDemo.getFoodEndEffectorTransform();
    // eeTransform.linear() *= Eigen::Matrix3d(
    //           Eigen::AngleAxisd(-M_PI/3, Eigen::Vector3d::UnitX()));
    // eeTransform.translation()[0] = -0.1;

    // waitForUser("next step?", ada); 
    // int num_try = 2;
    // while (num_try > 0)
    // {
    //     num_try--;

    //     // move above plate
    //     ROS_INFO_STREAM("Move above plate");

    //     bool abovePlaceSuccess = action::moveAbove(
    //         ada,
    //         collisionFree,
    //         target,
    //         feedingDemo.getPlateEndEffectorTransform(),
    //         0.01,
    //         0.01,
    //         0.1,
    //         0.00,
    //         feedingDemo.mPlanningTimeout,
    //         feedingDemo.mMaxNumTrials,
    //         feedingDemo.mVelocityLimits);

    //     if (!abovePlaceSuccess)
    //     {
    //         talk("Sorry, I'm having a little trouble moving. Mind if I get a little help?");
    //         ROS_WARN_STREAM("Move above plate failed. Please restart");
    //         continue;
    //     }
    //     else
    //     {
    //         std::cout <<"Move above Place Success"<<std::endl;
    //         talk("Move above Place Success", true);
    //         break;
    //     }
    // }

    waitForUser("next step?", ada); 

    while (true)
    {
        waitForUser("next step?", ada);

        // std::this_thread::sleep_for(std::chrono::milliseconds(200));

        ROS_INFO_STREAM("Running Kinova Scoop demo");

        action::kinovaScoop(
        ada,
        collisionFree,
        plate,
        feedingDemo.getFoodEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters.at("height"),
        feedingDemo.mPlateTSRParameters["horizontalTolerance"],
        feedingDemo.mPlateTSRParameters["verticalTolerance"],
        feedingDemo.mPlateTSRParameters["rotationTolerance"],
        feedingDemo.mEndEffectorOffsetPositionTolerance,
        feedingDemo.mEndEffectorOffsetAngularTolerance,
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits);

        workspace.reset();
    }

    // ===== DONE =====
    ROS_INFO("Demo finished.");
}
};
