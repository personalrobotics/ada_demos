
#include <ros/ros.h>
#include <libada/util.hpp>
#include "std_msgs/String.h"
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/DetectScoop.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveAbove.hpp"


using ada::util::waitForUser;
using ada::util::getRosParam;
using ada::util::createIsometry;

namespace feeding {

bool IsValid(Eigen::Isometry3d T)
{
    double tx = T.translation()[0];
    double ty = T.translation()[1];
    double tz = T.translation()[2];
    std::cout << "tx = " << tx << ", ty = " << ty << ", tz = " << tz << std::endl;
    if (tx > 0.35 || tx < 0.18)
        return false;
    if (ty > -0.20 || ty < -0.38)
        return false;
    if (tz > 0.3 || tz < 0.22)
        return false;

    std::cout << "Find Valid Pose" << std::endl;
    return true;
} 

void DetectScoopDemo(
    FeedingDemo& feedingDemo,
    ros::NodeHandle nodeHandle)
{

//--------------------------------------------------------------------------------------------------------
// Food Detector preparation.

    MarkersListener ml = MarkersListener(nodeHandle);
    std::string foodDetectorTopicName = getRosParam<std::string>(
        "/perception/foodDetectorTopicName", nodeHandle);
    std::cout << "food marker topic " << foodDetectorTopicName << std::endl;
    double PerceptionTimeout
        = getRosParam<double>("/perception/timeoutSeconds", nodeHandle);
    ros::Subscriber sub = nodeHandle.subscribe(foodDetectorTopicName, 100, &MarkersListener::callback, &ml);
    visualization_msgs::MarkerArrayConstPtr markerMessage = 
        ros::topic::waitForMessage<visualization_msgs::MarkerArray>(foodDetectorTopicName, nodeHandle, ros::Duration(PerceptionTimeout));

    auto ada = feedingDemo.getAda();
    auto workspace = feedingDemo.getWorkspace();
    auto collisionFree = feedingDemo.getCollisionConstraint();
    auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

    while (true) 
    {
        waitForUser("Keep On Demo?", ada); 

    //--------------------------------------------------------------------------------------------------------
    // move above plate

        ROS_INFO_STREAM("Move above plate");
        bool abovePlateSuccess = action::moveAbovePlate(
        ada,
        collisionFree,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters["horizontalTolerance"],
        feedingDemo.mPlateTSRParameters["verticalTolerance"],
        0.1, 
        // feedingDemo.mPlateTSRParameters["rotationTolerance"], // M_PI
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits);

        if (!abovePlateSuccess)
            ROS_WARN_STREAM("Move above plate failed. Please restart");
        else
            std::cout <<"Move above Place Success"<<std::endl;

    //--------------------------------------------------------------------------------------------------------
    // Decide to excecute which scooping mode

        int demotype = 1;
        std::cout << "input scooping type \n \
                      0: kinovascoop \n \
                      1: kinovascoop with twist \n \
                      2: foward kinovascoop with twist \n \
                      else will triger 1"<< std::endl;
        std::cout << "> ";
        std::cin >> demotype;

        if (demotype > 2 || demotype < 0)
            demotype = 1;
        /* demotype: 0 kinovascoop
                     1 kinovascoop with twist
                     2 foward kinovascoop with twist
                     3 Ryan Scoop
        */

        //--------------------------------------------------------------------------------------------------------
        // Set Scooping Traj Parameters

        double beta = 1.0;
        std::cout << "input height coefficient < default beta = 1.0 >"<< std::endl;
        std::cout << "> ";
        std::cin >> beta;
        // std::cout << "height coefficient = "<< beta << std::endl;
        double height = feedingDemo.mPlateTSRParameters.at("height") * beta;
        double minima = 0.8 * height;

        double delta = 0.0;
        if (demotype == 1 || demotype == 2) 
        {
            std::cout << "input delta adjustment < default delta = 0.0 >" << std::endl;
            std::cout << "> ";
            std::cin >> delta;
        }

        // determine from which angle to scoop
        double theta;
        std::cout << "input theta coefficient < default theta coefficient = 0.0 >"<< std::endl;
        std::cout << "> ";
        std::cin >> theta;
        theta *= M_PI;
        std::cout << "input theta = " << theta << std::endl;

    //--------------------------------------------------------------------------------------------------------
    // Move to the initial position above the food for scooping
        Eigen::Isometry3d target = ml.getTransform();
        // while (!IsValid(target))
        while (!ml.isValid())
        {
            target = ml.getTransform();
        }

        // need to fix this bug. why isn't it right in the marker listener.
        Eigen::Matrix3d m;
        m << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        target.linear() = m;

        // std::cout << "food transform debug \n " << target.linear() << "\n" << target.translation() << std::endl;

        theta = ml.getTheta2();
        std::cout << "final theta = " << theta << std::endl;

        double direction = ml.getDirection2();

    //--------------------------------------------------------------------------------------------------------
    // Move to the initial position above the food for scooping

        waitForUser("next step?", ada);

        Eigen::Isometry3d eeTransform;
        eeTransform = feedingDemo.getFoodEndEffectorTransform(demotype, height, minima, theta, direction, delta);

        ROS_INFO_STREAM("Move to the initial position above the food for scooping");

        bool aboveFoodSuccess = action::moveAbove(
            ada,
            collisionFree,
            target,
            eeTransform,
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

        waitForUser("next step?", ada); 

        action::DetectScoop(
        ada,
        collisionFree,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        height,
        theta,
        minima,
        direction,
        demotype,
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
}; // namespace feeding
