#include <aikido/constraint/dart/TSR.hpp>
#include <libada/util.hpp>
#include "feeding/action/MoveAbove.hpp"

#include "feeding/util.hpp"

#include "feeding/action/PolyScoop.hpp"
#include "feeding/FeedingDemo.hpp"

using aikido::constraint::dart::TSR;
using ada::util::waitForUser;

// Contains motions which are mainly TSR actions
namespace feeding {

bool moveAboveAndScoop(
    const std::shared_ptr<ada::Ada>& ada,
    // feeding::FoodItem& item,
    const Eigen::Isometry3d& foodTransform,
    FeedingDemo& feedingDemo,
    ros::NodeHandle nodeHandle)
{

//--------------------------------------------------------------------------------------------------------
// Decide to excecute which scooping mode

    int scoop_mode = 1;
    std::cout << "input scooping type \n \
                  0: kinovascoop \n \
                  1: kinovascoop with twist \n \
                  2: foward kinovascoop with twist \n \
                  else will triger 1"<< std::endl;
    std::cout << "> ";
    std::cin >> scoop_mode;

    if (scoop_mode > 2 || scoop_mode < 0)
        scoop_mode = 1;
    /* scoop_mode: 0 kinovascoop
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
    std::cout << "height coefficient = "<< beta << std::endl;

    double height = 0.1;
    // double height = feedingDemo.mPlateTSRParameters.at("height");
    std::cout << "height = "<< feedingDemo.mPlateTSRParameters["horizontalTolerance"] << std::endl;

    // double height = 0.15 * beta;
    double minima = 0.8 * height;

    double delta = 0.0;
    if (scoop_mode == 1 || scoop_mode == 2) 
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

//     ScoopHelper scoopHelper = ScoopHelper(nodeHandle, foodTransform);
//     // Q: is the pose there same as the pose that we received directly from MarkerArray.
//     std::cout << "fffffffffff = " << std::endl;

// //--------------------------------------------------------------------------------------------------------
// // Move to the initial position above the food for scooping
//     // Eigen::Isometry3d foodTarget = item->getPose();
//     // // need to fix this bug. why isn't it right in the marker listener.
//     // Eigen::Matrix3d m;
//     // m << 1, 0, 0,
//     //      0, 1, 0,
//     //      0, 0, 1;
//     // foodTarget.linear() = m;

//     // std::cout << "food transform debug \n " << foodTarget.linear() << "\n" << foodTarget.translation() << std::endl;

//     theta = scoopHelper.getTheta2();
//     std::cout << "final theta = " << theta << std::endl;

//     double direction = scoopHelper.getDirection2();

// //--------------------------------------------------------------------------------------------------------
// // Move to the initial position above the food for scooping

//     waitForUser("next step?", ada);

//     Eigen::Isometry3d eeTransform;
//     eeTransform = feedingDemo.getFoodEndEffectorTransform(scoop_mode, height, minima, theta, direction, delta);

//     ROS_INFO_STREAM("Move to the initial position above the food for scooping");

//     bool aboveFoodSuccess = moveAbove(
//         ada,
//         feedingDemo.getCollisionConstraint(),
//         // foodTarget,
//         scoopHelper.getTransform(),
//         eeTransform,
//         0.01,
//         0.01,
//         0.1,
//         0.00,
//         feedingDemo.mPlanningTimeout,
//         feedingDemo.mMaxNumTrials,
//         feedingDemo.mVelocityLimits);

//     if (!aboveFoodSuccess)
//         ROS_WARN_STREAM("Move above plate failed. Please restart");
//     else
//         std::cout <<"Move above Place Success"<<std::endl;

// //--------------------------------------------------------------------------------------------------------
// // Excecute Scooping    

//     waitForUser("next step?", ada); 

//     PolyScoop(
//     ada,
//     feedingDemo.getCollisionConstraint(),
//     height,
//     theta,
//     minima,
//     direction,
//     scoop_mode,
//     feedingDemo.mPlateTSRParameters["horizontalTolerance"],
//     feedingDemo.mPlateTSRParameters["verticalTolerance"],
//     feedingDemo.mPlateTSRParameters["rotationTolerance"],
//     feedingDemo.mEndEffectorOffsetPositionTolerance,
//     feedingDemo.mEndEffectorOffsetAngularTolerance,
//     feedingDemo.mPlanningTimeout,
//     feedingDemo.mMaxNumTrials,
//     feedingDemo.mVelocityLimits);

    // workspace.reset();

  }

} // namespace feeding
