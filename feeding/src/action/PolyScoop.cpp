#include "feeding/action/PolyScoop.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"

#include "feeding/util.hpp"

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include "aikido/trajectory/util.hpp"
#include "aikido/robot/util.hpp"
#include <aikido/trajectory/Interpolated.hpp>
#include <libada/util.hpp>

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

using ada::util::waitForUser;
// using ada::util::waitForAnyKey;
static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

using namespace std;
using namespace Eigen;
using aikido::trajectory::concatenate;
using aikido::trajectory::Interpolated;
using aikido::trajectory::TrajectoryPtr;
using State = aikido::statespace::dart::MetaSkeletonStateSpace::State;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using ada::util::createIsometry;

namespace feeding {
namespace action {

//==============================================================================
bool PolyScoop(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    double height,
    double theta,
    double minima,
    double direction, 
    int demotype,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits)
{
    auto mArm = ada->getArm();
    auto mArmSpace = mArm->getStateSpace();
    auto metaSkeleton = mArm->getMetaSkeleton();
    auto startState = mArmSpace->createState();
    mArmSpace->getState(metaSkeleton.get(), startState);

    std::cout << "demotype = " << demotype << std::endl;
    std::cout << "height = " << height << std::endl;

// ----------------------------------------------------------------------------------------------------
// Variable Preparation

    int num = 10;
    PolyTraj scoopTraj = PolyTraj(-height, minima);
    std::vector<Pose> wayPoints = scoopTraj.getWayPoints(num);
    Pose last_pose = scoopTraj.eval(0);

    double delta_x, delta_z, delta_l, delta_roll;
    TrajectoryPtr *traj_vector = new TrajectoryPtr[wayPoints.size()];

// ----------------------------------------------------------------------------------------------------
// Plan Scoop Traj

    int i = 0;
    TrajectoryPtr traj;
    bool successConTraj;
    // ----------------------------------------------------------------------------------------------------
    // Kinova Traj and its variations


    for (auto pose: wayPoints)
    {
        delta_x = (pose.z - last_pose.z) * direction;
        delta_z = (pose.y - last_pose.y);
        delta_roll = (pose.roll_angle - last_pose.roll_angle) * direction;
        delta_l = sqrt(delta_x*delta_x + delta_z*delta_z); // length

        Eigen::Vector3d unitDir = Eigen::Vector3d(delta_x/delta_l, 0.0, delta_z/delta_l);
        Eigen::VectorXd twists(6);

        if (demotype == 0)
        {
            // 0 kinovascoop

            if (i<5)
                twists << 0.0, 0.0, 0.0, delta_x * cos(theta), delta_x * sin(theta), delta_z;
            else
                twists << 0.0, 0.0, 0.0, delta_x * cos(theta), delta_x * sin(theta), 3 * delta_z;

            traj = ada->planWithEndEffectorTwist(
                twists,
                1,
                startState,
                collisionFree,
                planningTimeout,
                endEffectorOffsetPositionTolerance,
                endEffectorOffsetAngularTolerance);
        }
        if (demotype == 1)
        {
            // 1 kinovascoop with twist

            if (i==0)
                twists << - pose.roll_angle * sin(theta) * direction, pose.roll_angle * cos(theta) * direction, 0.0, delta_x * cos(theta), delta_x * sin(theta), delta_z;
            else if (i<5)
                twists << - delta_roll * sin(theta), delta_roll * cos(theta), 0.0, delta_x * cos(theta), delta_x * sin(theta), delta_z;
            else
                twists << - delta_roll * sin(theta), delta_roll * cos(theta), 0.0, delta_x * cos(theta), delta_x * sin(theta), 3 * delta_z;

            traj = ada->planWithEndEffectorTwist(
                twists,
                1,
                startState,
                collisionFree,
                planningTimeout,
                endEffectorOffsetPositionTolerance,
                endEffectorOffsetAngularTolerance);
        }
        if (demotype==2)
        {
            // 2 foward kinovascoop with twist

            if (i==0)
                twists << 0.0, -(M_PI/2 - pose.roll_angle), 0.0, delta_x, 0.0, delta_z;
            else if (i<3)
                twists << 0.0, delta_roll, 0.0, delta_x, 0.0, delta_z;
            else if (i>5)
                twists << 0.0, 0, 0.0, delta_x, 0.0, 3*delta_z;
            else
                twists << 0.0, 0, 0.0, delta_x, 0.0, delta_z;

            traj = ada->planWithEndEffectorTwist(
                twists,
                1,
                startState,
                collisionFree,
                planningTimeout,
                endEffectorOffsetPositionTolerance,
                endEffectorOffsetAngularTolerance);
        }

        traj_vector[i] = traj;
        std::cout << "round " << i++ << std::endl;
        last_pose = pose;

        // set the virtual state to next position.
        auto _traj = dynamic_cast<Interpolated*>(traj.get());
        _traj->evaluate(_traj->getEndTime(), startState);
    }

    //------------------------------------------------------------------------------------
    // Concatenate planned traj and excecute it.

    TrajectoryPtr trajnext, concatenatedTraj = traj_vector[0];

    // concatenate all trajs
    for (int k=1; k<wayPoints.size(); k++)
    {
        trajnext = traj_vector[k];
        concatenatedTraj = concatenate(
          *dynamic_cast<Interpolated*>(concatenatedTraj.get()),
          *dynamic_cast<Interpolated*>(trajnext.get()));
    }

    successConTraj = ada->moveArmOnTrajectory(concatenatedTraj, collisionFree, ada::KUNZ, velocityLimits);



// -------------------------------------------------------------------------------------------------------
// Lift the arm after every excecution

    // char lift;

    // if (!successConTraj)
    //     lift = 'Y';
    // else
    // {        
    //     std::cout<<"lift or not? Y/y"<< std::endl;
    //     std::cin >> lift;
    // }

    // if ( (lift == 'Y') || (lift == 'y'))
    // {
    //     Eigen::VectorXd twists(6);
    //     twists << 0, 0, 0.0, 0, 0.0, 0.075;
    //     ada->moveArmWithEndEffectorTwist(
    //         twists,
    //         1,
    //         collisionFree,
    //         planningTimeout,
    //         endEffectorOffsetPositionTolerance,
    //         endEffectorOffsetAngularTolerance,
    //         velocityLimits);
    // }

    return true;
}

} // namespace action
} // namespace feeding
