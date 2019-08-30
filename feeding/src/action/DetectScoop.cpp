#include "feeding/action/DetectScoop.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"
#include "feeding/AcquisitionAction.hpp"

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include "aikido/trajectory/util.hpp"
#include "aikido/robot/util.hpp"
#include <aikido/trajectory/Interpolated.hpp>
#include <libada/util.hpp>

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>
#include "feeding/action/ScoopHelper.hpp"

using ada::util::waitForUser;
using ada::util::waitForAnyKey;
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
bool DetectScoop(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double height,
    double theta,
    double minima, 
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

    std::cout << "demotype = " << demotype << std::endl;
    std::cout << "height = " << height << std::endl;

// ----------------------------------------------------------------------------------------------------
// Variable Preparation

    int num = 10;
    PolyTraj scoopTraj = PolyTraj(-height, minima);
    std::vector<Pose> wayPoints = scoopTraj.getWayPoints(num);
    Pose last_pose = scoopTraj.eval(0);

    // std::cout <<"begin scoop traj"<<std::endl;
    double delta_x, delta_z, delta_l, delta_roll;
    TrajectoryPtr *traj_vector = new TrajectoryPtr[wayPoints.size()];

    auto mArm = ada->getArm();
    auto mArmSpace = mArm->getStateSpace();
    auto metaSkeleton = mArm->getMetaSkeleton();
    auto startState = mArmSpace->createState();

// ----------------------------------------------------------------------------------------------------
// Plan Scoop Traj

    mArmSpace->getState(metaSkeleton.get(), startState);
    int i = 0;
    TrajectoryPtr traj;
    bool successConTraj;
    // ----------------------------------------------------------------------------------------------------
    // Kinova Traj and its variations

    if (demotype == 0 || demotype == 1 || demotype == 2)
    {
        for (auto pose = wayPoints.begin(); pose != wayPoints.end(); ++pose)
        {
            delta_x = (*pose).z - last_pose.z;
            delta_z = (*pose).y - last_pose.y;
            delta_roll = (*pose).roll_angle - last_pose.roll_angle;
            delta_l = sqrt(delta_x*delta_x + delta_z*delta_z); // length

            Eigen::Vector3d direction = Eigen::Vector3d(delta_x/delta_l, 0.0, delta_z/delta_l);
            if (demotype == 0)
            {
                // 0 kinovascoop

                Eigen::VectorXd twists(6);
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
            else if (demotype == 1)
            {
                // 1 kinovascoop with twist

                Eigen::VectorXd twists(6);
                if (i==0)
                    twists << - (*pose).roll_angle * sin(theta), (*pose).roll_angle * cos(theta) , 0.0, delta_x * cos(theta), delta_x * sin(theta), delta_z;
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
            else if (demotype==2)
            {
                // 2 foward kinovascoop with twist

                Eigen::VectorXd twists(6);
                if (i==0)
                    twists << 0.0, -(M_PI/2 - (*pose).roll_angle), 0.0, delta_x, 0.0, delta_z;
                else if (i<3)
                    twists << 0.0, delta_roll, 0.0, delta_x, 0.0, delta_z;
                else if (i>5)
                    twists << 0.0, 0, 0.0, delta_x, 0.0, 3*delta_z;
                else
                    twists << 0.0, 0, 0.0, delta_x, 0.0, delta_z;
                // if (i==0)
                //     twists << 0.0, -M_PI/3, 0.0, delta_x, 0.0, delta_z;
                // else if (i>5)
                //     twists << 0.0, 0, 0.0, delta_x, 0.0, 3*delta_z;
                // else
                //     twists << 0.0, 0, 0.0, delta_x, 0.0, delta_z;

                traj = ada->planWithEndEffectorTwist(
                    twists,
                    1,
                    startState,
                    collisionFree,
                    planningTimeout,
                    endEffectorOffsetPositionTolerance,
                    endEffectorOffsetAngularTolerance);
            }
            else
                std::cout << "wrong demotype" << std::endl;

            traj_vector[i] = traj;
            std::cout << "round " << i++ << std::endl;
            last_pose = (*pose);

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

    }

    // ----------------------------------------------------------------------------------------------------
    // // Ryan's Scoop.

    else
    {
        // twist1: [-0.0, -0.20, -0, 0.005, -0.00, -0.02]
        // twist2: [-0.0, -0.60, -0, 0.04, -0.00, -0.005]
        // twist3: [-0.0, -0.50, -0, 0.03, -0.00, 0.06]

        Eigen::VectorXd twists1(6);
        twists1 << -0.0, -0.20, -0, -0.005, -0.00, -0.02;
        ada->moveArmWithEndEffectorTwist(
            twists1,
            1,
            collisionFree,
            planningTimeout,
            endEffectorOffsetPositionTolerance,
            endEffectorOffsetAngularTolerance,
            velocityLimits);

        Eigen::VectorXd twists2(6);
        twists2 << -0.0, -0.60, -0, -0.04, -0.00, -0.005;
        ada->moveArmWithEndEffectorTwist(
            twists2,
            1,
            collisionFree,
            planningTimeout,
            endEffectorOffsetPositionTolerance,
            endEffectorOffsetAngularTolerance,
            velocityLimits);

        Eigen::VectorXd twists3(6);
        twists3 << -0.0, -0.50, -0, -0.03, -0.00, 0.06;
        ada->moveArmWithEndEffectorTwist(
            twists3,
            1,
            collisionFree,
            planningTimeout,
            endEffectorOffsetPositionTolerance,
            endEffectorOffsetAngularTolerance,
            velocityLimits);
    }

// -------------------------------------------------------------------------------------------------------
// Lift the arm after every excecution

    char lift;

    if (!successConTraj)
        lift = 'Y';
    else
    {        
        std::cout<<"lift or not? Y/y"<< std::endl;
        std::cin >> lift;
    }

    if ( (lift == 'Y') || (lift == 'y'))
    {
        Eigen::VectorXd twists(6);
        twists << 0, 0, 0.0, 0, 0.0, 0.15;
        ada->moveArmWithEndEffectorTwist(
            twists,
            1,
            collisionFree,
            planningTimeout,
            endEffectorOffsetPositionTolerance,
            endEffectorOffsetAngularTolerance,
            velocityLimits);
    }

    return true;
}

} // namespace action
} // namespace feeding
