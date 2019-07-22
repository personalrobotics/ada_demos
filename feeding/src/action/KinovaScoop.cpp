#include "feeding/action/KinovaScoop.hpp"
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

#include "magi/solution/PlanSolution.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>

using ada::util::waitForUser;
static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

using namespace std;
using namespace Eigen;
using aikido::trajectory::concatenate;
using aikido::trajectory::Interpolated;
using aikido::trajectory::TrajectoryPtr;
using State = aikido::statespace::dart::MetaSkeletonStateSpace::State;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using magi::solution::PlanSolution;
using ada::util::createIsometry;


using magi::solution::PlanSolution;
using ada::util::createIsometry;

void waitForAnyKey() { 
    std::cout << "Press any key to continue..."; 
    char input;
    std::cin.clear();
    std::cin >> input;
} 

class Pose
{
public:
    double z, y, roll_angle;

    Pose(double z1, double y1, double roll_angle1) {
        z = z1;
        y = y1;
        roll_angle = roll_angle1;
    }
};

class PolyTraj {
    private:
        double h, d;
        double a[4];

        void solve() {
            Matrix4d A;
            Vector4d b;
            A << 1,0,0,0, 0,d,d*d,d*d*d, 0,1,2*d,3*d*d, 0,1,4*d,12*d*d;
            b << 0, h, 0, 0;
            Vector4d x = A.colPivHouseholderQr().solve(b);
            for(int i=0; i<4; i++) {
                a[i] = x[i];
            }
        }

        double func(double z, int n =4)
        {
            double d = 0;
            for (int i = 0; i < n; i++)
                d += pow(z, i) * a[i];
            return d;
        }

        double der1(double z, int n = 4)
        {
            double d = 0;
            for (int i = 1; i < n; i++)
                d += pow(z, i - 1) * i * a[i];
            return d;
        }

        double roll_angle(double z) {
            return atan(-der1(z));
        }

    public:

        PolyTraj(double h1, double d1) {
            h = h1;
            d = d1;
            solve();
        }

        void printCoefficient() {
            cout<< "h: " << h << " d: " << d << endl;
            for(int i=0; i<(sizeof(a)/sizeof(*a)); i++)
                cout << a[i] << endl;
        }

        Pose eval(double z) {
            // cout << "z:" << z << endl;
            // cout << "y:" << func(z) << endl;
            // cout << "roll_angle:" << roll_angle(z) << endl;
            return Pose(z, func(z), roll_angle(z));
        }

        // simply discretize z axis, but discretize time might be a better idea. implement that later.
        // num: num of waypoints
        std::vector<Pose> getWayPoints(int num) {
            double step = 2*d / (double) num;
            std::vector<Pose> wayPoints;
            for (int i = 1; i <= num; i++) {
                wayPoints.push_back(eval(i*step));
            }
            return wayPoints;
        }

        void test() {
            cout << "result:" << func(0) << endl;
            cout << "der1:" << der1(0) << endl;
            cout << "roll_angle:" << roll_angle(0) << endl;
        }
};

namespace feeding {
namespace action {

//==============================================================================
bool kinovaScoop(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double height,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits)
{
  // move above plate
  ROS_INFO_STREAM("Move above plate");
  bool abovePlaceSuccess = moveAbovePlate(
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

  if (!abovePlaceSuccess)
  {
    talk("Sorry, I'm having a little trouble moving. Mind if I get a little help?");
    ROS_WARN_STREAM("Move above plate failed. Please restart");
    return false;
  }
  else
  {
    std::cout <<"Move above Place Success"<<std::endl;
    talk("Move above Place Success", true);
  }

  double minima = height;

  int demotype;
  std::cout << "> ";
  std::cin >> demotype;
  if (demotype > 3 || demotype < 0)
    demotype = 0;
  // if (demotype = -1)
  //   return false;
  std::cout << "demotype = " << demotype << std::endl;
  /* demotype: 0 kinovascoop
               1 kinovascoop with twist
               2 foward kinovascoop with twist 
               3 Ryan Scoop
  */
  
  // std::cout << "plate to ee translation = " << plateEndEffectorTransform.translation() << std::endl;
  // std::cout << "plate to ee rotation = " << plateEndEffectorTransform.linear() << std::endl;

  // Scoop with twist
  if (demotype == 1) {
    height = 0.12;
    minima = 0.8*height;    
  }
  else if (demotype == 2) {
    height = 0.12;
    minima = 0.8*height;
  }
  else if (demotype == 0)
  {
    // need to fix planWithEndEffectorOffset? some constrains are not correct?
    height = 0.01;
    minima = 0.6*height;
  }
  
  // start scoop trajectory
  std::cout << "height = " << height << std::endl;

  int num = 10;
  PolyTraj scoopTraj = PolyTraj(-height, minima);
  std::vector<Pose> wayPoints = scoopTraj.getWayPoints(num);
  Pose last_pose = scoopTraj.eval(0);

  std::cout <<"begin scoop traj"<<std::endl;
  double delta_x, delta_z, delta_l, delta_roll;
  TrajectoryPtr *traj_vector = new TrajectoryPtr[wayPoints.size()];
  
  auto mArm = ada->getArm();
  auto mArmSpace = mArm->getStateSpace();
  auto metaSkeleton = mArm->getMetaSkeleton();
  auto startState = mArmSpace->createState();

  // Eigen::VectorXd config(6);
  // // is this the config of the point above the plate? change it to test.
  // config << -2.15583, 3.0954,  1.61802,  -2.45501,  -2.04492, -4.73983 ;

  // auto endEffector = ada->getHand()->getEndEffectorBodyNode();
  // auto curPose = endEffector->getWorldTransform();
  // double x = curPose.translation()[0];
  // double y = curPose.translation()[1];
  // double z = curPose.translation()[2];
  // double roll = curPose.linear()[0];
  // double pitch = curPose.linear()[1];
  // double yaw = curPose.linear()[2];

  // std::cout << "startPose translation :\n" << curPose.translation() << std::endl; 
  // std::cout << "startPose rotation :\n" << curPose.linear() << std::endl; 

  if (demotype == 0 || demotype == 1) {
    std::cout<< demotype <<"kinovascoop" << std::endl;
    Eigen::VectorXd twists(6);
    twists << 0, 0.0, M_PI/2+M_PI/24, 0.0, 0.0, 0.0;
    auto init_traj = ada->planWithEndEffectorTwist(
      twists,
      1,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);  
    ada->moveArmOnTrajectory(init_traj, collisionFree, ada::KUNZ, velocityLimits);

    Eigen::VectorXd twists2(6);
    twists2 << -M_PI/3, 0, 0.0, -0.03, 0.0, 0.06;
    init_traj = ada->planWithEndEffectorTwist(
      twists2,
      1,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);  
    ada->moveArmOnTrajectory(init_traj, collisionFree, ada::KUNZ, velocityLimits);
  }
  // else if (demotype==1) {
  //   std::cout<< "1 kinovascoop with twist" << std::endl; 
  // }
  else if (demotype==2) {
    std::cout<< "sideway scoop" << std::endl;
    Eigen::VectorXd twists0(6);
    // twists0 << 0, 0.0, M_PI, 0.1, 0.0, 0.0;
    twists0 << 0, 0.0, M_PI, 0.0, 0.0, 0.1;

    auto init_traj = ada->planWithEndEffectorTwist(
      twists0,
      1,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);  
    ada->moveArmOnTrajectory(init_traj, collisionFree, ada::KUNZ, velocityLimits);
  }
  else if (demotype==3) {
    std::cout<< "sideway scoop" << std::endl;
    Eigen::VectorXd twists0(6);
    twists0 << 0, 0.0, M_PI, 0.1, 0.0, 0.0;
    // twists0 << 0, 0.0, M_PI, 0.02, 0.0, 0.05;

    auto init_traj = ada->planWithEndEffectorTwist(
      twists0,
      1,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);  
    ada->moveArmOnTrajectory(init_traj, collisionFree, ada::KUNZ, velocityLimits);
  }
  else
    std::cout<< "no demo" << std::endl;

  waitForUser("next step?", ada);
  mArmSpace->getState(metaSkeleton.get(), startState);
  int i = 0;
  TrajectoryPtr traj;
  if (demotype == 0 || demotype == 1 || demotype == 2) {
    for (auto pose = wayPoints.begin(); pose != wayPoints.end(); ++pose) {
        delta_x = (*pose).z - last_pose.z;
        delta_z = (*pose).y - last_pose.y;
        delta_roll = (*pose).roll_angle - last_pose.roll_angle;
        delta_l = sqrt(delta_x*delta_x + delta_z*delta_z); // length

        Eigen::Vector3d direction = Eigen::Vector3d(delta_x/delta_l, 0.0, delta_z/delta_l);
        // Eigen::Isometry3d goalPose = createIsometry(x + (*pose).z, y,  z+(*pose).y,  0,  0, 0);
        // goalPose.linear() = curPose.linear();
        // if (i != 0)
        //     goalPose = createIsometry((*pose).z + 1, 0.0, (*pose).y, 0.0, 0.0, 0.0);
        if (demotype == 1) {
          Eigen::VectorXd twists(6);
          if (i==0)
            twists << 0.0, (*pose).roll_angle, 0.0, delta_x, 0.0, delta_z;
          else
            twists << 0.0, delta_roll, 0.0, delta_x, 0.0, delta_z;

          traj = ada->planWithEndEffectorTwist(
            twists,
            0.8,
            collisionFree,
            planningTimeout,
            endEffectorOffsetPositionTolerance,
            endEffectorOffsetAngularTolerance);  
        }
        else if (demotype==2) {
          Eigen::VectorXd twists(6);
          if (i==0)
            twists << 0.0, -(M_PI/2 - (*pose).roll_angle), 0.0, delta_x, 0.0, delta_z;
            // twists << 0.0, 0, 0.0, delta_x, 0.0, delta_z;
          else if (i<5)
            twists << 0.0, delta_roll, 0.0, delta_x, 0.0, delta_z;
          else
            twists << 0.0, 0, 0.0, delta_x, 0.0, 3*delta_z;

           traj = ada->planWithEndEffectorTwist(
            twists,
            0.8,
            collisionFree,
            planningTimeout,
            endEffectorOffsetPositionTolerance,
            endEffectorOffsetAngularTolerance);  
        }
        else if (demotype == 0){
          traj = ada->planArmToEndEffectorOffset(
          direction,
          delta_l,
          startState,
          collisionFree,
          planningTimeout,
          endEffectorOffsetPositionTolerance,
          endEffectorOffsetAngularTolerance);
        }
        else
          std::cout << "wrong" << std::endl;

        auto _traj = dynamic_cast<Interpolated*>(traj.get());
        _traj->evaluate(_traj->getEndTime(), startState);
        if (demotype == 1 || demotype == 2)
          std::cout << "executing traj in loop" << std::endl;
          ada->moveArmOnTrajectory(traj, collisionFree, ada::KUNZ, velocityLimits);
          std::cout << "success !!!!!!!" << std::endl;

        traj_vector[i] = traj;
        std::cout << "round " << i++ << std::endl; 
        last_pose = (*pose);
    } 

    waitForUser("do concatenated trajectory", ada);

    TrajectoryPtr trajnext, concatenatedTraj = traj_vector[0];

    // concatenate all trajs
    for (int k=1; k<wayPoints.size(); k++) {
      trajnext = traj_vector[k]; 
      concatenatedTraj = concatenate(
        *dynamic_cast<Interpolated*>(concatenatedTraj.get()), 
        *dynamic_cast<Interpolated*>(trajnext.get())
      );
    }


    std::cout << "move concatenated traj "<< std::endl;
    char current; 
    while (true) {
      cout<<"Press Enter to continue"<<endl;
      waitForAnyKey();
      ada->moveArmOnTrajectory(traj_vector[0], collisionFree, ada::KUNZ, velocityLimits);
      std::cin >> current;
      if ( (current == 'Y') || (current == 'y'))
        ada->moveArmOnTrajectory(concatenatedTraj, collisionFree, ada::KUNZ, velocityLimits);
      else
        break;
    }
  }
  else {   
    // twist1: [-0.0, -0.20, -0, 0.005, -0.00, -0.02]
    // twist2: [-0.0, -0.60, -0, 0.04, -0.00, -0.005]
    // twist3: [-0.0, -0.50, -0, 0.03, -0.00, 0.06]    

    Eigen::VectorXd twists1(6);
    twists1 << -0.0, -0.20, -0, -0.005, -0.00, -0.02;

    auto init_traj = ada->planWithEndEffectorTwist(
      twists1,
      1,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);  
    ada->moveArmOnTrajectory(init_traj, collisionFree, ada::KUNZ, velocityLimits);

    Eigen::VectorXd twists2(6);
    twists2 << -0.0, -0.60, -0, -0.04, -0.00, -0.005;

    init_traj = ada->planWithEndEffectorTwist(
      twists2,
      1,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);  
    ada->moveArmOnTrajectory(init_traj, collisionFree, ada::KUNZ, velocityLimits);

    Eigen::VectorXd twists3(6);
    twists3 << -0.0, -0.50, -0, -0.03, -0.00, 0.06;
    init_traj = ada->planWithEndEffectorTwist(
      twists3,
      1,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);  
    ada->moveArmOnTrajectory(init_traj, collisionFree, ada::KUNZ, velocityLimits);
  }
  // test twists---- it works! nice!
  int lift;
  std::cout << "lift? 1:YES, Else:NO > ";
  std::cin >> lift;
  if (lift == 1){
    Eigen::VectorXd twists(6);
    twists << 0, 0, 0.0, 0, 0.0, 0.05;
    traj = ada->planWithEndEffectorTwist(
      twists,
      0.8,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);  
    ada->moveArmOnTrajectory(traj, collisionFree, ada::KUNZ, velocityLimits);
  }
  return true;
}

} // namespace action
} // namespace feeding
