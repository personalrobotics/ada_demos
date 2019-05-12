#include "feeding/action/KinovaScoop.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include "feeding/action/Grab.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/util.hpp"
#include "feeding/AcquisitionAction.hpp"

#include <libada/util.hpp>

static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;


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
            // cout << "Here is the matrix A:\n" << A << endl;
            // cout << "Here is the vector b:\n" << b << endl;
            Vector4d x = A.colPivHouseholderQr().solve(b);
            // cout << "The solution is:\n" << x << endl;
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


  // start scoop trajectory
  std::cout << "height = " << height << std::endl;
  height = 0.01;
  double minima = 2*height;
  int num = 10;
  PolyTraj scoopTraj = PolyTraj(-height, minima);
  std::vector<Pose> wayPoints = scoopTraj.getWayPoints(num);
  Pose last_pose = scoopTraj.eval(0);
  std::cout <<"begin scoop traj"<<std::endl;
  std::cout <<"endEffectorOffsetPositionTolerance = "<<endEffectorOffsetPositionTolerance<<std::endl;
  double delta_x, delta_z, delta_l, delta_roll;

  for (auto pose = wayPoints.begin(); pose != wayPoints.end(); ++pose) {

      delta_x = (*pose).z - last_pose.z;
      delta_z = (*pose).y - last_pose.y;
      delta_roll = (*pose).roll_angle - last_pose.roll_angle;
      delta_l = sqrt(delta_x*delta_x + delta_z*delta_z);
      delta_x = delta_x/delta_l;
      delta_z = delta_z/delta_l;
      last_pose = (*pose);

      std::cout << "waypoint = [" << (*pose).z << ", " << (*pose).y <<", "<< (*pose).roll_angle << "]" << std::endl;
      std::cout << "direction = [" << delta_x << ", " << 0 <<", "<< delta_z << "]" << std::endl;
      std::cout << "length = " << delta_l;

      bool testSuccess = ada->moveArmToEndEffectorOffset(
      Eigen::Vector3d(delta_x, 0, delta_z), // what's the reference frame ?
      delta_l,
      collisionFree,
      planningTimeout,
      endEffectorOffsetPositionTolerance,
      endEffectorOffsetAngularTolerance);

      if (!testSuccess) {
          std::cout <<"Test Fail!!!!!!!!!"<<std::endl;
          return false;
      }
  }

  // bool testSuccess = ada->moveArmToEndEffectorOffset(
  //     Eigen::Vector3d(0, 0, 1), // what's the reference frame ?
  //     0.0001,
  //     collisionFree,
  //     planningTimeout,
  //     endEffectorOffsetPositionTolerance,
  //     endEffectorOffsetAngularTolerance);

  //   testSuccess = ada->moveArmToEndEffectorOffset(
  //     Eigen::Vector3d(0, 0, -1), // what's the reference frame ?
  //     0.005,
  //     collisionFree,
  //     planningTimeout,
  //     endEffectorOffsetPositionTolerance,
  //     endEffectorOffsetAngularTolerance);

  // if (!testSuccess)
  // {
  //   std::cout <<"Test Fail!!!!!!!!!"<<std::endl;
  //   talk("Test Fail!!!!!!!!!", true);
  //   return false;
  // }
  // else
  // {
  //   talk("Move above Place Success");
  // }

  return true;
}

} // namespace action
} // namespace feeding
