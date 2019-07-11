#include "feeding/action/Pushing.hpp"
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

using ada::util::waitForUser;
static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

using namespace std;
using namespace Eigen;
using aikido::trajectory::concatenate;
using aikido::trajectory::Interpolated;
using aikido::trajectory::TrajectoryPtr;
using State = aikido::statespace::dart::MetaSkeletonStateSpace::State;
using aikido::statespace::dart::MetaSkeletonStateSpace;
// using magi::solution::PlanSolution;
using ada::util::createIsometry;

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
bool pushing(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
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
}

} // namespace action
} // namespace feeding
