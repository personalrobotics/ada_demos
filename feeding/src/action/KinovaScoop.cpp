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
  double delta_x, delta_z, delta_l, delta_roll;
  TrajectoryPtr *traj_vector = new TrajectoryPtr[wayPoints.size()];
  
  auto mArm = ada->getArm();
  auto mArmSpace = mArm->getStateSpace();
  auto metaSkeleton = mArm->getMetaSkeleton();
  // auto startState = mArmSpace->createState();
//   State* startState;
  // auto startState = ada->getArm()->getMetaSkeleton().get()->getPositions()
  auto startState = mArmSpace->createState();
  mArmSpace->getState(metaSkeleton.get(), startState);

  int i = 0;
  for (auto pose = wayPoints.begin(); pose != wayPoints.end(); ++pose) {
      delta_x = (*pose).z - last_pose.z;
      delta_z = (*pose).y - last_pose.y;
      delta_roll = (*pose).roll_angle - last_pose.roll_angle;
      delta_l = sqrt(delta_x*delta_x + delta_z*delta_z); // length
      delta_x = delta_x/delta_l;
      delta_z = delta_z/delta_l;
      // std::cout << "waypoint = [" << (*pose).z << ", " << (*pose).y <<", "<< (*pose).roll_angle << "]" << std::endl;
      // std::cout << "direction = [" << delta_x << ", " << 0 <<", "<< delta_z << "]" << std::endl;
      // std::cout << "length = " << delta_l << std::endl;

      Eigen::Vector3d direction = Eigen::Vector3d(delta_x, 0.0, delta_z);
      // Eigen::Isometry3d goalPose = createIsometry((*pose).z, 0.0, (*pose).y, 0, 0, 0)
      Eigen::Isometry3d goalPose = createIsometry(0, 0.0, 0, 0, 0, 0);
      if (i != 0)
          goalPose = createIsometry((*pose).z, 0.0, (*pose).y, 0, 0, 0);

      // auto traj = ada->planArmToEndEffectorOffset(
      // direction,
      // delta_l,
      // startState,
      // collisionFree,
      // planningTimeout,
      // endEffectorOffsetPositionTolerance,
      // endEffectorOffsetAngularTolerance);

      Eigen::VectorXd twists(6);
      twists << delta_x, 0.0, delta_z, 0.0, 0.0, 0.0;
      // auto traj = ada->planWithEndEffectorTwist(
      //   twists,
      //   0.1,
      //   collisionFree,
      //   planningTimeout,
      //   endEffectorOffsetPositionTolerance,
      //   endEffectorOffsetAngularTolerance);  

      auto traj = ada->planToEndEffectorPose(
      goalPose,
      1, //??? test it 
      // State* startState,
      collisionFree,
      planningTimeout,
      0.001);

      auto _traj = dynamic_cast<Interpolated*>(traj.get());
      // _traj->evaluate(_traj->getEndTime(), startState);
      ada->moveArmOnTrajectory(traj, collisionFree, ada::KUNZ, velocityLimits);

      traj_vector[i] = traj;
      std::cout << "round " << i++ << std::endl; 
      last_pose = (*pose);
  } 

  TrajectoryPtr trajnext, concatenatedTraj = traj_vector[0];

  // concatenate all trajs
  for (int k=1; k<wayPoints.size(); k++) {
    trajnext = traj_vector[k]; 
    concatenatedTraj = concatenate(
      *dynamic_cast<Interpolated*>(concatenatedTraj.get()), 
      *dynamic_cast<Interpolated*>(trajnext.get())
    );
  }
  ada->moveArmOnTrajectory(concatenatedTraj, collisionFree, ada::KUNZ, velocityLimits);

  // test twists---- it works! nice!
  return true;
}

} // namespace action
} // namespace feeding
