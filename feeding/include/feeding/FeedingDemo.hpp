#ifndef FEEDING_DEMO_HPP
#define FEEDING_DEMO_HPP

#include <ros/ros.h>
#include <libada/Ada.hpp>
#include <aikido/planner/World.hpp>
#include "feeding/Workspace.hpp"

namespace feeding {

enum TrajectoryPostprocessType
{
  RETIME,
  SMOOTH,
};

class FeedingDemo {

  bool adaReal;
  ros::NodeHandle nodeHandle;
  aikido::planner::WorldPtr world;

  std::unique_ptr<ada::Ada> ada;
  /*aikido::robot::ConcreteManipulatorPtr arm;
  dart::dynamics::MetaSkeletonPtr armSkeleton;
  std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> armSpace;
  auto hand = robot.getHand();*/
  std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> armSpace;
  std::unique_ptr<Workspace> workspace;
  std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint;

public:
  FeedingDemo(bool adaReal, ros::NodeHandle& nodeHandle);

  aikido::planner::WorldPtr getWorld() {return world;}

  bool isCollisionFree(std::string& result);

  void openHand();
  void closeHand();

  void moveAbovePlate();

  bool moveArmToTSR(aikido::constraint::dart::TSR& tsr);

  bool moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    TrajectoryPostprocessType postprocessType = SMOOTH);

};

}

#endif