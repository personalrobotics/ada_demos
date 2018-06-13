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
  const ros::NodeHandle& nodeHandle;
  aikido::planner::WorldPtr world;

  std::unique_ptr<ada::Ada> ada;
  std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> armSpace;
  std::unique_ptr<Workspace> workspace;
  std::shared_ptr<aikido::constraint::dart::CollisionFree> collisionFreeConstraint;

public:
  FeedingDemo(bool adaReal, const ros::NodeHandle& nodeHandle);

  aikido::planner::WorldPtr getWorld() {return world;}
  std::unique_ptr<Workspace>& getWorkspace() {return workspace;}
  std::unique_ptr<ada::Ada>& getAda() {return ada;}
  Eigen::Isometry3d getDefaultFoodTransform() {return workspace->getDefaultFoodItem()->getRootBodyNode()->getWorldTransform();}

  bool isCollisionFree(std::string& result);
  void printRobotConfiguration();

  void openHand();
  void closeHand();

  void grabFoodWithForque();
  void ungrabAndDeleteFood();

  void moveAbovePlate();
  void moveAboveFood(Eigen::Isometry3d foodTransform);
  void moveIntoFood();
  void moveOutOfFood();
  void moveInFrontOfPerson();
  void moveTowardsPerson();
  void moveAwayFromPerson();

  bool moveArmToTSR(aikido::constraint::dart::TSR& tsr);
  bool moveWithEndEffectorOffset(Eigen::Vector3d direction, double length);

  bool moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    TrajectoryPostprocessType postprocessType = SMOOTH);

};

}

#endif