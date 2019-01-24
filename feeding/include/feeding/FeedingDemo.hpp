#ifndef FEEDING_FEEDINGDEMO_HPP_
#define FEEDING_FEEDINGDEMO_HPP_

#include <aikido/planner/World.hpp>
#include <ros/ros.h>
#include <libada/Ada.hpp>
#include "feeding/Workspace.hpp"

namespace feeding {

enum TrajectoryPostprocessType
{
  RETIME,
  SMOOTH,
};

/// The FeedingDemo class is responsible for
/// - The robot (loading + control)
/// - The workspace
///
/// It contains not only generalized functions, for example moveArmToTSR(...),
/// but also some functions that are very specialized for the feeding demo,
/// like moveInFrontOfPerson(). It uses the robot, workspace info and ros
/// parameters
/// to accomplish these tasks.
class FeedingDemo
{

public:
  /// Constructor for the Feeding Demo.
  /// Takes care of setting up the robot and the workspace
  /// \param[in] adaReal True if the real robot is used, false it's running in
  /// simulation.
  /// \param[in] useFTSensing turns the FTSensor and the
  /// MoveUntilTouchController on and off
  /// \param[in] nodeHandle Handle of the ros node.
  FeedingDemo(bool adaReal, bool useFTSensing, ros::NodeHandle nodeHandle);

  /// Destructor for the Feeding Demo.
  /// Also shuts down the trajectory controllers.
  ~FeedingDemo();

  /// Gets the aikido world
  aikido::planner::WorldPtr getWorld();

  /// Gets the workspace
  Workspace& getWorkspace();

  /// Gets Ada
  ada::Ada& getAda();

  /// Gets the transform of the default food object (defined in Workspace)
  Eigen::Isometry3d getDefaultFoodTransform();

  /// Checks robot and workspace collisions.
  /// \param[out] result Contains reason for collision.
  /// \return True if no collision was detected.
  bool isCollisionFree(std::string& result);

  /// Prints the configuration of the robot joints.
  void printRobotConfiguration();

  /// Opens Ada's hand
  void openHand();

  /// Closes Ada's hand
  void closeHand();

  /// Attach food to forque
  void grabFoodWithForque();

  /// Detach food from forque and remove it from the aikido world.
  void ungrabAndDeleteFood();

  /// Moves the robot to the start configuration as defined in the ros
  /// parameter.
  void moveToStartConfiguration();

  /// Moves the forque above the plate.
  void moveAbovePlate();

  /// Moves the forque above the food item using the values in the ros
  /// parameters.
  /// \param[in] foodTransform the transform of the food which the robot should
  /// move over.
  void moveAboveFood(const Eigen::Isometry3d& foodTransform);

  /// Moves the forque downwards into the food.
  /// This function does not throw an exception if the trajectory is aborted,
  /// because we expect that.
  void moveIntoFood();

  /// Moves the forque upwards above the food.
  void moveOutOfFood();

  /// Moves the forque to a position ready to approach the person.
  void moveInFrontOfPerson();

  /// Moves the forque towards the person.
  /// This function does not throw an exception if the trajectory is aborted,
  /// because we expect that.
  void moveTowardsPerson();

  /// Moves the forque away from the person.
  void moveAwayFromPerson();

  /// Moves the end effector to a TSR.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveArmToTSR(const aikido::constraint::dart::TSR& tsr);

  /// Moves the end effector along a certain position offset.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveWithEndEffectorOffset(
      const Eigen::Vector3d& direction, double length);

  /// Moves the robot to a configuration.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveArmToConfiguration(const Eigen::Vector6d& configuration);

  /// Postprocesses and executes a trjectory.
  /// Throws runtime_error if the trajectory is empty.
  /// \return True if the trajectory was completed successfully.
  bool moveArmOnTrajectory(
      aikido::trajectory::TrajectoryPtr trajectory,
      TrajectoryPostprocessType postprocessType = SMOOTH);

private:
  bool mAdaReal;
  ros::NodeHandle mNodeHandle;
  aikido::planner::WorldPtr mWorld;

  std::unique_ptr<ada::Ada> mAda;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mArmSpace;
  std::unique_ptr<Workspace> mWorkspace;
  aikido::constraint::dart::CollisionFreePtr mCollisionFreeConstraint;
};
}

#endif
