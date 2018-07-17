#ifndef FEEDING_FEEDINGDEMO_HPP_
#define FEEDING_FEEDINGDEMO_HPP_

#include <aikido/planner/World.hpp>
#include <ros/ros.h>
#include <libada/Ada.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

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

  /// Gets Ada
  ada::Ada& getAda();

  /// Prints the configuration of the robot joints.
  void printRobotConfiguration();

  /// Opens Ada's hand
  void openHand();

  /// Closes Ada's hand
  void closeHand();

  /// Moves the robot to the start configuration as defined in the ros
  /// parameter.
  void moveToStartConfiguration();

  /// Moves the forque above the plate.
  void moveAbovePlate();

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
  aikido::constraint::dart::CollisionFreePtr mCollisionFreeConstraint;
};
}

#endif
