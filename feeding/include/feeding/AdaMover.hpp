#ifndef FEEDING_ADAMOVER_HPP_
#define FEEDING_ADAMOVER_HPP_

#include <libada/Ada.hpp>

namespace feeding {

enum TrajectoryPostprocessType
{
  RETIME,
  SMOOTH,
  TRYOPTIMALRETIME
};

class AdaMover
{

public:
  AdaMover(
      ada::Ada& ada,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr armSpace,
      aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
      ros::NodeHandle nodeHandle);

  /// Moves the end effector to a TSR.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveArmToTSR(const aikido::constraint::dart::TSR& tsr);

  /// Moves the end effector along a certain position offset.
  /// Throws a runtime_error if no trajectory could be found.
  /// \return True if the trajectory was completed successfully.
  bool moveToEndEffectorOffset(const Eigen::Vector3d& direction, double length);

  bool moveWithEndEffectorTwist(
    const Eigen::Vector6d& transform, double duration, double timelimit);

  aikido::trajectory::TrajectoryPtr planToEndEffectorOffset(
      const Eigen::Vector3d& direction, double length);


  aikido::trajectory::TrajectoryPtr planWithEndEffectorTwist(
    const Eigen::Vector6d& transform, double duration, double timelimit);

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

  ada::Ada& mAda;
private:
  ros::NodeHandle mNodeHandle;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mArmSpace;
  aikido::constraint::dart::CollisionFreePtr mCollisionFreeConstraint;
};
}

#endif
