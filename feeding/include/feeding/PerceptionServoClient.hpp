#ifndef FEEDING_PERCEPTIONSERVOCLIENT_HPP_
#define FEEDING_PERCEPTIONSERVOCLIENT_HPP_

#include <dart/dynamics/BodyNode.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include "feeding/Perception.hpp"

namespace feeding {

class PerceptionServoClient 
{
public:
  PerceptionServoClient(
    ::ros::NodeHandle node,
    boost::function<bool (Eigen::Isometry3d&)> getTransform,
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::BodyNodePtr bodyNode,
    std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> trajectoryExecutor,
    aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
    aikido::rviz::WorldInteractiveMarkerViewer& viewer,
    double perceptionUpdateTime,
    double goalPoseUpdateTolerance
  );
  virtual ~PerceptionServoClient();

  void start();

  void stop();

  bool wait(double timelimit);
  
protected:
  void nonRealtimeCallback(const ros::TimerEvent& event);

  bool updatePerception(Eigen::Isometry3d& goalPose);  
  aikido::trajectory::SplinePtr planToGoalPose(const Eigen::Isometry3d& goalPose);

  ::ros::NodeHandle mNode;
  Perception* mPerception;
  /// Meta skeleton state space.
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
      mMetaSkeletonStateSpace;

  /// Meta Skeleton
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// BodyNode
  ::dart::dynamics::BodyNodePtr mBodyNode;

  std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> mTrajectoryExecutor;
  double mPerceptionUpdateTime;
  double mGoalPoseUpdateTolerance;

  aikido::trajectory::SplinePtr mCurrentTrajectory;
  std::future<void> mExec;
  
  ros::Timer mNonRealtimeTimer;
  Eigen::Isometry3d mGoalPose;
  Eigen::Isometry3d mLastGoalPose;

  Eigen::VectorXd mMaxVelocity;
  Eigen::VectorXd mMaxAcceleration;

  aikido::constraint::dart::CollisionFreePtr mCollisionFreeConstraint;
  aikido::rviz::WorldInteractiveMarkerViewer& mViewer;

  std::vector<dart::dynamics::SimpleFramePtr> mFrames;
  std::vector<aikido::rviz::FrameMarkerPtr> mFrameMarkers;
  bool mExecutionDone;
  
};

}

#endif // FEEDING_PERCEPTIONSERVOCLIENT_HPP_
