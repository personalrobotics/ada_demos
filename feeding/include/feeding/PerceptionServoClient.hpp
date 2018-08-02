#ifndef FEEDING_PERCEPTIONSERVOCLIENT_HPP_
#define FEEDING_PERCEPTIONSERVOCLIENT_HPP_

#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include "feeding/Perception.hpp"
#include "feeding/AdaMover.hpp"

namespace feeding {

class PerceptionServoClient
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

  PerceptionServoClient(
      ::ros::NodeHandle node,
      boost::function<bool(Eigen::Isometry3d&)> getTransform,
      aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
          metaSkeletonStateSpace,
          AdaMover* adaMover,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      ::dart::dynamics::BodyNodePtr bodyNode,
      std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor>
          trajectoryExecutor,
      aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
      double perceptionUpdateTime,
      double goalPoseUpdateTolerance,
      const Eigen::VectorXd& veloctiyLimits);
  virtual ~PerceptionServoClient();

  void start();

  void stop();

  bool isRunning();

  void wait(double timelimit);

protected:
  void nonRealtimeCallback(const ros::TimerEvent& event);

  void jointStateUpdateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  bool updatePerception(Eigen::Isometry3d& goalPose);
  aikido::trajectory::SplinePtr planToGoalPose(
      const Eigen::Isometry3d& goalPose);

  std::unique_ptr<aikido::trajectory::Spline> timeTrajectoryUsingConstantVelocity(
      const aikido::trajectory::TrajectoryPtr traj,
      double constantVelocity);

  double getElapsedTime();

  ::ros::NodeHandle mNode;
  boost::function<bool(Eigen::Isometry3d&)> mGetTransform;
  /// Meta skeleton state space.
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
      mMetaSkeletonStateSpace;

  /// Meta Skeleton
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// BodyNode
  ::dart::dynamics::BodyNodePtr mBodyNode;

  std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor>
      mTrajectoryExecutor;
  double mPerceptionUpdateTime;
  double mGoalPoseUpdateTolerance;

  aikido::trajectory::SplinePtr mCurrentTrajectory;
  std::future<void> mExec;

  ros::Timer mNonRealtimeTimer;

  Eigen::Isometry3d mGoalPose;
  Eigen::Isometry3d mLastGoalPose;

  Eigen::VectorXd mMaxVelocity;
  Eigen::VectorXd mMaxAcceleration;

  Eigen::VectorXd mCurrentPosition;
  Eigen::VectorXd mCurrentVelocity;

  aikido::constraint::dart::CollisionFreePtr mCollisionFreeConstraint;

  std::vector<dart::dynamics::SimpleFramePtr> mFrames;
  std::vector<aikido::rviz::FrameMarkerPtr> mFrameMarkers;
  bool mExecutionDone;
  bool mIsRunning;

  ros::Subscriber mSub;
  std::mutex mJointStateUpdateMutex;
  std::mutex timerMutex;

  AdaMover* mAdaMover;

  std::chrono::time_point<std::chrono::system_clock> mStartTime;
};

}

#endif // FEEDING_PERCEPTIONSERVOCLIENT_HPP_
