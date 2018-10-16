#ifndef FEEDING_PERCEPTIONSERVOCLIENT_HPP_
#define FEEDING_PERCEPTIONSERVOCLIENT_HPP_

#include <mutex>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "feeding/AdaMover.hpp"
#include "feeding/Perception.hpp"

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
      const Eigen::VectorXd& veloctiyLimits,
      float originalDirectionExtension,
      float goalPrecision);
  virtual ~PerceptionServoClient();

  void start();

  void stop();

  bool isRunning();

  bool wait(double timelimit);

protected:
  void nonRealtimeCallback(const ros::TimerEvent& event);

  void jointStateUpdateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  bool updatePerception(Eigen::Isometry3d& goalPose);
  aikido::trajectory::SplinePtr planToGoalPose(
      const Eigen::Isometry3d& goalPose);

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

  aikido::trajectory::SplinePtr mCurrentTrajectory;
  std::future<void> mExec;

  ros::Timer mNonRealtimeTimer;

  Eigen::VectorXd mMaxVelocity;
  Eigen::VectorXd mMaxAcceleration;

  Eigen::VectorXd mCurrentPosition;
  Eigen::VectorXd mCurrentVelocity;

  Eigen::Isometry3d mOriginalPose;
  Eigen::VectorXd mOriginalConfig;

  aikido::constraint::dart::CollisionFreePtr mCollisionFreeConstraint;

  std::vector<dart::dynamics::SimpleFramePtr> mFrames;
  std::vector<aikido::rviz::FrameMarkerPtr> mFrameMarkers;
  bool mExecutionDone;
  bool mIsRunning;
  bool mNotFailed;

  ros::Subscriber mSub;
  std::mutex mJointStateUpdateMutex;
  std::mutex timerMutex;

  AdaMover* mAdaMover;

  std::chrono::time_point<std::chrono::system_clock> mStartTime;
  std::chrono::time_point<std::chrono::system_clock> mLastSuccess;

  bool hasOriginalDirection = false;
  Eigen::Vector3d originalDirection;

  float mOriginalDirectionExtension = 0;
  float mGoalPrecision = 0.01;
};
}

#endif // FEEDING_PERCEPTIONSERVOCLIENT_HPP_
