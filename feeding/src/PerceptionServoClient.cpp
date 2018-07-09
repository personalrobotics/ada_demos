#include "feeding/PerceptionServoClient.hpp"

namespace feeding {

  PerceptionServoClient::PerceptionServoClient(
      ::ros::NodeHandle node,
      std::shared_ptr<Perception> perception,
      aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      ::dart::dynamics::ConstBodyNodePtr bodyNode,
      std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> trajectoryExecutor,
      double perceptionUpdateTime
  ) : mNode(node)
    , mPerception(perception)
    , mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
    , mMetaSkeleton(std::move(metaSkeleton))
    , mBodyNode(bodyNode)
    , mTrajectoryExecutor(trajectoryExecutor)
    , mPerceptionUpdateTime(perceptionUpdateTime)
    , mCurrentTrajectory(nullptr)
  {
    mNonRealtimeTimer = mNode.createTimer(
      ros::Duration(mPerceptionUpdateTime), &PerceptionServoClient::nonRealtimeCallback,
      this, false, false);

    // initially set the current pose as the goal pose
    mGoalPose = mBodyNode->getTransform();
    mLastGoalPose = mGoalPose; 
  }

  PerceptionServoClient::~PerceptionServoClient()
  {
    // DO NOTHING
  }

  void PerceptionServoClient::start()
  {
    mNonRealtimeTimer.start();
  }

  void PerceptionServoClient::stop()
  {
    // Always abort the executing trajectory when quitting
    mTrajectoryExecutor->abort();
    mNonRealtimeTimer.stop();
  }

  void PerceptionServoClient::nonRealtimeCallback(const ros::TimerEvent& event)
  {
    if(updatePerception(mGoalPose))
    {
      // if | mGoalPose - mLastGoalPose | > linearTolerance angularTolerance
      // TODO: calc linear deviation and angular deviation
      if( true )
      {      

        mTrajectoryExecutor->abort();

        // TODO: update current Skeleton 

        // Generate a new reference trajectory to the goal pose
        mCurrentTrajectory = planToGoalPose(mGoalPose);
        
        // Execute the new reference trajectory
        if(mCurrentTrajectory)
        {
          mTrajectoryExecutor->execute(mCurrentTrajectory);
        }
      }  

      // updateGoalPose
      mLastGoalPose = mGoalPose;
    }
  }

  bool PerceptionServoClient::updatePerception(Eigen::Isometry3d& goalPose)
  {
    if(mPerception)
    {
      // update new goal Pose
      return mPerception->perceiveFood(goalPose);
    }
    return false;
  }

  aikido::trajectory::SplinePtr PerceptionServoClient::planToGoalPose(const Eigen::Isometry3d& goalPose)
  {
    // TODO: get the goal configuration from the goal pose using IK

    // TODO: get the current configuration
   
    // TODO: plan a trajectory from the current configuration to the goal configuration

    return nullptr;
  }
}
