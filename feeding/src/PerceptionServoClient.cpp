#include "feeding/PerceptionServoClient.hpp"

namespace feeding {

  PerceptionServoClient::PerceptionServoClient(
      ::ros::NodeHandle node,
      std::shared_ptr<Perception> perception,
      std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> trajectoryExecutor,
      double perceptionUpdateTime
  ) : mNode(node)
    , mPerception(perception)
    , mTrajectoryExecutor(trajectoryExecutor)
    , mPerceptionUpdateTime(perceptionUpdateTime)
    , mCurrentTrajectory(nullptr)
  {
    mNonRealtimeTimer = mNode.createTimer(
      ros::Duration(mPerceptionUpdateTime), &PerceptionServoClient::nonRealtimeCallback,
      this, false, false);
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
    mTrajectoryExecutor->abort();
    mNonRealtimeTimer.stop();
  }

  void PerceptionServoClient::nonRealtimeCallback(const ros::TimerEvent& event)
  {
    if(updatePerception(mGoalPose))
    {
      // if | mGoalPose - mLastGoalPose | > linearTolerance angularTolerance
      if( true )
      {      

        mTrajectoryExecutor->abort();
        mCurrentTrajectory = planToGoalPose(mGoalPose);
        
        // update current Skeleton 

        mTrajectoryExecutor->execute(mCurrentTrajectory);
      }  

      // updateGoalPose
      mLastGoalPose = mGoalPose;
    }
  }

  bool PerceptionServoClient::updatePerception(Eigen::Isometry3d& goalPose)
  {
    if(mPerception)
    {
      return mPerception->perceiveFood(goalPose);
    }
    return false;
  }

  aikido::trajectory::SplinePtr PerceptionServoClient::planToGoalPose(const Eigen::Isometry3d& goalPose)
  {
    return nullptr;
  }
}
