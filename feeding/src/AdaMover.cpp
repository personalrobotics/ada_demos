#include "feeding/AdaMover.hpp"
#include <aikido/constraint/TestableIntersection.hpp>
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
AdaMover::AdaMover(
    ada::Ada& ada,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr armSpace,
    aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
    ros::NodeHandle nodeHandle)
  : mAda(ada)
  , mArmSpace(armSpace)
  , mCollisionFreeConstraint(collisionFreeConstraint)
  , mNodeHandle(nodeHandle)
{
}

//==============================================================================
bool AdaMover::moveArmToTSR(const aikido::constraint::dart::TSR& tsr)
{
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(tsr);

  auto trajectory = mAda.planToTSR(
      mArmSpace,
      mAda.getArm()->getMetaSkeleton(),
      mAda.getHand()->getEndEffectorBodyNode(),
      goalTSR,
      mCollisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle));

  return moveArmOnTrajectory(trajectory);
}

//==============================================================================
bool AdaMover::moveToEndEffectorOffset(
    const Eigen::Vector3d& direction, double length, bool respectCollision)
{
  return moveArmOnTrajectory(planToEndEffectorOffset(direction, length, respectCollision), TRYOPTIMALRETIME);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr AdaMover::planToEndEffectorOffset(
    const Eigen::Vector3d& direction, double length, bool respectCollision)
{
  ROS_INFO_STREAM("Plan to end effector offset state: " << mAda.getArm()->getMetaSkeleton()->getPositions().matrix().transpose());
  ROS_INFO_STREAM("Plan to end effector offset direction: " << direction.matrix().transpose() << ",  length: " << length);

  return mAda.planToEndEffectorOffset(
      mArmSpace,
      mAda.getArm()->getMetaSkeleton(),
      mAda.getHand()->getEndEffectorBodyNode(),
      respectCollision ? mCollisionFreeConstraint : nullptr,
      direction,
      length,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<double>(
          "/planning/endEffectorOffset/positionTolerance", mNodeHandle),
      getRosParam<double>(
          "/planning/endEffectorOffset/angularTolerance", mNodeHandle));
}

//==============================================================================
bool AdaMover::moveArmToConfiguration(const Eigen::Vector6d& configuration)
{
  auto trajectory = mAda.planToConfiguration(
      mArmSpace,
      mAda.getArm()->getMetaSkeleton(),
      configuration,
      mCollisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle));

  return moveArmOnTrajectory(trajectory, SMOOTH);
}

//==============================================================================
bool AdaMover::moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    TrajectoryPostprocessType postprocessType)
{
  if (!trajectory)
  {
    throw std::runtime_error("Trajectory execution failed: Empty trajectory.");
  }

  std::vector<aikido::constraint::ConstTestablePtr> constraints;
  if (mCollisionFreeConstraint)
  {
    constraints.push_back(mCollisionFreeConstraint);
  }
  auto testable = std::make_shared<aikido::constraint::TestableIntersection>(
      mArmSpace, constraints);

  aikido::trajectory::TrajectoryPtr timedTrajectory;
  switch (postprocessType)
  {
    case RETIME:
      timedTrajectory
          = mAda.retimePath(mAda.getArm()->getMetaSkeleton(), trajectory.get());
      break;

    case SMOOTH:
      timedTrajectory = mAda.smoothPath(
          mAda.getArm()->getMetaSkeleton(), trajectory.get(), testable);
      break;

    case TRYOPTIMALRETIME:
      timedTrajectory = mAda.retimeTimeOptimalPath(
          mAda.getArm()->getMetaSkeleton(), trajectory.get());

      if (!timedTrajectory)
      {
        // If using time-optimal retining failed, back to parabolic timing
        timedTrajectory = mAda.retimePath(
            mAda.getArm()->getMetaSkeleton(), trajectory.get());
      }
      break;

    default:
      throw std::runtime_error(
          "Feeding demo: Unexpected trajectory post processing type!");
  }

  auto future = mAda.executeTrajectory(std::move(timedTrajectory));
  try
  {
    future.get();
  }
  catch (const std::exception& e)
  {
    ROS_INFO_STREAM("trajectory execution failed: " << e.what());
    return false;
  }
  return true;
}
}
