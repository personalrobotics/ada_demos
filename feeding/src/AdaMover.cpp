#include "feeding/AdaMover.hpp"
#include <aikido/constraint/TestableIntersection.hpp>
#include "feeding/util.hpp"
#include <memory>

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
bool AdaMover::moveArmToTSR(const aikido::constraint::dart::TSR& tsr, const std::vector<double>& velocityLimits) 
{
  return moveArmToTSR(tsr, velocityLimits, Eigen::VectorXd(0));
}

//==============================================================================
aikido::trajectory::TrajectoryPtr AdaMover::planArmToTSR(const aikido::constraint::dart::TSR& tsr, const Eigen::VectorXd& nominalConfiguration)
{
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(tsr);

  auto trajectory = mAda.planToTSR(
      mArmSpace,
      mAda.getArm()->getMetaSkeleton(),
      mAda.getHand()->getEndEffectorBodyNode(),
      goalTSR,
      nominalConfiguration,
      mCollisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle));

  return trajectory;
}

//==============================================================================
bool AdaMover::moveArmToTSR(const aikido::constraint::dart::TSR& tsr, const std::vector<double>& velocityLimits, const Eigen::VectorXd& nominalConfiguration)
{
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(tsr);

  auto trajectory = mAda.planToTSR(
      mArmSpace,
      mAda.getArm()->getMetaSkeleton(),
      mAda.getHand()->getEndEffectorBodyNode(),
      goalTSR,
      nominalConfiguration,
      mCollisionFreeConstraint,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle));

  return moveArmOnTrajectory(trajectory, TRYOPTIMALRETIME, velocityLimits);
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

  auto skeleton = mAda.getArm()->getMetaSkeleton();

  std::vector<int> indices{0,3,4,5};
  std::vector<double> tempLower{-6.28, -6.28, -6.28, -6.28};
  std::vector<double> tempUpper{6.28, 6.28, 6.28, 6.28};
  auto llimits = skeleton->getPositionLowerLimits();
  auto ulimits = skeleton->getPositionUpperLimits();
  for (int i = 0; i < indices.size(); ++i)
  {
      llimits(indices[i]) = tempLower[i];
      ulimits(indices[i]) = tempUpper[i];
  }
  skeleton->setPositionLowerLimits(llimits);
  skeleton->setPositionUpperLimits(ulimits);

  auto space = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(skeleton.get());

  auto trajectory = mAda.planToEndEffectorOffset(
      space,
      skeleton,
      mAda.getHand()->getEndEffectorBodyNode(),
      nullptr,
      direction,
      length,
      getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
      getRosParam<double>(
          "/planning/endEffectorOffset/positionTolerance", mNodeHandle),
      getRosParam<double>(
          "/planning/endEffectorOffset/angularTolerance", mNodeHandle));

    for (int i = 0; i < indices.size(); ++i)
    {
        llimits(indices[i]) = -dart::math::constantsd::inf();
        ulimits(indices[i]) = dart::math::constantsd::inf();
    }
    mAda.getArm()->getMetaSkeleton()->setPositionLowerLimits(llimits);
    mAda.getArm()->getMetaSkeleton()->setPositionUpperLimits(ulimits);

    return trajectory;
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

  return moveArmOnTrajectory(trajectory, TRYOPTIMALRETIME);
}

//==============================================================================
bool AdaMover::moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    TrajectoryPostprocessType postprocessType,
    std::vector<double> smoothVelocityLimits)
{
  if (!trajectory)
  {
    throw std::runtime_error("Trajectory execution failed: Empty trajectory.");
  }

//  auto trajectory = mAda.convertTrajectory(mAda.getArm()->getMetaSkeleton(), path.get());

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
      if (smoothVelocityLimits.size() == 6) {
        Eigen::Vector6d velocityLimits;
        velocityLimits << smoothVelocityLimits[0], smoothVelocityLimits[1], smoothVelocityLimits[2], smoothVelocityLimits[3], smoothVelocityLimits[4], smoothVelocityLimits[5];
        Eigen::VectorXd previousLowerLimits = mAda.getArm()->getMetaSkeleton()->getVelocityLowerLimits();
        Eigen::VectorXd previousUpperLimits = mAda.getArm()->getMetaSkeleton()->getVelocityUpperLimits();
        mAda.getArm()->getMetaSkeleton()->setVelocityLowerLimits(-velocityLimits);
        mAda.getArm()->getMetaSkeleton()->setVelocityUpperLimits(velocityLimits);
        timedTrajectory = mAda.smoothPath(
            mAda.getArm()->getMetaSkeleton(), trajectory.get(), testable);
        mAda.getArm()->getMetaSkeleton()->setVelocityLowerLimits(previousLowerLimits);
        mAda.getArm()->getMetaSkeleton()->setVelocityUpperLimits(previousUpperLimits);
      } else {
        timedTrajectory = mAda.smoothPath(
            mAda.getArm()->getMetaSkeleton(), trajectory.get(), testable);
      }
      break;

    case TRYOPTIMALRETIME:
      timedTrajectory = mAda.retimeTimeOptimalPath(
          mAda.getArm()->getMetaSkeleton(), trajectory.get());

      if (!timedTrajectory)
      {
        throw std::runtime_error("retiming failed!");
        // If using time-optimal retining failed, back to parabolic timing
        timedTrajectory = mAda.retimePath(
            mAda.getArm()->getMetaSkeleton(), trajectory.get());
      }
      // {
      // aikido::trajectory::SplinePtr traj = std::dynamic_pointer_cast<aikido::trajectory::Spline>(timedTrajectory);
      //   if (traj) {
      //     dumpSplinePhasePlot(*traj, "moveAboveFork.txt", 0.01);
      //   } else {
      //     ROS_INFO_STREAM("timed spline is null");
      //   }
      // }
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
