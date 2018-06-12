#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include <pr_tsr/plate.hpp>
#include <aikido/constraint/TestableIntersection.hpp>


namespace feeding {

FeedingDemo::FeedingDemo(bool adaReal, ros::NodeHandle& nodeHandle) :
  adaReal(adaReal),
  nodeHandle(nodeHandle) {


  world = std::make_shared<aikido::planner::World>("feeding");

  ada = std::unique_ptr<ada::Ada>(new ada::Ada(
      world,
      !adaReal,
      getRosParam<std::string>("/adaUrdfUri", nodeHandle),
      getRosParam<std::string>("/adaSrdfUri", nodeHandle),
      getRosParam<std::string>("/endEffectorName", nodeHandle)));
  armSpace = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(ada->getArm()->getMetaSkeleton().get());

  if (!adaReal) {
    auto home = getRosParam<std::vector<double>>("/homeConfiguration", nodeHandle);
    ada->getArm()->getMetaSkeleton()->setPositions(Eigen::Vector6d(home.data()));
  }

  Eigen::Isometry3d robotPose = createIsometry(getRosParam<std::vector<double>>("/robotPose", nodeHandle));

  workspace = std::unique_ptr<Workspace>(new Workspace(world, robotPose, nodeHandle));

  // Setting up collisions
  dart::collision::CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(
          ada->getMetaSkeleton().get(), ada->getHand()->getEndEffectorBodyNode());
  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup(
          workspace->getTable().get(), workspace->getTom().get(), workspace->getWorkspaceEnvironment().get());
  collisionFreeConstraint = std::make_shared<aikido::constraint::dart::CollisionFree>(
      armSpace, ada->getArm()->getMetaSkeleton(), collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);    
}

bool FeedingDemo::isCollisionFree(std::string& result) {
  auto robotState
      = ada->getStateSpace()->getScopedStateFromMetaSkeleton(ada->getMetaSkeleton().get());
  aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
  if (!collisionFreeConstraint->isSatisfied(robotState, &collisionCheckOutcome)) {
    result =  "Robot is in collison: " + collisionCheckOutcome.toString();
    return false;
  }
  result = "Robot is not in collision";
  return true;
}

void FeedingDemo::openHand() {
  ada->getHand()->executePreshape("open").wait();
}

void FeedingDemo::closeHand() {
  ada->getHand()->executePreshape("closed").wait();
}

void FeedingDemo::moveAbovePlate() {
  double heightAbovePlate = getRosParam<double>("/heightAbovePlate", nodeHandle);
  double horizontalToleranceAbovePlate = getRosParam<double>("/horizontalToleranceAbovePlate", nodeHandle);
  double verticalToleranceAbovePlate = getRosParam<double>("/verticalToleranceAbovePlate", nodeHandle);

  auto abovePlateTSR = pr_tsr::getDefaultPlateTSR();
  abovePlateTSR.mT0_w = workspace->getPlate()->getRootBodyNode()->getWorldTransform();
  abovePlateTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, heightAbovePlate};

  abovePlateTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceAbovePlate, verticalToleranceAbovePlate, 0, 0);
  abovePlateTSR.mTw_e.matrix()
      *= ada->getHand()->getEndEffectorTransform("plate")->matrix();

  bool successfulMove = moveArmToTSR(abovePlateTSR);
  if (!successfulMove)
  {
    throw std::runtime_error("Trajectory execution failed");
  }
}

bool FeedingDemo::moveArmToTSR(aikido::constraint::dart::TSR& tsr)
{
  auto goalTSR = std::make_shared<aikido::constraint::dart::TSR>(tsr);

  auto trajectory = ada->planToTSR(
      armSpace,
      ada->getArm()->getMetaSkeleton(),
      ada->getHand()->getEndEffectorBodyNode(),
      goalTSR,
      collisionFreeConstraint,
      getRosParam<double>("/planningTimeoutSeconds", nodeHandle),
      getRosParam<int>("/maxNumberOfTrials", nodeHandle));

  return moveArmOnTrajectory(trajectory);
}

bool FeedingDemo::moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    TrajectoryPostprocessType postprocessType)
{
  if (!trajectory)
  {
    throw std::runtime_error("Failed to find a solution: Empty trajectory.");
  }

  std::vector<aikido::constraint::ConstTestablePtr> constraints;
  if (collisionFreeConstraint)
  {
    constraints.push_back(collisionFreeConstraint);
  }
  auto testable = std::make_shared<aikido::constraint::TestableIntersection>(
      armSpace, constraints);

  aikido::trajectory::TrajectoryPtr timedTrajectory;
  switch (postprocessType)
  {
    case RETIME:
      timedTrajectory = ada->retimePath(ada->getArm()->getMetaSkeleton(), trajectory.get());
      break;

    case SMOOTH:
      timedTrajectory
          = ada->smoothPath(ada->getArm()->getMetaSkeleton(), trajectory.get(), testable);
      break;

    default:
      throw std::runtime_error(
          "Feeding demo: Unexpected trajectory post processing type!");
  }

  auto future = ada->executeTrajectory(std::move(timedTrajectory));
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

};