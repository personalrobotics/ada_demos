#include "feeding/frechet/FrechetUtil.hpp"

#include <dart/math/Geometry.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/planner/ConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp>
#include <pr-ompl-frechet/NNFrechet.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace feeding {

using dart::collision::CollisionGroup;
using dart::dynamics::InverseKinematics;
using aikido::constraint::SampleGenerator;
using aikido::constraint::dart::createSampleableBounds;
using aikido::constraint::dart::CollisionFree;
using aikido::distance::createDistanceMetric;
using aikido::planner::ompl::GeometricStateSpace;
using aikido::planner::ompl::OMPLConfigurationToConfigurationPlanner;
using aikido::statespace::CartesianProduct;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::trajectory::Interpolated;
using NNFrechet::NNFrechet;

namespace {

// Helper for `readADAPath`, so kept in anon namespace.
std::vector<Eigen::VectorXd> readVectorsFromFile(
  std::string dirPath,
  int vectorLength
) {
  std::vector<Eigen::VectorXd> readVecs;

  std::ifstream recordFile(dirPath);
  std::string line;

  while(std::getline(recordFile, line))
  {
    Eigen::VectorXd readVector(vectorLength);

    std::stringstream ss;
    ss.str(line);
    std::string stringValue;
    for (int i = 0; i < vectorLength; i++)
    {
      // Break up by whitespace.
      std::getline(ss, stringValue, ' ');
      double numericalValue = boost::lexical_cast<double>(stringValue);
      readVector[i] = numericalValue;
    }

    readVecs.push_back(readVector);
  }

  return readVecs;
}

} // namespace

std::vector<Eigen::Isometry3d> readADAPath(
  std::string pathFile
) {
  std::vector<Eigen::Isometry3d> rawPoses;

  std::vector<Eigen::VectorXd> flatPoses = readVectorsFromFile(pathFile, 12);

  for (auto curFlat : flatPoses)
  {
    Eigen::Isometry3d curPose = Eigen::Isometry3d::Identity();

    curPose.translation()
      = Eigen::Vector3d(curFlat[0], curFlat[1], curFlat[2]);

     Eigen::Matrix3d rot;
     rot << curFlat[3], curFlat[4], curFlat[5],
            curFlat[6], curFlat[7], curFlat[8],
            curFlat[9], curFlat[10], curFlat[11];
    curPose.linear() = rot;

    rawPoses.push_back(curPose);
  }

  return rawPoses;
}

TrajectoryPtr planFollowEndEffectorPath(
    std::vector<Eigen::Isometry3d>& referencePath,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<ada::Ada>& ada
) {
  // TODO: Reset arm statespace at the end...

  auto rightArmMetaSkeleton = ada->getArm()->getMetaSkeleton();
  auto rightArmStateSpace
    = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          rightArmMetaSkeleton.get());
  auto rightHand = ada->getHand()->getEndEffectorBodyNode();

  // TODO: Decide what to do about this.
  auto dummyStart = rightArmStateSpace->getScopedStateFromMetaSkeleton(
      rightArmMetaSkeleton.get());
  auto dummyGoal = rightArmStateSpace->getScopedStateFromMetaSkeleton(
      rightArmMetaSkeleton.get());

  // Includes self collision constraint.
  auto collisionTestable = ada->getFullCollisionConstraint(
      rightArmStateSpace, rightArmMetaSkeleton, collisionFree);

  // NOTE: NOT the DART ConfToConf problem.
  auto problem = aikido::planner::ConfigurationToConfiguration(
      rightArmStateSpace, dummyStart, dummyGoal, collisionTestable);

  OMPLConfigurationToConfigurationPlanner<NNFrechet> plannerOMPL(
      rightArmStateSpace, ada->cloneRNG().get());

  auto corePlanner
      = dynamic_cast<NNFrechet*>(plannerOMPL.getOMPLPlanner().get());

  // Captured variables for NNF IK, FK, and task-space distance functions.
  auto ikSeedSampler
      = createSampleableBounds(rightArmStateSpace, std::move(ada->cloneRNG()));
  std::shared_ptr<SampleGenerator> ikSeedGenerator
      = ikSeedSampler->createSampleGenerator();

  auto rightArmIK = InverseKinematics::create(rightHand);
  rightArmIK->setDofs(rightArmMetaSkeleton->getDofs());

  auto omplStateSpace = corePlanner->getSpaceInformation()->getStateSpace();

  // Use required NNF setters.
  corePlanner->setRefPath(referencePath);

  corePlanner->setDistanceFunc(
      [](Eigen::Isometry3d& firstPose, Eigen::Isometry3d& secondPose) {
        // TODO: Incooreperate SE(3) component.
        return (firstPose.translation() - secondPose.translation()).norm();
      });

  corePlanner->setFKFunc(
      [rightArmMetaSkeleton, rightArmStateSpace, rightHand](
          ompl::base::State* state) {
        auto saver = MetaSkeletonStateSaver(
            rightArmMetaSkeleton, MetaSkeletonStateSaver::Options::POSITIONS);
        DART_UNUSED(saver);

        auto geometricState
            = static_cast<GeometricStateSpace::StateType*>(state);
        auto aikidoState = static_cast<MetaSkeletonStateSpace::State*>(
            geometricState->mState);

        rightArmStateSpace->setState(rightArmMetaSkeleton.get(), aikidoState);
        return rightHand->getTransform();
      });

  corePlanner->setIKFunc(
      [rightArmMetaSkeleton,
       rightArmStateSpace,
       ikSeedGenerator,
       rightArmIK,
       omplStateSpace](Eigen::Isometry3d& targetPose, int numSolutions) {
        auto saver = MetaSkeletonStateSaver(
            rightArmMetaSkeleton, MetaSkeletonStateSaver::Options::POSITIONS);
        DART_UNUSED(saver);

        std::vector<ompl::base::State*> solutions;
        auto seedState = rightArmStateSpace->createState();

        // How many times the IK solver will re-sample a single solution if it
        // is out of tolerance.
        int maxRetries = 2;
        for (int i = 0; i < numSolutions; i++)
        {
          for (int tryIndex = 0; tryIndex < maxRetries; tryIndex++)
          {
            if (!ikSeedGenerator->sample(seedState))
              continue;

            rightArmStateSpace->setState(rightArmMetaSkeleton.get(), seedState);
            rightArmIK->getTarget()->setTransform(targetPose);

            if (rightArmIK->solve(true))
              break;
          }

          auto solutionState = omplStateSpace->allocState();
          auto geometricSolutionState
              = static_cast<GeometricStateSpace::StateType*>(solutionState);
          auto aikidoSolutionState
              = static_cast<MetaSkeletonStateSpace::State*>(
                  geometricSolutionState->mState);
          rightArmStateSpace->getState(
              rightArmMetaSkeleton.get(), aikidoSolutionState);

          solutions.push_back(solutionState);
        }

        return solutions;
      });

  // Also set NNF params.
  corePlanner->setNumWaypoints(5);
  corePlanner->setIKMultiplier(10);
  corePlanner->setNumNN(10);
  corePlanner->setDiscretization(3);

  // TODO: Time without violating Frechet metric.
  auto untimedTraj = plannerOMPL.plan(problem);
  return untimedTraj;
}

} // namespace feeding
