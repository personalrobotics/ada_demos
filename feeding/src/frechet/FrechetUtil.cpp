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

// Helpers that otherwise aren't useful, so we keep them in this anon namespace.
namespace {

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

double so2Distance(double first, double second)
{
  // https://stackoverflow.com/questions/9505862/shortest-distance-between-two-
  // degree-marks-on-a-circle
  double raw_diff = first > second ? first - second : second - first;
  double mod_diff = std::fmod(raw_diff, 2*M_PI);
  double dist = mod_diff > M_PI ? 2*M_PI - mod_diff : mod_diff;

  return dist;
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

double computeSE3Distance(
  const Eigen::Isometry3d& firstPose,
  const Eigen::Isometry3d& secondPose
) {
  Eigen::VectorXd errorComponents = Eigen::VectorXd::Zero(6);

  // Include translational components.
  const Eigen::Vector3d& firstTrans = firstPose.translation();
  const Eigen::Vector3d& secondTrans = secondPose.translation();
  for (int i = 0; i < 3; i++)
  {
    errorComponents[i] = firstTrans[i] - secondTrans[i];
  }

  // And rotational components.
  const Eigen::Vector3d& firstEuler
    = dart::math::matrixToEulerXYZ(firstPose.linear());
  const Eigen::Vector3d& secondEuler
    = dart::math::matrixToEulerXYZ(secondPose.linear());
  for (int i = 0; i < 3; i++)
  {
    errorComponents[i + 3] = so2Distance(firstEuler[i], secondEuler[i]);
  }

  // TODO: Tweak weights!
  double rotWeight = 1.0;
  Eigen::VectorXd weights = Eigen::VectorXd::Zero(6);
  weights << 1.0, 1.0, 1.0, rotWeight, rotWeight, rotWeight;

  Eigen::VectorXd finalWeights = errorComponents.cwiseProduct(weights);
  return finalWeights.norm();
}

Eigen::Isometry3d computeFK(
  Eigen::VectorXd& config,
  MetaSkeletonPtr arm,
  BodyNodePtr hand
) {
  auto saver = MetaSkeletonStateSaver(
      arm, MetaSkeletonStateSaver::Options::POSITIONS);
  DART_UNUSED(saver);

  arm->setPositions(config);
  return hand->getTransform();
}

TrajectoryPtr planFollowEndEffectorPath(
    std::vector<Eigen::Isometry3d>& referencePath,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<ada::Ada>& ada
) {
  // TODO: Reset arm statespace at the end...

  auto armMetaSkeleton = ada->getArm()->getMetaSkeleton();
  auto armStateSpace
    = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          armMetaSkeleton.get());
  auto hand = ada->getHand()->getEndEffectorBodyNode();

  // TODO: Decide what to do about this.
  auto dummyStart = armStateSpace->getScopedStateFromMetaSkeleton(
      armMetaSkeleton.get());
  auto dummyGoal = armStateSpace->getScopedStateFromMetaSkeleton(
      armMetaSkeleton.get());

  // Includes self collision constraint.
  auto collisionTestable = ada->getFullCollisionConstraint(
      armStateSpace, armMetaSkeleton, collisionFree);

  // NOTE: NOT the DART ConfToConf problem.
  auto problem = aikido::planner::ConfigurationToConfiguration(
      armStateSpace, dummyStart, dummyGoal, collisionTestable);

  OMPLConfigurationToConfigurationPlanner<NNFrechet> plannerOMPL(
      armStateSpace, ada->cloneRNG().get());

  // TODO: Make everything below here a little nicer...

  auto corePlanner
      = dynamic_cast<NNFrechet*>(plannerOMPL.getOMPLPlanner().get());

  // Captured variables for NNF IK, FK, and task-space distance functions.
  auto ikSeedSampler
      = createSampleableBounds(armStateSpace, std::move(ada->cloneRNG()));
  std::shared_ptr<SampleGenerator> ikSeedGenerator
      = ikSeedSampler->createSampleGenerator();

  auto rightArmIK = InverseKinematics::create(hand);
  rightArmIK->setDofs(armMetaSkeleton->getDofs());

  auto omplStateSpace = corePlanner->getSpaceInformation()->getStateSpace();

  // Use required NNF setters.
  corePlanner->setRefPath(referencePath);

  corePlanner->setDistanceFunc(
      [](Eigen::Isometry3d& firstPose, Eigen::Isometry3d& secondPose) {
        // TODO: Incooreperate SE(3) component.
        return computeSE3Distance(firstPose, secondPose);
      });

  corePlanner->setFKFunc(
      [armMetaSkeleton, armStateSpace, hand](
          ompl::base::State* state) {
        auto saver = MetaSkeletonStateSaver(
            armMetaSkeleton, MetaSkeletonStateSaver::Options::POSITIONS);
        DART_UNUSED(saver);

        auto geometricState
            = static_cast<GeometricStateSpace::StateType*>(state);
        auto aikidoState = static_cast<MetaSkeletonStateSpace::State*>(
            geometricState->mState);

        armStateSpace->setState(armMetaSkeleton.get(), aikidoState);
        return hand->getTransform();
      });

  corePlanner->setIKFunc(
      [armMetaSkeleton,
       armStateSpace,
       ikSeedGenerator,
       rightArmIK,
       omplStateSpace](Eigen::Isometry3d& targetPose, int numSolutions) {
        auto saver = MetaSkeletonStateSaver(
            armMetaSkeleton, MetaSkeletonStateSaver::Options::POSITIONS);
        DART_UNUSED(saver);

        std::vector<ompl::base::State*> solutions;
        auto seedState = armStateSpace->createState();

        // How many times the IK solver will re-sample a single solution if it
        // is out of tolerance.
        int maxRetries = 2;
        for (int i = 0; i < numSolutions; i++)
        {
          for (int tryIndex = 0; tryIndex < maxRetries; tryIndex++)
          {
            if (!ikSeedGenerator->sample(seedState))
              continue;

            armStateSpace->setState(armMetaSkeleton.get(), seedState);
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
          armStateSpace->getState(
              armMetaSkeleton.get(), aikidoSolutionState);

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
