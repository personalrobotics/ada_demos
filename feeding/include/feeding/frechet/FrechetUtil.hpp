#ifndef FEEDING_FRECHET_UTIL_HPP_
#define FEEDING_FRECHET_UTIL_HPP_

#include <dart/dart.hpp>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include "aikido/distance/DistanceMetric.hpp"
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <libada/Ada.hpp>

using dart::dynamics::BodyNodePtr;
using dart::dynamics::InverseKinematicsPtr;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SharedLibraryIkFast;
using dart::dynamics::SkeletonPtr;
using aikido::constraint::Sampleable;
using aikido::constraint::TestablePtr;
using aikido::distance::DistanceMetricPtr;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::trajectory::InterpolatedPtr;
using aikido::trajectory::TrajectoryPtr;

namespace feeding {

// Read reference path for ADA to follow.
std::vector<Eigen::Isometry3d> readADAPath(std::string pathFile);

// Helper method to compute error in SE(3). Accounts for both position and
// orientation.
double computeSE3Distance(
  const Eigen::Isometry3d& firstPose,
  const Eigen::Isometry3d& secondPose);

// Uses NNF to follow the given `referencePath`.
TrajectoryPtr planFollowEndEffectorPath(
    std::vector<Eigen::Isometry3d>& referencePath,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    MetaSkeletonPtr armMetaSkeleton,
    MetaSkeletonStateSpacePtr armStateSpace,
    const std::shared_ptr<ada::Ada>& ada);

// Used after planFollowEndEffectorPath has generated a motion plan/trajectory.
// Moves ada to the first state on that trajectory for execution.
bool moveToStartOfTraj(
    TrajectoryPtr traj,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    MetaSkeletonPtr armMetaSkeleton,
    MetaSkeletonStateSpacePtr armStateSpace,
    const std::shared_ptr<ada::Ada>& ada);

} // namespace feeding

#endif // FEEDING_FRECHET_UTIL_HPP_
