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

namespace feeding {

// Read reference path for ADA to follow.
std::vector<Eigen::Isometry3d> readADAPath(std::string pathFile);

} // namespace feeding

#endif // FEEDING_FRECHET_UTIL_HPP_
