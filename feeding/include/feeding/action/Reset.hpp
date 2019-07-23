#ifndef FEEDING_ACTION_RESET_HPP_
#define FEEDING_ACTION_RESET_HPP_

#include <libada/Ada.hpp>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/Workspace.hpp"
#include "feeding/perception/Perception.hpp"
#include "feeding/FeedingDemo.hpp"

namespace feeding {
namespace action {

bool reset(
	const std::shared_ptr<ada::Ada>& ada,
	const aikido::constraint::dart::CollisionFreePtr& collisionFree,
	const Eigen::Isometry3d& plate,
	const Eigen::Isometry3d& plateEndEffectorTransform,
	double horizontalToleranceAbovePlate,
	double verticalToleranceAbovePlate,
	double rotationToleranceAbovePlate,
	double planningTimeout,
	double endEffectorOffsetPositionTolerance,
	double endEffectorOffsetAngularTolerance,
	int maxNumTrials,
	std::vector<double> velocityLimits,
  const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper);
}
}

#endif
