#ifndef FEEDING_ACTION_MOVEINTO_HPP_
#define FEEDING_ACTION_MOVEINTO_HPP_

#include <libada/Ada.hpp>

#include "feeding/TargetItem.hpp"
#include "feeding/Workspace.hpp"
#include "feeding/perception/Perception.hpp"

#include "feeding/FTThresholdHelper.hpp"

namespace feeding {
namespace action {

bool moveInto(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Perception>& perception,
    const ::ros::NodeHandle* nodeHandle,
    std::shared_ptr<FTThresholdHelper> ftThresholdHelper,
    double velocityLimit,
    const std::string mFoodUid,
    Eigen::Vector3d foodOffset = Eigen::Vector3d(0, 0, 0));

} // namespace action
} // namespace feeding
#endif