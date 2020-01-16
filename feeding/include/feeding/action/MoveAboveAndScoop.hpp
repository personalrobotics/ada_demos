#ifndef FEEDING_ACTION_MOVEABOVEANDSCOOP_HPP_
#define FEEDING_ACTION_MOVEABOVEANDSCOOP_HPP_

#include <libada/Ada.hpp>
#include "feeding/FeedingDemo.hpp"
#include "feeding/FoodItem.hpp"
#include "feeding/Workspace.hpp"


// Contains motions which are mainly TSR actions
namespace feeding {
// namespace action {

std::unique_ptr<FoodItem> moveAboveAndScoop(
    const std::shared_ptr<ada::Ada>& ada,
    const Eigen::Isometry3d& foodTransform,
    FeedingDemo& feedingDemo,
    ros::NodeHandle nodeHandle);
// }
} // namespace feeding

#endif