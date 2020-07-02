#ifndef FEEDING_ACTION_FEEDFOODTOPERSON_HPP_
#define FEEDING_ACTION_FEEDFOODTOPERSON_HPP_

#include <libada/Ada.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/perception/Perception.hpp"

// Contains motions which are mainly TSR actions
namespace feeding {
namespace action {

void feedFoodToPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    ros::Duration waitAtPerson,
    FeedingDemo* feedingDemo,
    std::vector<double> jointVelocityLimits,
    // Visual Servoing Params
    double servoVelocityLimit,
    double distanceFromPerson,
    // Returning to plate params
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    // Tilting params
    const Eigen::Vector3d* tiltOffset,
    double horizontalToleranceForPerson,
    double verticalToleranceForPerson,
    double planningTimeout,
    int maxNumTrials);
}
} // namespace feeding

#endif
