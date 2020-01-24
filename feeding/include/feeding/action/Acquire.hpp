#ifndef FEEDING_ACTION_ACQUIRE_HPP_
#define FEEDING_ACTION_ACQUIRE_HPP_

#include <libada/Ada.hpp>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Workspace.hpp"
#include "feeding/perception/Perception.hpp"

namespace feeding {
namespace action {

bool acquire(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Workspace>& workspace,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    const std::string& foodName,
    const Eigen::Isometry3d& plate,
    const Eigen::Isometry3d& plateEndEffectorTransform,
    const std::unordered_map<std::string, double>& foodSkeweringForces,
    double horizontalToleranceAbovePlate,
    double verticalToleranceAbovePlate,
    double rotationToleranceAbovePlate,
    double heightAboveFood,
    double horizontalToleranceForFood,
    double verticalToleranceForFood,
    double rotationToleranceForFood,
    double tiltToleranceForFood,
    double moveOutofFoodLength,
    double endEffectorOffsetPositionTolerance,
    double endEffectorOffsetAngularTolerance,
    std::chrono::milliseconds waitTimeForFood,
    double planningTimeout,
    int maxNumTrials,
    std::vector<double> velocityLimits,
    const std::shared_ptr<FTThresholdHelper>& ftThresholdHelper,
    std::vector<std::string> rotationFreeFoodNames = std::vector<std::string>(),
    FeedingDemo* feedingDemo = nullptr
    
    const double incidentAngle = -1,
    const double force = -1,
    const double inFoodRotationAngle = -1,
    const double exitAngle = -1);
}
} // namespace feeding

#endif