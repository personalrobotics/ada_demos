#ifndef FEEDING_ACTION_MOVETOWARDSPERSON_HPP_
#define FEEDING_ACTION_MOVETOWARDSPERSON_HPP_

#include <libada/Ada.hpp>

#include "feeding/perception/Perception.hpp"
#include "feeding/perception/PerceptionServoClient.hpp"
#include "feeding/FeedingDemo.hpp"

namespace feeding {
namespace action {

bool moveTowardsPerson(
    const std::shared_ptr<ada::Ada>& ada,
    const std::shared_ptr<Perception>& perception,
    const ros::NodeHandle* nodeHandle,
    FeedingDemo* feedingDemo,
    double distanceFromPerson,
    double velocityLimit);
}
} // namespace feeding

#endif
