// Dana + Ramya

#ifndef FEEDING_ACTION_DETECTFOOD_HPP_
#define FEEDING_ACTION_DETECTFOOD_HPP_

#include "feeding/perception/Perception.hpp"

namespace feeding {
namespace action {

std::vector<std::unique_ptr<FoodItem>> detectFood(
  const std::shared_ptr<Perception>& perception,
  const std::string& foodName);

} // namespace feeding
} // namespace action

#endif
