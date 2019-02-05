#ifndef FEEDING_FOODITEM_HPP_
#define FEEDING_FOODITEM_HPP_

#include <dart/dart.hpp>
#include <aikido/perception/DetectedObject.hpp>
#include <aikido/common/pointers.hpp>

namespace feeding {

AIKIDO_DECLARE_POINTERS(FoodItemWithActionScore)

struct FoodItem
{
  const std::string name;
  const std::string uid; // unique id necessary for tracking
  const dart::dynamics::MetaSkeletonPtr skeleton;
  const Eigen::Isometry3d pose;
};

struct FoodItemWithActionScore
{
  FoodItem item;
  AcquisitionAction action;
  const double score;
};

FoodItemWithActionScorePtr createFoodItemWithActionScore(
  const aikido::perception::DetectedObject& item);

} // namespace feeding

#endif
