#ifndef FEEDING_FOODITEM_HPP_
#define FEEDING_FOODITEM_HPP_

#include <dart/dart.hpp>
#include <aikido/perception/DetectedObject.hpp>
#include <aikido/common/pointers.hpp>
#include "feeding/AcquisitionAction.hpp"

namespace feeding {

AIKIDO_DECLARE_POINTERS(FoodItemWithActionScore)

struct FoodItem
{
  std::string name;
  std::string uid; // unique id necessary for tracking
  dart::dynamics::MetaSkeletonPtr skeleton;
  Eigen::Isometry3d pose;
};

struct FoodItemWithActionScore
{
  FoodItem item;
  AcquisitionAction action;
  double score;
};

FoodItemWithActionScorePtr createFoodItemWithActionScore(
  const aikido::perception::DetectedObject& item);

} // namespace feeding

#endif
