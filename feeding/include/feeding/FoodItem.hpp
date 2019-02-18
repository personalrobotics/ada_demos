#ifndef FEEDING_FOODITEM_HPP_
#define FEEDING_FOODITEM_HPP_

#include <dart/dart.hpp>
#include <aikido/perception/DetectedObject.hpp>
#include <aikido/common/pointers.hpp>
#include "feeding/AcquisitionAction.hpp"

namespace feeding {

AIKIDO_DECLARE_POINTERS(FoodItemWithActionScore)

class FoodItem
{
public:
  explicit FoodItem(
    std::string name,
    std::string uid,
    dart::dynamics::MetaSkeletonPtr metaSkeleton);

  Eigen::Isometry3d getPose() const;

  std::string getName() const;

  std::string getUid() const;

  dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const;

private:
  const std::string mName;
  const std::string mUid; // unique id necessary for tracking
  const dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

class FoodItemWithActionScore
{
public:
  explicit FoodItemWithActionScore(
    FoodItem item, AcquisitionAction action, double score);

  FoodItem* getItem();
  AcquisitionAction const* getAction() const;
  double getScore() const;


private:

  FoodItem mItem;
  AcquisitionAction mAction;
  double mScore;
};

} // namespace feeding

#endif
