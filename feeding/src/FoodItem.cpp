#include "feeding/FoodItem.hpp"

namespace feeding {

//==============================================================================
FoodItem::FoodItem(
  std::string name,
  std::string uid,
  dart::dynamics::MetaSkeletonPtr metaSkeleton)
: mName(name)
, mUid(uid)
, mMetaSkeleton(metaSkeleton)
{
  if (!mMetaSkeleton)
    throw std::invalid_argument("MetaSkeleton is nullptr.");
}

//==============================================================================
Eigen::Isometry3d FoodItem::getPose() const
{
  return mMetaSkeleton->getBodyNode(0)->getWorldTransform();
}

//==============================================================================
std::string FoodItem::getName() const
{
  return mName;
}

//==============================================================================
std::string FoodItem::getUid() const
{
  return mUid;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr FoodItem::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//==============================================================================
FoodItemWithActionScore::FoodItemWithActionScore(
  FoodItem item, AcquisitionAction action, double score)
: mItem(item)
, mAction(action)
, mScore(score)
{
  // Do nothing
}

//==============================================================================
FoodItem* FoodItemWithActionScore::getItem()
{
  return &mItem;
}

//==============================================================================
AcquisitionAction const* FoodItemWithActionScore::getAction() const
{
  return &mAction;
}

//==============================================================================
double FoodItemWithActionScore::getScore() const
{
  return mScore;
}

} // namespace feeding