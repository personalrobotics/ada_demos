#include "feeding/FoodItem.hpp"

namespace feeding {

//==============================================================================
FoodItem::FoodItem(
    std::string name,
    std::string uid,
    dart::dynamics::MetaSkeletonPtr metaSkeleton,
    AcquisitionAction action,
    double score)
  : mName(name)
  , mUid(uid)
  , mMetaSkeleton(metaSkeleton)
  , mAction(action)
  , mScore(score)
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
AcquisitionAction const* FoodItem::getAction() const
{
  return &mAction;
}

//==============================================================================
double FoodItem::getScore() const
{
  return mScore;
}

} // namespace feeding