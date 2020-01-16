#include "feeding/FoodItem.hpp"
#include <yaml-cpp/exceptions.h>

namespace feeding {

//==============================================================================
FoodItem::FoodItem(
    std::string name,
    std::string uid,
    dart::dynamics::MetaSkeletonPtr metaSkeleton,
    AcquisitionAction action,
    double score,
    const std::string& yamlStr)
  : mName(name)
  , mUid(uid)
  , mMetaSkeleton(metaSkeleton)
  , mAction(action)
  , mScore(score)
{
  if (!mMetaSkeleton)
    throw std::invalid_argument("MetaSkeleton is nullptr.");
  try
  {
    // where to load it? and what's the yamlStr 
    mYamlNode = YAML::Load(yamlStr);
    // mYamlNode = YAML::Load("{push_direction: 'left', push_vec: [1,0,0]}");
  }
  catch (const YAML::Exception& e)
  {
    std::stringstream ss;
    ss << "[DetectedObject::DetectedObject] YAML String Exception: " << e.what()
       << std::endl;
    throw std::invalid_argument(ss.str());
  }

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

//==============================================================================
YAML::Node FoodItem::getYamlNode()
{
  return mYamlNode;
}

} // namespace feeding