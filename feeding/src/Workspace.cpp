#include "feeding/Workspace.hpp"
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/util.hpp>

namespace feeding {

//==============================================================================
Workspace::Workspace(
    aikido::planner::WorldPtr world,
    const Eigen::Isometry3d& robotPose,
    bool adaReal,
    ros::NodeHandle nodeHandle)
  : mNodeHandle(nodeHandle), mWorld(world)
{

  addToWorld(mPlate, "plate", robotPose);
  addToWorld(mTable, "table", robotPose);
  addToWorld(mPerson, "person", robotPose);
  addToWorld(mWorkspaceEnvironment, "workspaceEnvironment", robotPose);
  addToWorld(mWheelchair, "wheelchair", Eigen::Isometry3d::Identity());

  if (!adaReal)
  {
    addToWorld(mDefaultFoodItem, "defaultFoodItem", robotPose);
    mDefaultFoodItem->getRootBodyNode()->setCollidable(false);
  }
}

//==============================================================================
void Workspace::addToWorld(
    dart::dynamics::SkeletonPtr& skeleton,
    const std::string& name,
    const Eigen::Isometry3d& robotPose)
{
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  std::string urdfUri
      = getRosParam<std::string>("/" + name + "/urdfUri", mNodeHandle);
  Eigen::Isometry3d pose
      = robotPose.inverse() * createIsometry(
                                  getRosParam<std::vector<double>>(
                                      "/" + name + "/pose", mNodeHandle));
  skeleton = loadSkeletonFromURDF(resourceRetriever, urdfUri, pose);
  mWorld->addSkeleton(skeleton);
}

//==============================================================================
void Workspace::deleteFood()
{
  mWorld->removeSkeleton(mDefaultFoodItem);
}

//==============================================================================
dart::dynamics::ConstSkeletonPtr Workspace::getPlate() const
{
  return mPlate;
}

//==============================================================================
dart::dynamics::ConstSkeletonPtr Workspace::getTable() const
{
  return mTable;
}

//==============================================================================
dart::dynamics::ConstSkeletonPtr Workspace::getWorkspaceEnvironment() const
{
  return mWorkspaceEnvironment;
}

//==============================================================================
dart::dynamics::SkeletonPtr Workspace::getDefaultFoodItem() const
{
  return mDefaultFoodItem;
}

//==============================================================================
dart::dynamics::ConstSkeletonPtr Workspace::getPerson() const
{
  return mPerson;
}

dart::dynamics::ConstSkeletonPtr Workspace::getWheelchair() const
{
  return mWheelchair;
}
}
