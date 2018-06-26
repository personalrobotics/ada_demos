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
  : nodeHandle(nodeHandle), world(world)
{

  addToWorld(plate, "plate", robotPose);
  addToWorld(table, "table", robotPose);
  addToWorld(person, "person", robotPose);
  addToWorld(
      workspaceEnvironment, "workspaceEnvironment", robotPose);
  addToWorld(wheelchair, "wheelchair", Eigen::Isometry3d::Identity());


  if (!adaReal)
  {
    addToWorld(defaultFoodItem, "defaultFoodItem", robotPose);
    defaultFoodItem->getRootBodyNode()->setCollidable(false);
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
      = getRosParam<std::string>("/" + name + "/urdfUri", nodeHandle);
  Eigen::Isometry3d pose
      = robotPose.inverse() * createIsometry(
                                  getRosParam<std::vector<double>>(
                                      "/" + name + "/pose", nodeHandle));
  skeleton = loadSkeletonFromURDF(resourceRetriever, urdfUri, pose);
  world->addSkeleton(skeleton);
}

//==============================================================================
void Workspace::deleteFood()
{
  world->removeSkeleton(defaultFoodItem);
}

//==============================================================================
dart::dynamics::ConstSkeletonPtr Workspace::getPlate() const
{
  return plate;
}

//==============================================================================
dart::dynamics::ConstSkeletonPtr Workspace::getTable() const
{
  return table;
}

//==============================================================================
dart::dynamics::ConstSkeletonPtr Workspace::getWorkspaceEnvironment() const
{
  return workspaceEnvironment;
}

//==============================================================================
dart::dynamics::SkeletonPtr Workspace::getDefaultFoodItem() const
{
  return defaultFoodItem;
}

//==============================================================================
dart::dynamics::ConstSkeletonPtr Workspace::getPerson() const
{
  return person;
}

dart::dynamics::ConstSkeletonPtr Workspace::getWheelchair() const
{
  return wheelchair;
}
}
