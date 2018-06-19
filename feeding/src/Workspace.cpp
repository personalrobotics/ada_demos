#include "feeding/Workspace.hpp"
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/util.hpp>

namespace feeding {

Workspace::Workspace(
    aikido::planner::WorldPtr& world,
    const Eigen::Isometry3d& robotPose,
    bool adaReal,
    ros::NodeHandle nodeHandle)
  : world(world)
{

  addToWorld(plate, "plate", robotPose, nodeHandle);
  addToWorld(table, "table", robotPose, nodeHandle);
  addToWorld(tom, "tom", robotPose, nodeHandle);
  addToWorld(
      workspaceEnvironment, "workspaceEnvironment", robotPose, nodeHandle);

  if (!adaReal)
  {
    addToWorld(defaultFoodItem, "defaultFoodItem", robotPose, nodeHandle);
    defaultFoodItem->getRootBodyNode()->setCollidable(false);
  }
}

void Workspace::addToWorld(
    dart::dynamics::SkeletonPtr& skeleton,
    const std::string& name,
    const Eigen::Isometry3d& robotPose,
    ros::NodeHandle nodeHandle)
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

void Workspace::deleteFood()
{
  if (defaultFoodItem)
  {
    world->removeSkeleton(defaultFoodItem);
  }
}



  dart::dynamics::ConstSkeletonPtr Workspace::getPlate() const
  {
    return plate;
  }
  
  dart::dynamics::ConstSkeletonPtr Workspace::getTable() const
  {
    return table;
  }

  dart::dynamics::ConstSkeletonPtr Workspace::getWorkspaceEnvironment() const
  {
    return workspaceEnvironment;
  }

  dart::dynamics::SkeletonPtr Workspace::getDefaultFoodItem() const
  {
    return defaultFoodItem;
  }

  dart::dynamics::ConstSkeletonPtr Workspace::getTom() const
  {
    return tom;
  }

}
