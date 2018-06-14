#include "feeding/Workspace.hpp"
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/util.hpp>

namespace feeding {

Workspace::Workspace(
    aikido::planner::WorldPtr& world,
    const Eigen::Isometry3d& robotPose,
    bool adaReal,
    const ros::NodeHandle& nodeHandle)
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
    std::string name,
    const Eigen::Isometry3d& robotPose,
    const ros::NodeHandle& nodeHandle)
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
}
