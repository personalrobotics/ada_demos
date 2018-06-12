#include "feeding/Workspace.hpp"
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/util.hpp>

namespace feeding {

Workspace::Workspace(aikido::planner::WorldPtr world, const Eigen::Isometry3d& robotPose, const ros::NodeHandle& nodeHandle) {

  ROS_INFO("Loading Workspace");

  addToWorld(plate, "plate", world, robotPose, nodeHandle);
  addToWorld(table, "table", world, robotPose, nodeHandle);
  addToWorld(defaultFoodItem, "defaultFoodItem", world, robotPose, nodeHandle);
  addToWorld(tom, "tom", world, robotPose, nodeHandle);
  addToWorld(workspaceEnvironment, "workspaceEnvironment", world, robotPose, nodeHandle);
}

void Workspace::addToWorld(dart::dynamics::SkeletonPtr& skeleton, std::string name, aikido::planner::WorldPtr world, const Eigen::Isometry3d& robotPose, const ros::NodeHandle& nodeHandle) {
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  std::string urdfUri = getRosParam<std::string>("/" + name + "/urdfUri", nodeHandle);
  Eigen::Isometry3d pose = robotPose.inverse() * createIsometry(getRosParam<std::vector<double>>("/" + name + "/pose", nodeHandle));
  skeleton = loadSkeletonFromURDF(resourceRetriever, urdfUri, pose);
  world->addSkeleton(skeleton);
}

}