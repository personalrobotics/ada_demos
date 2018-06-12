#ifndef WORKSPACE_H
#define WORKSPACE_H

#include <feeding/util.hpp>
#include <aikido/planner/World.hpp>
#include <ros/ros.h>

namespace feeding {
  
class Workspace {

  aikido::planner::WorldPtr& world;
  dart::dynamics::SkeletonPtr plate, table, workspaceEnvironment, defaultFoodItem, tom;

  void addToWorld(dart::dynamics::SkeletonPtr& skeleton, std::string name, const Eigen::Isometry3d& robotPose, const ros::NodeHandle& nodeHandle);

public:
  Workspace(aikido::planner::WorldPtr& world, const Eigen::Isometry3d& robotPose, bool adaReal, const ros::NodeHandle& nodeHandle);

  dart::dynamics::SkeletonPtr getPlate() const {return plate;}
  dart::dynamics::SkeletonPtr getTable() const {return table;}
  dart::dynamics::SkeletonPtr getWorkspaceEnvironment() const {return workspaceEnvironment;}
  dart::dynamics::SkeletonPtr getDefaultFoodItem() const {return defaultFoodItem;}
  dart::dynamics::SkeletonPtr getTom() const {return tom;}

  void deleteFood();
};

}

#endif
