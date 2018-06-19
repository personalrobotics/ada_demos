#ifndef WORKSPACE_H
#define WORKSPACE_H

#include <aikido/planner/World.hpp>
#include <feeding/util.hpp>
#include <ros/ros.h>

namespace feeding {

/// The Workspace deals with everything in the aikido world
/// that is not a robot or something perceived by the camera.
class Workspace
{

  aikido::planner::WorldPtr& world;
  dart::dynamics::SkeletonPtr plate, table, workspaceEnvironment,
      defaultFoodItem, tom;

  void addToWorld(
      dart::dynamics::SkeletonPtr& skeleton,
      std::string name,
      const Eigen::Isometry3d& robotPose,
      const ros::NodeHandle& nodeHandle);

public:
  /// Fills the aikido world with stuff.
  /// Only loads the defaultFoodItem if the demo is run in simulation (because
  /// otherwise we will perceive the food).
  /// Since the robotPose needs to be in the origin of the aikido world,
  /// the placement of all objects depends on the robotPose on the table.
  Workspace(
      aikido::planner::WorldPtr& world,
      const Eigen::Isometry3d& robotPose,
      bool adaReal,
      const ros::NodeHandle& nodeHandle);

  dart::dynamics::SkeletonPtr getPlate() const
  {
    return plate;
  }
  dart::dynamics::SkeletonPtr getTable() const
  {
    return table;
  }
  dart::dynamics::SkeletonPtr getWorkspaceEnvironment() const
  {
    return workspaceEnvironment;
  }
  dart::dynamics::SkeletonPtr getDefaultFoodItem() const
  {
    return defaultFoodItem;
  }
  dart::dynamics::SkeletonPtr getTom() const
  {
    return tom;
  }

  void deleteFood();
};
}

#endif
