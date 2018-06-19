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
      ros::NodeHandle nodeHandle);

  /// Some getters
  dart::dynamics::ConstSkeletonPtr getPlate() const;
  dart::dynamics::ConstSkeletonPtr getTable() const;
  dart::dynamics::ConstSkeletonPtr getWorkspaceEnvironment() const;
  dart::dynamics::SkeletonPtr getDefaultFoodItem() const;
  dart::dynamics::ConstSkeletonPtr getTom() const;
  dart::dynamics::ConstSkeletonPtr getWheelchair() const;

  void deleteFood();

private:
  aikido::planner::WorldPtr& world;
  dart::dynamics::SkeletonPtr plate, table, workspaceEnvironment,
      defaultFoodItem, tom, wheelchair;

  void addToWorld(
      dart::dynamics::SkeletonPtr& skeleton,
      const std::string& name,
      const Eigen::Isometry3d& robotPose,
      ros::NodeHandle nodeHandle);
};
}

#endif
