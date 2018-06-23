#ifndef FEEDING_WORKSPACE_H_
#define FEEDING_WORKSPACE_H_

#include <aikido/planner/World.hpp>
#include <feeding/util.hpp>
#include <ros/ros.h>

namespace feeding {

/// The Workspace deals with everything in the aikido world
/// that is not a robot or something perceived by the camera.
class Workspace
{

public:
  /// Constructor of the Workspace.
  /// Fills the aikido world with stuff.
  /// Only loads the defaultFoodItem if the demo is run in simulation (because
  /// otherwise we will perceive the food).
  /// Since the robotPose needs to be in the origin of the aikido world,
  /// the placement of all objects depends on the robotPose on the table.
  /// \param[in] world The aikido world.
  /// \param[in] robotPose The pose of the robot relative to the workspace.
  /// \param[in] adaReal True if the real robot is used.
  /// \param[in] nodeHandle Handle of the ros node.
  Workspace(
      aikido::planner::WorldPtr world,
      const Eigen::Isometry3d& robotPose,
      bool adaReal,
      ros::NodeHandle nodeHandle);

  /// Gets the plate
  dart::dynamics::ConstSkeletonPtr getPlate() const;

  /// Gets the table
  dart::dynamics::ConstSkeletonPtr getTable() const;

  /// Gets the workspace environment
  dart::dynamics::ConstSkeletonPtr getWorkspaceEnvironment() const;

  /// Gets the default food item
  dart::dynamics::SkeletonPtr getDefaultFoodItem() const;

  /// Gets the mannequin
  dart::dynamics::ConstSkeletonPtr getTom() const;
  dart::dynamics::ConstSkeletonPtr getWheelchair() const;

  /// Removes the default food item from the world.
  void deleteFood();

private:
  aikido::planner::WorldPtr& world;
  dart::dynamics::SkeletonPtr plate;
  dart::dynamics::SkeletonPtr table;
  dart::dynamics::SkeletonPtr workspaceEnvironment;
  dart::dynamics::SkeletonPtr defaultFoodItem;
  dart::dynamics::SkeletonPtr tom;
  dart::dynamics::SkeletonPtr wheelchair;

  /// Takes a skeleton pointer, fills it with a new skeleton and adds that to
  /// the world.
  /// \param[out] skeleton The skeleton pointer where we want to store the
  /// loaded skeleton.
  /// \param[in] name The name of the object that should be loaded.
  /// \param[in] robotPose The pose of the robot relative to the workspace.
  /// \param[in] nodeHandle Handle of the ros node.
  void addToWorld(
      dart::dynamics::SkeletonPtr& skeleton,
      const std::string& name,
      const Eigen::Isometry3d& robotPose,
      ros::NodeHandle nodeHandle);
};
}

#endif
