#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <Eigen/Dense>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <ros/ros.h>
#include <libada/Ada.hpp>

namespace feeding {

/// The Perception class is responsible for everything that has to do with the
/// camera.
/// It adds perceived objects to the aikido world.
class Perception
{

public:
  Perception(
      aikido::planner::WorldPtr world,
      ada::Ada& ada,
      ros::NodeHandle& nodeHandle);

  /// Gets food items from active perception ros nodes and adds their new
  /// MetaSkeletons to the aikido world.
  /// If a food item is found, the foodTransform is assigned.
  /// Returns if a food item was found.
  bool perceiveFood(Eigen::Isometry3d& foodTransform);

private:
  aikido::planner::WorldPtr world;
  ros::NodeHandle& nodeHandle;
  std::unique_ptr<aikido::perception::PoseEstimatorModule> objDetector;
};
}

#endif
