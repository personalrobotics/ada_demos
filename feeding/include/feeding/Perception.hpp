#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <libada/Ada.hpp>
#include <aikido/perception/PoseEstimatorModule.hpp>

namespace feeding {

/// The Perception class is responsible for everything that has to do with the camera.
/// It adds perceived objects to the aikido world.
class Perception {

  aikido::planner::WorldPtr world;
  ros::NodeHandle& nodeHandle;
  std::unique_ptr<aikido::perception::PoseEstimatorModule> objDetector;

public:

  Perception(aikido::planner::WorldPtr world, ada::Ada& ada, ros::NodeHandle& nodeHandle);

  /// Gets food items from active perception ros nodes and adds their new MetaSkeletons to the aikido world.
  /// If a food item is found, the foodTransform is assigned.
  /// Returns if a food item was found.
  bool perceiveFoodClosest(Eigen::Isometry3d& foodTransform, Eigen::Isometry3d forqueTip);

};

}

#endif
