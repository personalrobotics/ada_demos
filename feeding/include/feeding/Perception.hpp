#ifndef FEEDING_PERCEPTION_H_
#define FEEDING_PERCEPTION_H_

#include <Eigen/Dense>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <ros/ros.h>
#include <libada/Ada.hpp>

namespace feeding {

/// The Perception class is responsible for everything that has to do with the
/// camera.
/// Currently, this means that it calls the detectObjects function in aikido,
/// which adds some objects to the aikido world. The Perception class is also
/// responsible for dealing with those objects.
class Perception
{

public:
  Perception(
      aikido::planner::WorldPtr world,
      dart::dynamics::ConstMetaSkeletonPtr adasMetaSkeleton,
      ros::NodeHandle nodeHandle);

  /// Gets food items from active perception ros nodes and adds their new
  /// MetaSkeletons to the aikido world.
  /// If a food item is found, the foodTransform is assigned.
  /// Returns if a food item was found.
  bool perceiveFood(Eigen::Isometry3d& foodTransform);

private:
  aikido::planner::WorldPtr world;
  ros::NodeHandle& nodeHandle;
  aikido::perception::UniqueConstPoseEstimatorModulePtr objDetector;
};
}

#endif
