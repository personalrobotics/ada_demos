#ifndef FEEDING_PERCEPTION_HPP_
#define FEEDING_PERCEPTION_HPP_

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
  /// Constructor
  /// Mainly sets up the objDetector
  /// \param[in] world The aikido world.
  /// \param[in] adasMetaSkeleton Ada's MetaSkeleton.
  /// \param[in] nodeHandle Handle of the ros node.
  Perception(
      aikido::planner::WorldPtr world,
      dart::dynamics::MetaSkeletonPtr adasMetaSkeleton,
      ros::NodeHandle nodeHandle);

  /// Gets food items from active perception ros nodes and adds their new
  /// MetaSkeletons to the aikido world.
  /// \param[out] foodTransform the transform of a food item that has been
  /// found.
  /// \return True if a food item was found.
  bool perceiveFood(Eigen::Isometry3d& foodTransform);

  bool perceiveFace(Eigen::Isometry3d& faceTransform);

  bool isMouthOpen();

private:
  aikido::planner::WorldPtr mWorld;
  ros::NodeHandle& mNodeHandle;
  std::unique_ptr<aikido::perception::PoseEstimatorModule> mFoodDetector;
  std::unique_ptr<aikido::perception::PoseEstimatorModule> mFaceDetector;
  std::shared_ptr<aikido::perception::ObjectDatabase> mObjectDatabase;

  Eigen::Isometry3d mLastPerceivedFoodTransform = Eigen::Isometry3d::Identity();

  double mPerceptionTimeout;
  std::string mPerceivedFoodName, mPerceivedFaceName;
};
}

#endif
