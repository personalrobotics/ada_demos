#ifndef FEEDING_PERCEPTION_HPP_
#define FEEDING_PERCEPTION_HPP_

#include <Eigen/Dense>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>

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

  /// Detect all food items Updates output parameters with
  /// the item closest to the forque.
  /// returns The tranform of the detected food with shortest distance.
  boost::optional<Eigen::Isometry3d> perceiveAllFood();

  /// Gets food items from active perception ros nodes and adds their new
  /// MetaSkeletons to the aikido world. Updates output parameters with
  /// the item closest to the forque.
  /// \param[in] foodNameToFind Name of the food item to detect.
  /// \param[in] perceiveDepthPlane Ignore anything below certain depth.
  /// \param[out] foodTransform The transform of the detected food.
  /// \return True if a food item was found.
  boost::optional<Eigen::Isometry3d> perceiveFood(
    const std::string& foodNameToFind,
    bool perceiveDepthPlane = false);

  boost::optional<Eigen::Isometry3d> perceiveFace();

  /// Returns true if mouth is detected to be open.
  bool isMouthOpen();

  Eigen::Isometry3d getForqueTransform();

  Eigen::Isometry3d getCameraToWorldTransform();

  Eigen::Isometry3d getOpticalToWorld();

  void setFaceZOffset(float faceZOffset);

private:

  /// Returns distance between item and forque.
  /// \param[in] item Item to get distance for.
  double getDistanceFromForque(const Eigen::Isometry3d& item);

  tf::TransformListener mTFListener;
  aikido::planner::WorldPtr mWorld;
  ros::NodeHandle& mNodeHandle;

  std::unique_ptr<aikido::perception::PoseEstimatorModule> mFoodDetector;
  std::unique_ptr<aikido::perception::PoseEstimatorModule> mFaceDetector;
  std::shared_ptr<aikido::perception::ObjectDatabase> mObjectDatabase;

  Eigen::Isometry3d mLastPerceivedFoodTransform;
  float mFaceZOffset;

  double mPerceptionTimeout;
  std::string mFoodNameToPerceive;
  std::string mPerceivedFaceName;

  Eigen::Hyperplane<double, 3> mDepthPlane;

  std::vector<dart::dynamics::SimpleFramePtr> frames;
  std::vector<aikido::rviz::FrameMarkerPtr> frameMarkers;

  std::vector<std::string> mFoodNames;
};
}

#endif
