#ifndef FEEDING_PERCEPTION_HPP_
#define FEEDING_PERCEPTION_HPP_

#include <Eigen/Dense>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
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

  /// Gets food items from active perception ros nodes and adds their new
  /// MetaSkeletons to the aikido world.
  /// \param[out] foodTransform the transform of a food item that has been
  /// found.
  /// \return True if a food item was found.
  bool perceiveFood(
      Eigen::Isometry3d& foodTransform,
      bool perceiveDepthPlane,
      aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
      const std::string& foundFoodName = std::string(""),
      bool perceiveAnyFood = false);

  bool perceiveFace(Eigen::Isometry3d& faceTransform);

  /// Returns true if mouth is detected to be open.
  bool isMouthOpen();

  Eigen::Isometry3d getForqueTransform();

  Eigen::Isometry3d getCameraToWorldTransform();

  Eigen::Isometry3d getOpticalToWorld();

  bool setFoodName(std::string foodName);

  void setFaceZOffset(float faceZOffset);

private:
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

  Eigen::Hyperplane<double, 3> depthPlane;

  std::vector<dart::dynamics::SimpleFramePtr> frames;
  std::vector<aikido::rviz::FrameMarkerPtr> frameMarkers;

  std::vector<std::string> mFoodNames;
};
}

#endif
