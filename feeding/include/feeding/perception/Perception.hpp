#ifndef FEEDING_PERCEPTION_HPP_
#define FEEDING_PERCEPTION_HPP_

#include <memory>
#include <Eigen/Dense>
#include <aikido/perception/AssetDatabase.hpp>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <libada/Ada.hpp>

#include "feeding/FoodItem.hpp"
#include "feeding/ranker/ShortestDistanceRanker.hpp"
#include "feeding/ranker/TargetFoodRanker.hpp"

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
  /// \param[in] ranker Ranker to rank detected items.
  Perception(
      aikido::planner::WorldPtr world,
      dart::dynamics::MetaSkeletonPtr adaMetaSkeleton,
      const ros::NodeHandle* nodeHandle,
      std::shared_ptr<TargetFoodRanker> ranker
      = std::make_shared<ShortestDistanceRanker>(),
      float faceZOffset = 0.0);

  /// Gets food items of the name set by setFoodName
  /// from active perception ros nodes and adds their new
  /// MetaSkeletons to the aikido world. Updates output parameters with
  /// the item ranked highest by the ranker.
  /// \param[out] foodTransform The transform of the detected food.
  /// \return All food items on the plate of matching name.
  std::vector<std::unique_ptr<FoodItem>> perceiveFood(
      const std::string& foodName = "");

  void setFoodItemToTrack(FoodItem* target);

  /// Throws exception if target item is not set.
  Eigen::Isometry3d getTrackedFoodItemPose();

  Eigen::Isometry3d perceiveFace();

  /// Returns true if mouth is detected to be open.
  bool isMouthOpen();

  void setFaceZOffset();

private:
  tf::TransformListener mTFListener;
  aikido::planner::WorldPtr mWorld;
  const ros::NodeHandle* mNodeHandle;
  dart::dynamics::MetaSkeletonPtr mAdaMetaSkeleton;

  std::unique_ptr<aikido::perception::PoseEstimatorModule> mFoodDetector;
  std::unique_ptr<aikido::perception::PoseEstimatorModule> mFaceDetector;
  std::shared_ptr<aikido::perception::AssetDatabase> mAssetDatabase;

  std::shared_ptr<TargetFoodRanker> mTargetFoodRanker;
  FoodItem* mTargetFoodItem;

  float mFaceZOffset;
  std::string mPerceivedFaceName;
  std::vector<std::string> mFoodNames;

  double mPerceptionTimeout;
};

} // namespace feeding

#endif
