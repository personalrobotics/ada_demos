#ifndef FEEDING_PERCEPTION_HPP_
#define FEEDING_PERCEPTION_HPP_

#include <Eigen/Dense>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <libada/Ada.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "feeding/ranker/TargetFoodRanker.hpp"
#include "feeding/FoodItem.hpp"

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
      dart::dynamics::MetaSkeletonPtr adasMetaSkeleton,
      ros::NodeHandle nodeHandle,
      std::shraed_ptr<TargetFoodRanker> ranker,
      float faceZOffset = 0.0);

  /// Gets food items of the name set by setFoodName
  /// from active perception ros nodes and adds their new
  /// MetaSkeletons to the aikido world. Updates output parameters with
  /// the item ranked highest by the ranker.
  /// \param[out] foodTransform The transform of the detected food.
  /// \return All food items on the plate of matching name.
  std::vector<FoodItemWithActionScorePtr> perceiveFood(
    const std::string& foodName = "") const;

  void setFoodItemToTrack(const FoodItem& target);

  /// Throws exception if target item is not set.
  Eigen::Isometry3d getTrackedFoodItemPose();

  Eigen::Isometry3d perceiveFace();

  /// Returns true if mouth is detected to be open.
  bool isMouthOpen();

  void setFaceZOffset();

private:

  tf::TransformListener mTFListener;
  aikido::planner::WorldPtr mWorld;
  ros::NodeHandle& mNodeHandle;

  std::unique_ptr<aikido::perception::PoseEstimatorModule> mFoodDetector;
  std::unique_ptr<aikido::perception::PoseEstimatorModule> mFaceDetector;
  std::shared_ptr<aikido::perception::ObjectDatabase> mObjectDatabase;

  std::shared_ptr<TargetFoodRanker> mTargetFoodRanker;
  FoodItem mFoodItem;

  float mFaceZOffset;
  std::string mPerceivedFaceName;
  std::vector<std::string> mFoodNames;
};

} // namespace feeding

#endif
