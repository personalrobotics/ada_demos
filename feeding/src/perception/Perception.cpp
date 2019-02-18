#include "feeding/perception/Perception.hpp"
#include "feeding/util.hpp"

#include <algorithm>
#include <aikido/perception/DetectedObject.hpp>
#include <aikido/perception/AssetDatabase.hpp>
#include <tf_conversions/tf_eigen.h>
#include <libada/util.hpp>

#include "feeding/FoodItem.hpp"

using ada::util::getRosParam;
using aikido::perception::DetectedObject;

namespace feeding {

//==============================================================================
Perception::Perception(
    aikido::planner::WorldPtr world,
    dart::dynamics::MetaSkeletonPtr adaMetaSkeleton,
    const ros::NodeHandle* nodeHandle,
    std::shared_ptr<TargetFoodRanker> ranker,
    float faceZOffset)
  : mWorld(world)
  , mAdaMetaSkeleton(adaMetaSkeleton)
  , mNodeHandle(nodeHandle)
  , mTargetFoodRanker(ranker)
  , mFaceZOffset(faceZOffset)
{
  if (!mNodeHandle)
    throw std::invalid_argument("Ros nodeHandle is nullptr.");
  std::string detectorDataURI
      = getRosParam<std::string>("/perception/detectorDataUri", *mNodeHandle);
  std::string referenceFrameName
      = getRosParam<std::string>("/perception/referenceFrameName", *mNodeHandle);
  std::string foodDetectorTopicName = getRosParam<std::string>(
      "/perception/foodDetectorTopicName", *mNodeHandle);
  std::string faceDetectorTopicName = getRosParam<std::string>(
      "/perception/faceDetectorTopicName", *mNodeHandle);

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  mAssetDatabase = std::make_shared<aikido::perception::AssetDatabase>(
      resourceRetriever, detectorDataURI);

  mFoodDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          *mNodeHandle,
          foodDetectorTopicName,
          mAssetDatabase,
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *adaMetaSkeleton, referenceFrameName)));

  //mFoodDetector->setObjectProjectionHeight(0.25);

  mFaceDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          *mNodeHandle,
          faceDetectorTopicName,
          mAssetDatabase,
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *adaMetaSkeleton, referenceFrameName)));

  mPerceptionTimeout
      = getRosParam<double>("/perception/timeoutSeconds", *mNodeHandle);
  mPerceivedFaceName
      = getRosParam<std::string>("/perception/faceName", *mNodeHandle);
  mFoodNames
      = getRosParam<std::vector<std::string>>("/foodItems/names", *nodeHandle);

  if (!mTargetFoodRanker)
    throw std::invalid_argument("TargetFoodRanker not set for perception.");
}

//==============================================================================
std::vector<FoodItemWithActionScorePtr> Perception::perceiveFood(
  const std::string& foodName)
{
  if (foodName != "" & (
    std::find(mFoodNames.begin(), mFoodNames.end(), foodName) == mFoodNames.end()))
  {
    std::stringstream ss;
    ss << "[" << foodName << "] is unknown." << std::endl;
    throw std::invalid_argument(ss.str());
  }

  std::vector<FoodItemWithActionScorePtr> detectedFoodItems;

  // Detect items
  std::vector<DetectedObject> detectedObjects;
  mFoodDetector->detectObjects(
      mWorld, ros::Duration(mPerceptionTimeout), ros::Time(0), &detectedObjects);

  detectedFoodItems.reserve(detectedObjects.size());
  for (const auto& item: detectedObjects)
  {
    auto itemWithActionScore = mTargetFoodRanker->createFoodItemWithActionScore(item);

    if (foodName != "" && itemWithActionScore->getItem()->getName() != foodName)
      continue;
    detectedFoodItems.emplace_back(itemWithActionScore);
  }

  Eigen::Isometry3d forqueTF
    = mAdaMetaSkeleton->getBodyNode("j2n6s200_forque_end_effector")->getWorldTransform();

  // Return sorted items
  return mTargetFoodRanker->sort(
    detectedFoodItems,
    forqueTF
  );
}

//==============================================================================
Eigen::Isometry3d Perception::perceiveFace()
{
  std::vector<DetectedObject> detectedObjects;

  if (!mFaceDetector->detectObjects(mWorld,
        ros::Duration(mPerceptionTimeout), ros::Time(0), &detectedObjects))
  {
    ROS_WARN("face perception failed");
    throw std::runtime_error("Face perception failed");
  }

  // TODO: the needs to be updated
  // just choose one for now
  for (int skeletonFrameIdx = 0; skeletonFrameIdx < 5; skeletonFrameIdx++)
  {
    auto perceivedFace = mWorld->getSkeleton(
        mPerceivedFaceName + "_" + std::to_string(skeletonFrameIdx));
    if (perceivedFace != nullptr)
    {
      auto faceTransform = perceivedFace->getBodyNode(0)->getWorldTransform();

      // fixed distance:
      double fixedFaceY
          = getRosParam<double>("/feedingDemo/fixedFaceY", *mNodeHandle);
      if (fixedFaceY > 0)
      {
        faceTransform.translation().y() = fixedFaceY;
      }

      faceTransform.translation().z()
          = faceTransform.translation().z() + mFaceZOffset;
      ROS_INFO_STREAM(
          "perceived Face: "
          << faceTransform.translation().matrix().transpose());
      return faceTransform;
    }
  }
  ROS_WARN("face perception failed");
  throw std::runtime_error("Face perception failed");
}

//==============================================================================
bool Perception::isMouthOpen()
{
  // return mAssetDatabase->mObjData["faceStatus"].as<bool>();
  ROS_WARN("Always returning true for isMouthOpen");
  return true;
}

//==============================================================================
void Perception::setFoodItemToTrack(FoodItem* target)
{
  mTargetFoodItem = target;
}

//==============================================================================
Eigen::Isometry3d Perception::getTrackedFoodItemPose()
{
  if (!mTargetFoodItem)
    throw std::runtime_error("Target item not set.");

  DetectedObject object;
  auto detectionResult = mFoodDetector->detectObject(
    mWorld,
    mTargetFoodItem->getUid(),
    object,
    ros::Duration(mPerceptionTimeout));

  if (!detectionResult)
    ROS_WARN("Failed to detect new update on the target object.");

  // Pose should've been updated since same metaSkeleton is shared.
  return mTargetFoodItem->getPose();
}

} // namespace feeding
