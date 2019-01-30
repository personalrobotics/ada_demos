#include "feeding/perception/Perception.hpp"
#include "feeding/util.hpp"

#include <algorithm>
#include <aikido/perception/ObjectDatabase.hpp>
#include <tf_conversions/tf_eigen.h>
#include <libada/util.hpp>

using ada::util::getRosParam;

namespace feeding {

//==============================================================================
Perception::Perception(
    aikido::planner::WorldPtr world,
    dart::dynamics::MetaSkeletonPtr adasMetaSkeleton,
    ros::NodeHandle nodeHandle,
    std::shared_ptr<TargetFoodRanker> ranker,
    float faceZOffset)
  : mWorld(world)
  , mNodeHandle(nodeHandle)
  , mTargetFoodRanker(ranker)
  , mFaceZOffset(faceZOffset)
{
  std::string detectorDataURI
      = getRosParam<std::string>("/perception/detectorDataUri", mNodeHandle);
  std::string referenceFrameName
      = getRosParam<std::string>("/perception/referenceFrameName", mNodeHandle);
  std::string foodDetectorTopicName = getRosParam<std::string>(
      "/perception/foodDetectorTopicName", mNodeHandle);
  std::string faceDetectorTopicName = getRosParam<std::string>(
      "/perception/faceDetectorTopicName", mNodeHandle);

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  mObjectDatabase = std::make_shared<aikido::perception::ObjectDatabase>(
      resourceRetriever, detectorDataURI);

  mFoodDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          mNodeHandle,
          foodDetectorTopicName,
          mObjectDatabase,
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *adasMetaSkeleton, referenceFrameName)));

  mFoodDetector->setObjectProjectionHeight(0.25);

  mFaceDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          mNodeHandle,
          faceDetectorTopicName,
          mObjectDatabase,
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *adasMetaSkeleton, referenceFrameName)));

  mPerceptionTimeout
      = getRosParam<double>("/perception/timeoutSeconds", mNodeHandle);
  mPerceivedFaceName
      = getRosParam<std::string>("/perception/faceName", mNodeHandle);
  mFoodNames
      = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
}

//==============================================================================
std::vector<TargetFoodItem> Perception::perceiveFood(
  const std::string& foodName)
{
  if (foodName != "" & (
    std::find(mFoodNames.begin(), mFoodNames.end(), foodName) == mFoodNames.end()))
  {
    std::stringstream ss;
    ss << "[" << foodName << "] is unknown." << std::endl;
    throw std::invalid_argument(ss.str());
  }

  // Detect items
  auto detectedItems = mFoodDetector->detectObjects(
    mWorld, ros::Duration(mPerceptionTimeout));

  std::vector<TargetFoodItem> detectedFoodItems;
  detectedFoodItems.reserve(detectedItems.size());
  for (const auto& item: detectedItems)
  {
    auto targetFoodItem = TargetFoodItem(item);
    if (foodName != "" && targetFoodItem.getName() != foodName)
      continue;
    detectedFoodItems.emplace_back(TargetFoodItem(item));
  }

  // Return sorted items
  return mTargetFoodRanker->sort(
    detectedFoodItems,
    getForqueTransform());
}

//==============================================================================
Eigen::Isometry3d Perception::perceiveFace()
{
  if (!mFaceDetector->detectObjects(mWorld, ros::Duration(mPerceptionTimeout)))
  {
    ROS_WARN("face perception failed");
    throw std::runtime_error("Face perception failed");
  }

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
          = getRosParam<double>("/feedingDemo/fixedFaceY", mNodeHandle);
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
  // return mObjectDatabase->mObjData["faceStatus"].as<bool>();
  ROS_WARN("Always returning true for isMouthOpen");
  return true;
}

//==============================================================================
void Perception::setFoodItemToTrack(const TargetFoodItem& target)
{
  mTargetFoodItem = target;
}

//==============================================================================
Eigen::Isometry3d Perception::getTrackedFoodItemPose()
{
  if (mTargetFoodItem.getName() == "")
    throw std::runtime_error("Target item not set.");

  auto item = mFoodDetector->detectObject(mWorld,
    mTargetFoodItem.getUID(),
    ros::Duration(mPerceptionTimeout));

  // Update the current target item.
  mTargetFoodItem = TargetFoodItem(item);
  return mTargetFoodItem.getPose();
}

} // namespace feeding
