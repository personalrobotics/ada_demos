#include "feeding/perception/Perception.hpp"
#include "feeding/util.hpp"

#include <algorithm>
#include <aikido/perception/AssetDatabase.hpp>
#include <aikido/perception/DetectedObject.hpp>
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
    bool removeRotationForFood,
    bool projectFoodToTable)
  : mWorld(world)
  , mAdaMetaSkeleton(adaMetaSkeleton)
  , mNodeHandle(nodeHandle)
  , mTargetFoodRanker(ranker)
  , mRemoveRotationForFood(removeRotationForFood)
  , mProjectFoodToTable(projectFoodToTable)

{
  if (!mNodeHandle)
    throw std::invalid_argument("Ros nodeHandle is nullptr.");
  std::string detectorDataURI
      = getRosParam<std::string>("/perception/detectorDataUri", *mNodeHandle);
  std::string referenceFrameName = getRosParam<std::string>(
      "/perception/referenceFrameName", *mNodeHandle);
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
std::vector<std::unique_ptr<FoodItem>> Perception::perceiveFood(
    const std::string& foodName)
{
  if (foodName != ""
      & (std::find(mFoodNames.begin(), mFoodNames.end(), foodName)
         == mFoodNames.end()))
  {
    std::stringstream ss;
    ss << "[" << foodName << "] is unknown." << std::endl;
    throw std::invalid_argument(ss.str());
  }

  std::cout << "PerceiveFood" << std::endl;
  std::vector<std::unique_ptr<FoodItem>> detectedFoodItems;

  // Detect items
  std::vector<DetectedObject> detectedObjects;
  mFoodDetector->detectObjects(
      mWorld,
      ros::Duration(mPerceptionTimeout),
      ros::Time(0),
      &detectedObjects);

  std::cout << "Detected " << detectedObjects.size() << " " << foodName
            << std::endl;
  detectedFoodItems.reserve(detectedObjects.size());

  Eigen::Isometry3d forqueTF
      = mAdaMetaSkeleton->getBodyNode("j2n6s200_forque_end_effector")
            ->getWorldTransform();

  for (const auto& item : detectedObjects)
  {
    auto foodItem = mTargetFoodRanker->createFoodItem(item, forqueTF);

    //if (mRemoveRotationForFood)
    //{
    //  ROS_WARN_STREAM("REMOVING ROTATION");
      //removeRotation(foodItem.get());
  //}

    if (mProjectFoodToTable)
    {
     projectToTable(foodItem.get());
    }

    Eigen::Isometry3d foodPose(foodItem->getPose());
    std::cout << "Original food pose\n" << foodPose.matrix() << std::endl;

    std::cout << "PerceiveFood " << foodItem->getName() << " " << foodItem->getAction()->getRotationAngle() * 180 / 3.14 << std::endl;
    if (foodName != "" && foodItem->getName() != foodName)
      continue;
    detectedFoodItems.emplace_back(std::move(foodItem));
  }

  // sort
  mTargetFoodRanker->sort(detectedFoodItems);
  return detectedFoodItems;
}

//==============================================================================
Eigen::Isometry3d Perception::perceiveFace()
{
  std::vector<DetectedObject> detectedObjects;

  if (!mFaceDetector->detectObjects(
          mWorld,
          ros::Duration(mPerceptionTimeout),
          ros::Time(0),
          &detectedObjects))
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

      //fixed distance:
      double fixedFaceY
          = getRosParam<double>("/feedingDemo/fixedFaceY", *mNodeHandle);
      if (fixedFaceY > 0)
      {
        faceTransform.translation().y() = fixedFaceY;
      }
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
  std::cout << "Set Food Item To Track" << target->getName() << std::endl;
  mTargetFoodItem = target;
}

//==============================================================================
Eigen::Isometry3d Perception::getTrackedFoodItemPose()
{
  if (!mTargetFoodItem)
    throw std::runtime_error("Target item not set.");

  auto detectionResult
      = mFoodDetector->detectObjects(mWorld, ros::Duration(mPerceptionTimeout));

  if (!detectionResult)
    ROS_WARN("Failed to detect new update on the target object.");

  // Pose should've been updated since same metaSkeleton is shared.
  //if (mRemoveRotationForFood)
  //  removeRotation(mTargetFoodItem);

  if (mProjectFoodToTable)
    projectToTable(mTargetFoodItem);

  return mTargetFoodItem->getPose();
}

//==============================================================================
void Perception::removeRotation(const FoodItem* item)
{
  Eigen::Isometry3d foodPose(Eigen::Isometry3d::Identity());
  foodPose.translation() = item->getPose().translation();

  // Downcast Joint to FreeJoint
  dart::dynamics::FreeJoint* freejtptr
      = dynamic_cast<dart::dynamics::FreeJoint*>(item->getMetaSkeleton()->getJoint(0));

  if (freejtptr == nullptr)
  {
    dtwarn << "[Perception::removeRotation] Could not cast the joint "
              "of the body to a Free Joint so ignoring the object "
           << item->getName() << std::endl;
    return;
  }
  freejtptr->setTransform(foodPose);
}

//==============================================================================
void Perception::projectToTable(const FoodItem* item)
{
  std::cout << "Perception Project to table" << std::endl;
  Eigen::Isometry3d foodPose(item->getPose());
  std::cout << "Original food pose\n" << foodPose.matrix() << std::endl;
  auto rotation = foodPose.linear();
  double yaw = rotation.eulerAngles(2, 1, 0)[0];
  std::cout << "Rotation " << rotation.eulerAngles(2, 1, 0).transpose() << std::endl;
  foodPose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // foodPose.translation()[2] = 0.22;
  std::cout << "Projected food pose\n" << foodPose.matrix() << std::endl;

  // Downcast Joint to FreeJoint
  dart::dynamics::FreeJoint* freejtptr
      = dynamic_cast<dart::dynamics::FreeJoint*>(item->getMetaSkeleton()->getJoint(0));

  if (freejtptr == nullptr)
  {
    dtwarn << "[Perception::projectToTable] Could not cast the joint "
              "of the body to a Free Joint so ignoring the object "
           << item->getName() << std::endl;
    return;
  }
  freejtptr->setTransform(foodPose);
}

} // namespace feeding
